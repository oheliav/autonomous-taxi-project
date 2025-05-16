# routing/graph_builder.py  ⟵  completely rewritten
from collections import defaultdict
import carla
import math
from math import inf


class CarlaGraph:
    """
    Dense waypoint‑level graph:
      • node  = unique ID for every sampled waypoint
      • edges = (successor, distance)   for
          – each wp.next(resolution)           (forward, junctions, turns…)
          – left/right lane with same heading  (lateral moves)
      • bidirectional on the same lane so you can search both ways.
    """

    def __init__(self, world, resolution: float = 2.0) -> None:
        self.world       = world
        self.map         = world.get_map()
        self.resolution  = resolution
        self.graph       = defaultdict(list)   # node_id → List[(neigh_id, dist)]
        self.node_lookup = {}                  # node_id → waypoint

    # ---------- internal helpers ------------------------------------------------

    @staticmethod
    def _round(x: float, digits: int = 1) -> int:
        """coarse‑quantise to avoid FP duplicates but keep ~10 cm precision"""
        return int(round(x, digits) * 10**digits)

    def _id(self, wp: carla.Waypoint):
        loc = wp.transform.location
        return (self._round(loc.x), self._round(loc.y), wp.road_id, wp.lane_id)

    def _add_edge(self, from_id, to_id, dist: float, bidirectional: bool = False):
        self.graph[from_id].append((to_id, dist))
        if bidirectional:
            self.graph[to_id].append((from_id, dist))

    def _nearest_sample(self, wp):
        best_id   = None
        best_dist = inf
        for cand_id, cand_wp in self.node_lookup.items():
            if (cand_wp.road_id, cand_wp.lane_id) == (wp.road_id, wp.lane_id):
                d = cand_wp.transform.location.distance(wp.transform.location)
                if d < best_dist:
                    best_dist, best_id = d, cand_id
        return best_id if best_dist <= self.resolution else None
    # ---------- public API ------------------------------------------------------

    def build_graph(self):
        print("🔄   sampling map …")
        wps = self.map.generate_waypoints(self.resolution)
        print(f"🧩  {len(wps):,} waypoints sampled at {self.resolution} m")

        # 1) register every waypoint as a node
        for wp in wps:
            nid = self._id(wp)
            self.node_lookup[nid] = wp

        # 2) add forward / junction edges
        fwd_edges = 0
        for wp in wps:
            nid = self._id(wp)
            for nxt in wp.next(self.resolution):
                n_nid = self._id(nxt)
                dist  = wp.transform.location.distance(nxt.transform.location)
                self._add_edge(nid, n_nid, dist, bidirectional=True)
                fwd_edges += 1

        # 3) add lateral edges (same direction lanes only)
        lat_edges = 0
        for nid, wp in list(self.node_lookup.items()):
            for side in (wp.get_left_lane(), wp.get_right_lane()):
                if side                                 and \
                   side.lane_type == carla.LaneType.Driving and \
                   math.copysign(1, side.lane_id) == math.copysign(1, wp.lane_id):
                    sid   = self._id(side)
                    self.node_lookup[sid] = side
                    dist = wp.transform.location.distance(side.transform.location)
                    self._add_edge(nid, sid, dist, bidirectional=True)
                    lat_edges += 1
                    
        topo_turns = self.map.get_topology()
        jx_edges   = 0
        for entry_wp, exit_wp in topo_turns:
            # snap both ends to already‑sampled nodes if possible
            nid_from = self._nearest_sample(entry_wp) or self._id(entry_wp)
            nid_to   = self._nearest_sample(exit_wp)  or self._id(exit_wp)

            self.node_lookup.setdefault(nid_from, entry_wp)
            self.node_lookup.setdefault(nid_to,   exit_wp)

            dist = entry_wp.transform.location.distance(exit_wp.transform.location)
            self._add_edge(nid_from, nid_to, dist, bidirectional=False)
            jx_edges += 1

        print(f"↪️  junction‑turn edges: {jx_edges:,}")



        print(f"✅  graph built   nodes: {len(self.node_lookup):,}   "
              f"edges: forward {fwd_edges:,}  lateral {lat_edges:,}")

    # ---------- convenience -----------------------------------------------------

    def get_neighbors(self, node_id):
        return self.graph.get(node_id, [])

    def get_waypoint(self, node_id):
        return self.node_lookup.get(node_id)

    def get_all_nodes(self):
        return list(self.graph.keys())

    # optional visual helpers
    def visualize(self, color=(0, 255, 0), life_time=30.0):
        for wp in self.node_lookup.values():
            self.world.debug.draw_string(wp.transform.location,
                                         'O',
                                         draw_shadow=False,
                                         color=carla.Color(*color),
                                         life_time=life_time)

    def get_closest_node(self, location: carla.Location):
        """
        Return the ID of the waypoint‑node closest to the given CARLA Location.
        """
        closest_id   = None
        min_distance = float("inf")

        for node_id, wp in self.node_lookup.items():
            dist = location.distance(wp.transform.location)
            if dist < min_distance:
                min_distance = dist
                closest_id   = node_id

        return closest_id


