from collections import defaultdict
import carla  # type: ignore

class CarlaGraph:
    def __init__(self, world, resolution=2.0):
        self.world = world
        self.map = world.get_map()
        self.graph = defaultdict(list)
        self.node_lookup = {}
        self.resolution = resolution

    def _id(self, wp):
        loc = wp.transform.location
        return (int(loc.x * 10), int(loc.y * 10), wp.lane_id)

    def build_graph(self):
        topology = self.map.get_topology()
        print(f"🧩 Loaded {len(topology)} topology connections")

        skipped_lateral = 0

        for entry_wp, exit_wp in topology:
            from_id = self._id(entry_wp)
            to_id = self._id(exit_wp)

            self.node_lookup[from_id] = entry_wp
            self.node_lookup[to_id] = exit_wp

            dist = entry_wp.transform.location.distance(exit_wp.transform.location)
            self.graph[from_id].append((to_id, dist))

        # Now add lateral links for all unique waypoints in node_lookup
        for wp_id, wp in self.node_lookup.items():
            # Right lane connection
            right_wp = wp.get_right_lane()
            if right_wp and right_wp.lane_type == carla.LaneType.Driving and wp.lane_id * right_wp.lane_id > 0:
                right_id = self._id(right_wp)
                self.node_lookup[right_id] = right_wp
                dist = wp.transform.location.distance(right_wp.transform.location)
                self.graph[wp_id].append((right_id, dist))
                self.graph[right_id].append((wp_id, dist))
            else:
                skipped_lateral += 1

            # Left lane connection
            left_wp = wp.get_left_lane()
            if left_wp and left_wp.lane_type == carla.LaneType.Driving and wp.lane_id * left_wp.lane_id > 0:
                left_id = self._id(left_wp)
                self.node_lookup[left_id] = left_wp
                dist = wp.transform.location.distance(left_wp.transform.location)
                self.graph[wp_id].append((left_id, dist))
                self.graph[left_id].append((wp_id, dist))
            else:
                skipped_lateral += 1

        print(f"↔️ Skipped {skipped_lateral} lateral connections due to invalid lanes")

        total_nodes = len(self.graph)
        disconnected = [nid for nid, nbrs in self.graph.items() if len(nbrs) == 0]
        print(f"🧠 Built graph with {total_nodes} nodes (from topology)")
        print(f"⚠️  {len(disconnected)} nodes have no outgoing connections")

    def get_neighbors(self, node_id):
        return self.graph.get(node_id, [])

    def get_waypoint(self, node_id):
        return self.node_lookup.get(node_id)

    def get_all_nodes(self):
        return list(self.graph.keys())

    def visualize(self, color=(0, 255, 0), life_time=30.0):
        for node_id in self.graph:
            wp = self.get_waypoint(node_id)
            if wp:
                self.world.debug.draw_string(
                    wp.transform.location,
                    'O',
                    draw_shadow=False,
                    color=carla.Color(*color),
                    life_time=life_time
                )

    def get_closest_node(self, location):
        min_dist = float('inf')
        closest_node = None

        for node_id in self.graph:
            wp = self.get_waypoint(node_id)
            if wp is None:
                continue
            dist = location.distance(wp.transform.location)
            if dist < min_dist:
                min_dist = dist
                closest_node = node_id

        return closest_node

    def print_graph_summary(self, limit=20):
        print(f"\n📊 Graph Summary: {len(self.graph)} nodes")
        print(f"{'Node':>20} -> Neighbors (count)")

        count = 0
        for node_id, neighbors in self.graph.items():
            print(f"{str(node_id):>20} -> {len(neighbors)} neighbors")
            count += 1
            if count >= limit:
                print("... (truncated)")
                break

    def print_edges(self, limit=20):
        print("\n🔗 Graph Edges (from → to, distance):")
        count = 0
        for from_id, neighbors in self.graph.items():
            for to_id, dist in neighbors:
                print(f"{from_id} → {to_id}   (dist={dist:.2f})")
                count += 1
                if count >= limit:
                    print("... (truncated)")
                    return
