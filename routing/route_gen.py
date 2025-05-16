
#route_gen.py
from routing.graph_builder import CarlaGraph
import carla
import heapq



def _draw_debug(world, graph, visited, start_id, end_id,
                path=None, reached=False,
                life_time=30.0):
    """Blue dots = visited.  Green = start.  Red = target.
    Cyan line  = final path if reached."""
    blue  = carla.Color(0,   0, 255)
    green = carla.Color(0, 255,   0)
    red   = carla.Color(255,  0,  0)
    cyan  = carla.Color(0, 255, 255)

    # visited nodes
    for nid in visited:
        wp = graph.get_waypoint(nid)
        if wp:
            world.debug.draw_string(wp.transform.location,
                                    'R', draw_shadow=False,
                                    color=blue, life_time=life_time)

    # start / end labels
    for nid, label, col in [(start_id, 'START', green),
                            (end_id,   'END',   red)]:
        wp = graph.get_waypoint(nid)
        if wp:
            world.debug.draw_string(wp.transform.location + carla.Location(z=0.5),
                                    label, draw_shadow=False,
                                    color=col, life_time=life_time)

    # final path line (if found)
    if reached and path and len(path) > 1:
        for a, b in zip(path[:-1], path[1:]):
            wpa = graph.get_waypoint(a).transform.location
            wpb = graph.get_waypoint(b).transform.location
            world.debug.draw_line(wpa + carla.Location(z=0.3),
                                wpb + carla.Location(z=0.3),
                                thickness=0.05,
                                color=cyan,
                                life_time=life_time)


class RouteGenerator:
    def __init__(self, graph: CarlaGraph, world):
        self.graph = graph
        self.world = world
    
    def _get_node_id_from_location(self, location):
        return self.graph.get_closest_node(location)
    
    # ---------------------------------------------------------------------------
    # Pass draw=True (default) if RouteGenerator has a `world` attribute
    # ---------------------------------------------------------------------------

    def dijkstra(self, start_id, end_id, draw=True):
        """Shortest‑path on CarlaGraph.  Draws visited nodes for debugging."""
        graph    = self.graph
        queue    = [(0, start_id)]
        distances = {start_id: 0}
        previous  = {}
        visited   = set()

        # ---------- main loop ---------------------------------------------------
        while queue:
            current_dist, current_id = heapq.heappop(queue)

            if current_id in visited:
                continue
            visited.add(current_id)

            if current_id == end_id:
                print(f"✅  reached destination node {end_id}")
                break

            for neighbor_id, weight in graph.get_neighbors(current_id):
                new_dist = current_dist + weight
                if neighbor_id not in distances or new_dist < distances[neighbor_id]:
                    distances[neighbor_id] = new_dist
                    previous[neighbor_id]  = current_id
                    heapq.heappush(queue, (new_dist, neighbor_id))

        # ---------- no path? ----------------------------------------------------
        if end_id not in previous:
            print(f"❌  no path — explored {len(visited):,} nodes")
            if draw and hasattr(self, "world"):
                _draw_debug(self.world, graph, visited, start_id, end_id,
                            reached=False)
            return []

        # ---------- reconstruct -------------------------------------------------
        path = []
        cur  = end_id
        while cur is not None:
            path.insert(0, cur)
            cur = previous.get(cur)

        print(f"✅  path found: {len(path):,} nodes  "
            f"total distance {distances[end_id]:.1f} m  "
            f"(explored {len(visited):,})")

        if draw and hasattr(self, "world"):
            _draw_debug(self.world, graph, visited, start_id, end_id,
                        path=path, reached=True)

        return path


    # ---------------------------------------------------------------------------
    # Helper: draw everything with CARLA’s debug API
    # ---------------------------------------------------------------------------

                    
    def generate_k_shortest_routes(self, start_loc, end_loc, k=3):
        start_id = self._get_node_id_from_location(start_loc)
        end_id = self._get_node_id_from_location(end_loc)

        base_route = self.find_shortest_route(start_loc, end_loc)
        if not base_route:
            print("❌ No base route found.")
            return []

        routes = [base_route]
        candidates = []

        for i in range(1, k):
            for j in range(len(base_route) - 1): 
                spur_node = base_route[j]
                root_path = base_route[:j + 1]

                modified_graph = self._copy_graph_with_removed_edges(routes, root_path)

                alt_route = self._dijkstra_custom(spur_node, end_id, modified_graph)
                if not alt_route:
                    continue

                full_route = root_path[:-1] + alt_route

                if all(full_route != r[1] for r in candidates) and full_route not in routes:
                    cost = self.compute_route_distance(full_route)
                    candidates.append((cost, full_route))

            if not candidates:
                break

            candidates.sort(key=lambda x: x[0])
            _, best_route = candidates.pop(0)
            routes.append(best_route)

        return routes


    def _copy_graph_with_removed_edges(self, existing_routes, root_path):
        from copy import deepcopy
        modified_graph = deepcopy(self.graph.graph)

        root_length = len(root_path)

        for route in existing_routes:
            if route[:root_length] == root_path and len(route) > root_length:
                node = route[root_length - 1]
                next_node = route[root_length]
                if node in modified_graph:
                    modified_graph[node] = [
                        (n, d) for (n, d) in modified_graph[node] if n != next_node
                    ]

        return modified_graph

    
    def _dijkstra_custom(self, start_id, end_id, custom_graph):
        visited = set()
        min_heap = [(0.0, start_id, [])]  # (cost, current_node, path_so_far)

        while min_heap:
            cost, current, path = heapq.heappop(min_heap)

            if current in visited:
                continue
            visited.add(current)

            new_path = path + [current]

            if current == end_id:
                return new_path

            for neighbor_id, distance in custom_graph.get(current, []):
                if neighbor_id not in visited:
                    heapq.heappush(min_heap, (cost + distance, neighbor_id, new_path))

        return None  # If no route found

    def _draw_route(self, node_path, world, color):
        for i in range(len(node_path) - 1):
            wp1 = self.graph.get_waypoint(node_path[i])
            wp2 = self.graph.get_waypoint(node_path[i + 1])
            if wp1 and wp2:
                world.debug.draw_line(
                    wp1.transform.location + carla.Location(z=0.3),
                    wp2.transform.location + carla.Location(z=0.3),
                    thickness=0.2,
                    color=color,
                    life_time=60.0
                )


    def compute_route_distance(self, route):
        total = 0.0
        for i in range(len(route) - 1):
            neighbors = self.graph.get_neighbors(route[i])
            for neighbor, dist in neighbors:
                if neighbor == route[i + 1]:
                    total += dist
                    break
        return total

# wrapper that lets you pass Locations directly
    def dijkstra_locations(self, start_loc: carla.Location, end_loc: carla.Location):
        s_id = self.graph.get_closest_node(start_loc)
        e_id = self.graph.get_closest_node(end_loc)
        if s_id is None or e_id is None:
            print("⚠️  start or end not on any drivable waypoint")
            return []
        return self.dijkstra(s_id, e_id)