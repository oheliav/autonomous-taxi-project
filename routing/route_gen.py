
#route_gen.py
from routing.graph_builder import CarlaGraph
import carla
import heapq

class RouteGenerator:
    def __init__(self, graph: CarlaGraph):
        self.graph = graph
    
    def _get_node_id_from_location(self, location):
        return self.graph.get_closest_node(location)

        
    def find_shortest_route(self, start_loc, end_loc, world=None, draw=True, draw_failed_path=True):
        start_id = self._get_node_id_from_location(start_loc)
        end_id = self._get_node_id_from_location(end_loc)

        print(f"🧭 Start Node ID: {start_id}")
        print(f"🏁 End Node ID: {end_id}")

        if start_id not in self.graph.graph:
            print("❌ Start node not in graph.")
            return None
        if end_id not in self.graph.graph:
            print("❌ End node not in graph.")
            return None

        visited = set()
        min_heap = [(0.0, start_id, [])]

        while min_heap:
            cost, current, path = heapq.heappop(min_heap)

            if current in visited:
                continue
            visited.add(current)

            new_path = path + [current]

            if current == end_id:
                print(f"✅ Path found with cost {cost}")

                if draw and world:
                    self._draw_route(new_path, world, carla.Color(255, 0, 0))  # Red route
                return new_path

            for neighbor_id, distance in self.graph.get_neighbors(current):
                if neighbor_id not in visited:
                    heapq.heappush(min_heap, (cost + distance, neighbor_id, new_path))

        print("❌ Dijkstra failed: no path.")

        if draw and draw_failed_path and world:
            print(f"👀 Drawing visited nodes to visualize explored area ({len(visited)} nodes)...")
            for node_id in visited:
                wp = self.graph.get_waypoint(node_id)
                if wp:
                    world.debug.draw_string(
                        wp.transform.location + carla.Location(z=0.3),
                        'C', draw_shadow=False,
                        color=carla.Color(255, 100, 100),
                        life_time=60.0
                    )

        return None

                    
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
