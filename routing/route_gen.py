
#route_gen.py
from routing.graph_builer import CarlaGraph
import heapq

class RouteGenerator:
    def __init__(self, graph: CarlaGraph):
        self.graph = graph
    
    def _get_node_id_from_location(self, location):
        wp = self.graph.getwaypoint(location)
        return (round(wp.transform.location.x, 1), round(wp.transform.location.y, 1))
        
    def find_shortest_route(self, start_loc, end_loc):
        start_id = self._get_node_id_from_location(start_loc)
        end_id = self._get_node_id_from_location(end_loc)
        
        visited = set()
        min_heap = [(0.0), start_id, []]
        
        while min_heap:
            cost, current, path = heapq.heappop(min_heap)
            
            if current in visited:
                continue
            visited.add(current)
            
            new_path = path + [current]
            
            if (current == end_id):
                return new_path
                
            for neighbor_id, distance in self.graph.get_neighbors(current):
                if neighbor_id not in visited:
                    heapq.heappush(min_heap, (cost + distance, neighbor_id, new_path))
                    
    def generate_k_shortest_routes(self, start_loc, end_loc, k=3):
        start_id = self._get_node_id_from_location(start_loc)
        end_id = self._get_node_id_from_location(end_loc)
        
        base_route = self.find_shortest_route(start_loc, end_loc)
        if not base_route:
            return []
        
        routes = [base_route]
        candidates = []
        
        for i in range (1, k):
            for j in range(len(base_route) - 1):
                spur_node = base_route[j]
                root_path = base_route[:j + 1]
                
                modified_graph = self._copy_graph_with_removed_edges(root_path)
                
                alt_route = self._dijkstra_custom(spur_node, end_id, modified_graph)
                
                if not alt_route:
                    continue
                full_route = root_path[:-1] + alt_route
                
                if full_route not in routes and full_route not in candidates:
                    candidates.append(full_route)
                    
            if not candidates:
                break
            
            candidates.sort(key=self.compute_route_distance)
            best = candidates.pop(0)
            routes.append(best)
            
        return routes
        
    def _copy_graph_with_removed_edges(self, root_path):
        from copy import deepcopy
        modified_graph = deepcopy(self.graph.graph)
        
        for i in range (len(root_path) - 1):
            node = root_path[i]
            next_node = root_path[i+1]
            
            if node in modified_graph:
                modified+graph[node] = [(n, d) for (n, d) in modified_graph[node] if n != next_node]
                
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
