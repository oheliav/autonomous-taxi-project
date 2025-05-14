# graph_builder.py
from collections import defaultdict
import carla # type: ignore

class CarlaGraph:
    def __init__(self, world, resolution=2.0):
        self.world = world
        self.map = world.get_map()
        self.graph = defaultdict(list)
        self.node_lookup = {}
        self.resolution = resolution
            
    def build_graph(self):
        topology = self.map.get_topology()

        for entry_wp, exit_wp in topology:
            from_id = self._id(entry_wp)
            to_id = self._id(exit_wp)
            dist = entry_wp.transform.location.distance(exit_wp.transform.location)

            self.node_lookup[from_id] = entry_wp
            self.node_lookup[to_id] = exit_wp

            self.graph[from_id].append((to_id, dist))

    def _id(self, wp):
        loc = wp.transform.location
        return (round(loc.x, 2), round(loc.y, 2))

            
    def get_neighbors(self, node_id):
        return self.graph.get(node_id, [])
    
    def get_waypoint(self, node_id):
        return self.node_lookup.get(node_id)
    
    def get_all_nodes(self):
        return list(self.graph.keys())
        
    def visualize(self, color=(0,255,0), life_time=30.0):
        for node_id in self.graph:
            wp = self.get_waypoint(node_id)
            if wp:
                self.world.debug.draw_string(
                    wp.transform.location, 'O', draw_shadow=False, color=carla.Color(*color), life_time=life_time)
    
    def get_closest_node(self, location):
        min_dist = float('inf')
        closest_node = None

        for node_id in self.graph.keys():  # only use actual nodes with neighbors
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


