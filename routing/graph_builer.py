# graph_builder.py
from collections import defaultdict
import carla

class CarlaGraph:
    def __init__(self, world, resolution=2.0):
        self.world = world
        self.map = world.get_map()
        self.graph = defaultdict(list)
        self.node_loopup = {}
        self.resolution = resolution
        
    def build_graph(self):
        waypoints = self.map.generate_waypoints(self.resolution)
        
        for wp in waypoints:
            wp_id = self._id(wp)
            self.node_lookup[wp_id] = wp
            
            for next_wp in wp.next(self.resolution):
                next_id = self._id(next_wp)
                dist = wp.transform.location.distance(next_wp.transform.location)
                
                self.graph[wp_id].append((next_id, dist))
    def _id(self, wp):
        loc = wp.transform.location
        return (round(loc.x, 1), round(loc.y, 1))
        
    def get_neighbors(self, node_id):
        return self.graph.get(node_id, [])
    def get_waypoint(self, node_id):
        return self.node_loopup.get(node_id)
    def get_all_nodes(self):
        return list(self.graph.keys())
        
    def visualize(self, color=(0,255,0), life_time=30.0):
        for node_id in self.graph:
            wp = self.get_waypoint(node_id)
            if wp:
                self.world.debug.draw_string(
                    wp.transform.location, 'O', draw_shadow=False, color=carla.Color(*color), life_time=life_time)
            