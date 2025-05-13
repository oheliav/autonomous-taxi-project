# core/dispatcher.py

from routing.route_gen import RouteGenerator
from routing.graph_builer import CarlaGraph
from routing.extract_features import extract_features
from routing.ai_router import ETAEstimator

class Dispatcher:
    def __init__(self, fleet_manager, graph: CarlaGraph, route_generator: RouteGenerator, world, driving_graph):
        self.fleet_manager = fleet_manager
        self.graph = graph
        self.route_generator = route_generator
        self.world = world
        self.driving_graph = driving_graph
        self.model = ETAEstimator()

    def dispatch(self, ride_request):
        available_taxis = self.fleet_manager.get_available_taxis()

        if not available_taxis:
            print("❌ No taxis available for dispatch.")
            return None

        best_taxi = None
        best_eta = float('inf')

        for taxi in available_taxis:
            # Generate shortest path from taxi to pickup
            route = self.route_generator.find_shortest_route(
                taxi.get_location(), ride_request.pickup
            )

            if not route:
                continue

            waypoints = [self.graph.get_waypoint(nid) for nid in route]
            feature_vector = extract_features(waypoints, self.graph, self.world, self.driving_graph)
            eta = self.model.predict_eta(feature_vector)

            if eta < best_eta:
                best_eta = eta
                best_taxi = taxi

        if best_taxi:
            print(f"✅ Dispatching taxi {best_taxi.id} with ETA {best_eta:.2f} sec.")
            self.fleet_manager.mark_taxi_unavailable(best_taxi.id)
            return best_taxi
        else:
            print("⚠️ Could not find optimal taxi.")
            return None
