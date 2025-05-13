# run_simulation.py

from carla_interface.scenario_controller import initialize_scenario
from carla_interface.taxi_agent import TaxiAgent
from routing.graph_builder import CarlaGraph
from routing.route_generator import RouteGenerator
from routing.evaluation import log_evaluation
from utils.helpers import load_driving_graph, save_driving_graph
from core.request_manager import RequestManager
from core.fleet_manager import FleetManager
from core.ai_router import AIRouter
from core.dispatcher import Dispatcher

def run_single_simulation():
    import carla
    import random

    # === SETUP ===
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world, tm, vehicle = initialize_scenario(client, map_name="Town04")

    spawn_points = world.get_map().get_spawn_points()
    request_manager = RequestManager(spawn_points)

    graph = CarlaGraph(world)
    graph.build_graph()

    route_gen = RouteGenerator(graph)
    driving_graph = load_driving_graph("data/driving_graph.json")

    ai_router = AIRouter(graph, world, driving_graph)
    fleet = FleetManager(vehicle)
    dispatcher = Dispatcher(fleet, ai_router)

    # === RIDE REQUEST ===
    pickup, dropoff = request_manager.generate_request()
    ride_request = type("RideRequest", (), {"pickup": pickup, "dropoff": dropoff})()

    # === DISPATCH ===
    taxi = dispatcher.dispatch(ride_request)
    if not taxi:
        print("No taxi dispatched.")
        return

    # === ROUTES ===
    routes = route_gen.generate_k_shortest_routes(pickup, dropoff, k=3)
    if not routes:
        print("No valid routes found.")
        return

    best_idx, etas = ai_router.predict_best_route(routes)
    selected_route = routes[best_idx]
    baseline_route = routes[0]

    # === DRIVE ===
    agent = TaxiAgent(world, vehicle, tm)
    actual_time = agent.drive_and_log_segments(
        [graph.get_waypoint(n) for n in selected_route],
        driving_graph
    )
    baseline_time = agent.drive_route([graph.get_waypoint(n) for n in baseline_route])

    # === LOG ===
    log_evaluation(
        ride_id=1,
        predicted_eta=etas[best_idx],
        actual_time=actual_time,
        baseline_time=baseline_time,
        selected_route=selected_route,
        baseline_route=baseline_route
    )

    save_driving_graph(driving_graph, "data/driving_graph.json")

    print("✅ Simulation complete.")

