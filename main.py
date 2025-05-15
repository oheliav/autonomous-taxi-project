import carla  # type: ignore
import time

from routing.graph_builder import CarlaGraph  # note: your file is 'graph_builder.py'
from routing.route_gen import RouteGenerator
from carla_interface.taxi_agent import TaxiAgent
from core.fleet_manager import FleetManager

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    client.load_world("Town01")
    world = client.get_world()
    map_name = world.get_map().name
    print(f"🗌️ Current map: {map_name}")

    print("🔄 Building waypoint graph...")
    graph = CarlaGraph(world, resolution=2.0)
    graph.build_graph()
    print(f"✅ Graph built with {len(graph.get_all_nodes())} nodes. Visualizing...")
    graph.visualize(color=(0, 255, 0), life_time=120.0)

    print("👀 Waypoints drawn. Press Ctrl+C to quit or wait.")
    try:
        for _ in range(5):
            time.sleep(1)
    except KeyboardInterrupt:
        print("📝 Stopped by user.")

    # Spawn vehicle
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter("vehicle.tesla.model3")[0]
    spawn_point = world.get_map().get_spawn_points()[0]
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)

    if vehicle is None:
        print("❌ Failed to spawn vehicle.")
        return

    # Init agent and traffic manager
    traffic_manager = client.get_trafficmanager()
    agent = TaxiAgent(world, vehicle, traffic_manager)

    # Optional: use FleetManager if scaling to multiple taxis
    fleet = FleetManager([vehicle])

    # Define route
    start_loc = vehicle.get_location()
    end_loc = carla.Location(x=80, y=20, z=0)

    start_node = graph.get_closest_node(start_loc)
    end_node = graph.get_closest_node(end_loc)

    route_gen = RouteGenerator(graph)
    route = route_gen.find_shortest_route(start_loc, end_loc, world=world, draw=True)

    if route:
        print(f"✅ Found route with {len(route)} nodes")
        waypoints = [graph.get_waypoint(n) for n in route]
        total_time = agent.drive_route(waypoints)
        print(f"🚕 Taxi finished route in {total_time:.2f} seconds")
    else:
        print("❌ No route found.")

if __name__ == "__main__":
    main()
