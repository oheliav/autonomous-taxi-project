import carla  # type: ignore
import time

from routing.graph_builder import CarlaGraph  # note: your file is 'graph_builder.py'
from routing.route_gen import RouteGenerator
from carla_interface.taxi_agent import TaxiAgent
from core.fleet_manager import FleetManager

def main():
    #connect to client
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    #client.load_world("Town01")
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
    spawn_points = world.get_map().get_spawn_points()
    print(f"Found {len(spawn_points)} spawn points.")

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter("vehicle.tesla.model3")[0]
    vehicle = None
    for sp in spawn_points:
        vehicle = world.try_spawn_actor(vehicle_bp, sp)
        if vehicle:
            print(f"✅ Spawned vehicle at {sp.location}")
            break

    if not vehicle:
        print("❌ Still failed to spawn vehicle after trying all points.")

    spectator = world.get_spectator()
    transform = vehicle.get_transform()
    spectator.set_transform(carla.Transform(
        transform.location + carla.Location(z=50),
        carla.Rotation(pitch=45)
    ))

    # Init agent and traffic manager
    # get TM from the client
    tm = client.get_trafficmanager(8000)

    # create agent with (vehicle, world, traffic_manager)
    agent = TaxiAgent(vehicle, world, tm)


    # Define route
    start_loc = vehicle.get_location()
    end_loc = carla.Location(x=200, y=100, z=0)

    start_node = graph.get_closest_node(start_loc)
    end_node = graph.get_closest_node(end_loc)

    route_gen = RouteGenerator(graph, world)

    # ① get a route **by node IDs**
    start_id = graph.get_closest_node(start_loc)
    end_id   = graph.get_closest_node(end_loc)
    route    = route_gen.dijkstra(start_id, end_id, draw=True)

    # ② or (simpler) pass Locations directly
    # route = route_gen.dijkstra_locations(start_loc, end_loc)

    world.debug.draw_string(
    vehicle.get_location() + carla.Location(z=2),
    '🚗 VEHICLE',
    draw_shadow=False,
    color=carla.Color(255, 255, 0),  # Yellow
    life_time=1000.0,
    persistent_lines=True
)

    # Start
    world.debug.draw_string(
        start_loc + carla.Location(z=2),
        'START',
        draw_shadow=False,
        color=carla.Color(255, 0, 0),  # Red
        life_time=1000.0,
        persistent_lines=True
    )

    # End
    world.debug.draw_string(
        end_loc + carla.Location(z=2),
        'END',
        draw_shadow=False,
        color=carla.Color(0, 0, 255),  # Blue
        life_time=1000.0,
        persistent_lines=True
    )
    
    waypoints = [graph.get_waypoint(n) for n in route]

    for i in range(len(waypoints) - 1):
        wp1 = waypoints[i]
        wp2 = waypoints[i + 1]
        if wp1 and wp2:
            world.debug.draw_line(
                wp1.transform.location + carla.Location(z=0.5),
                wp2.transform.location + carla.Location(z=0.5),
                thickness=0.2,
                color=carla.Color(0, 255, 0),  # Green
                life_time=10000.0
            )

    if route:
        waypoints = [graph.get_waypoint(n) for n in route]
        total = agent.drive_route(waypoints)
        print(f"🚕  Taxi finished route in {total} s  ({len(route)} nodes)")
    else:
        print("❌  No route found.")

if __name__ == "__main__":
    main()
