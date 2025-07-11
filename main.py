import carla  # type: ignore
import time

from routing.graph_builder import CarlaGraph  # note: your file is 'graph_builder.py'
from routing.route_gen import RouteGenerator
from carla_interface.taxi_agent import TaxiAgent
from core.fleet_manager import FleetManager
from carla_interface.scenario_controller import spawn_background_vehicles
import sys
sys.path.append(r"C:\Users\eliav\Desktop\Uni\Workshop\CARLA_0.9.11\WindowsNoEditor\PythonAPI\carla")

from agents.navigation.basic_agent import BasicAgent

def cleanup_actors(world):
    actors = world.get_actors()
    vehicles = actors.filter("vehicle.*")
    walkers = actors.filter("walker.*")

    to_destroy = list(vehicles) + list(walkers)

    print(f"💣 Destroying {len(to_destroy)} actors...")

    for actor in to_destroy:
        try:
            actor.destroy()
        except:
            pass

    print("✅ All agents cleaned up.")

def follow_vehicle(spectator, vehicle):
    transform = vehicle.get_transform()
    loc = transform.location + carla.Location(z=5) - transform.get_forward_vector() * 8
    rot = carla.Rotation(pitch=-15, yaw=transform.rotation.yaw)
    spectator.set_transform(carla.Transform(loc, rot))

def main():
    #connect to client
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    choise = 1

    #client.load_world("Town01")
    world = client.get_world()
    cleanup_actors(world)
    map_name = world.get_map().name
    print(f"🗌️ Current map: {map_name}")

    print("🔄 Building waypoint graph...")
    graph = CarlaGraph(world, resolution=2.0)
    graph.build_graph()
    print(f"✅ Graph built with {len(graph.get_all_nodes())} nodes. Visualizing...")
    graph.visualize(color=(0, 255, 0), life_time=15.0)

    # Spawn vehicle
    spawn_points = world.get_map().get_spawn_points()

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter("vehicle.tesla.model3")[0]
    vehicle = None
    
    for sp in spawn_points:
        vehicle = world.try_spawn_actor(vehicle_bp, sp)
        if vehicle:
            print(f"✅ Spawned vehicle at {sp.location}")
            start_transform = sp.location
            break

    if not vehicle:
        print("❌ Still failed to spawn vehicle after trying all points.")

    spectator = world.get_spectator()

    # Init agent and traffic manager
    # get TM from the client
    tm = client.get_trafficmanager(8000)
    # Setup Traffic Manager
    tm.set_synchronous_mode(True)

    # Spawn background traffic
    background_vehicles = spawn_background_vehicles(world, client, tm, count=30)

    # create agent with (vehicle, world, traffic_manager)
    #agent = TaxiAgent(vehicle, world, tm)
    for other in background_vehicles:
        if other.id != vehicle.id:
            tm.collision_detection(vehicle, other, True)


    # Define route
    start_loc = vehicle.get_location()
    end_loc = carla.Location(x=100, y=20, z=0)

    route_gen = RouteGenerator(graph, world)

    # ① get a route **by node IDs**
    start_id = graph.get_closest_node(start_loc)
    end_id   = graph.get_closest_node(end_loc)
    if (choise==1):
        route    = route_gen.dijkstra(start_id, end_id, draw=True)

    # ② or (simpler) pass Locations directly
    # route = route_gen.dijkstra_locations(start_loc, end_loc)

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
    if choise == 1:
        waypoints = [graph.get_waypoint(n) for n in route]
        # after you spawn the taxi vehicle
        from core.route_agent import RouteAwareAgent
        route_agent = RouteAwareAgent(vehicle, world)
        route_agent.load_route(waypoints)        # your graph’s way-points

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
            follow_vehicle(spectator, vehicle)
            start_time = time.time()
            while world.tick():
                if not route_agent.tick():
                    vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                    print("Taxi arrived at location: ", end_id)
                    
                    break
        else:
            print("❌  No route found.")
            
        end_time = time.time()
        elapsed = round(end_time - start_time, 2)
        print(f"⏱️  Custom path ETA: {elapsed} seconds")
    
    elif choise == 2:  
        agent = BasicAgent(vehicle)
        agent.set_destination((end_loc.x, end_loc.y, end_loc.z))

        start_time = time.time()

        follow_vehicle(world.get_spectator(), vehicle)
        while world.tick():
            control = agent.run_step()
            vehicle.apply_control(control)

            if agent.done():
                break


        end_time = time.time()
        elapsed = round(end_time - start_time, 2)
        print(f"⏱️  Native CARLA route ETA: {elapsed} seconds")
        print("Taxi arrived at location: ", end_id)



        
if __name__ == "__main__":
    main()
