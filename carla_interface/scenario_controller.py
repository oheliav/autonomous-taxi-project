from carla import Vehicle, Transform
import random

def spawn_background_vehicles(world, client, traffic_manager, count=20):
    blueprint_library = world.get_blueprint_library()
    vehicle_bps = blueprint_library.filter('vehicle.*')

    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)

    vehicles = []

    for i in range(min(count, len(spawn_points))):
        bp = random.choice(vehicle_bps)
        transform = spawn_points[i]
        try:
            vehicle = world.try_spawn_actor(bp, transform)
            if vehicle:
                vehicle.set_autopilot(True, traffic_manager.get_port())
                vehicles.append(vehicle)
        except:
            continue

    print(f"🚗 Spawned {len(vehicles)} background vehicles.")
    return vehicles
