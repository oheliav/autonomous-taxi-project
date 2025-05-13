# carla_interface/scenario_controller.py

import carla
import random
import time

def initialize_scenario(client, map_name=None, vehicle_filter="vehicle.tesla.model3"):
    """
    Sets up the CARLA world, loads map if needed, spawns vehicle, and attaches Traffic Manager.
    Returns: world, traffic_manager, vehicle
    """
    # Load map if different
    current_map = client.get_world().get_map().name
    if map_name and map_name not in current_map:
        print(f"🔁 Loading map: {map_name}")
        client.load_world(map_name)
        time.sleep(1.0)  # Wait for world to load

    world = client.get_world()

    # Set synchronous mode
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    # Traffic Manager setup
    tm = client.get_trafficmanager(port=8000)
    tm.set_synchronous_mode(True)

    # Spawn ego vehicle
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter(vehicle_filter)[0]

    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("❌ No spawn points found.")
    spawn_point = random.choice(spawn_points)

    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    print(f"✅ Spawned vehicle at: {spawn_point.location}")

    return world, tm, vehicle
