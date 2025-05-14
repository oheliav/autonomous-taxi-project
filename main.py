import carla # type: ignore
import time

from routing.graph_builder import CarlaGraph  # note the typo: your file is 'graph_builer.py'
from routing.route_gen import RouteGenerator

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    
    client.load_world("Town01") 
    world = client.get_world()
    map_name = world.get_map().name
    print(f"🗺️ Current map: {map_name}")
    
    print("🔄 Building waypoint graph...")
    graph = CarlaGraph(world, resolution=2.0)
    graph.build_graph()
    #graph.print_graph_summary(limit=50)
    #graph.print_edges(limit=100)


    print(f"✅ Graph built with {len(graph.get_all_nodes())} nodes. Visualizing...")

    graph.visualize(color=(0, 255, 0), life_time=120.0)

    print("👀 Waypoints drawn. Press Ctrl+C to quit or wait.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("🛑 Stopped by user.")
        
    # Define start/end locations (can be off-road)
    start_loc = carla.Location(x=10, y=10, z=0)
    end_loc = carla.Location(x=80, y=20, z=0)

    # Get nearest graph node IDs
    start_node = graph.get_closest_node(start_loc)
    end_node = graph.get_closest_node(end_loc)

    # Get their waypoint positions
    start_wp = graph.get_waypoint(start_node)
    end_wp = graph.get_waypoint(end_node)

    # Visualize start location
    world.debug.draw_string(start_loc + carla.Location(z=1),
                            'X', draw_shadow=False,
                            color=carla.Color(255, 0, 0), life_time=60.0, persistent_lines=True)

    # Visualize closest node to start
    if start_wp:
        world.debug.draw_string(start_wp.transform.location + carla.Location(z=1),
                                'o', draw_shadow=False,
                                color=carla.Color(255, 0, 0), life_time=60.0, persistent_lines=True)

    # Visualize end location
    world.debug.draw_string(end_loc + carla.Location(z=1),
                            'X', draw_shadow=False,
                            color=carla.Color(0, 0, 255), life_time=60.0, persistent_lines=True)

    # Visualize closest node to end
    if end_wp:
        world.debug.draw_string(end_wp.transform.location + carla.Location(z=1),
                                'o', draw_shadow=False,
                                color=carla.Color(0, 0, 255), life_time=60.0, persistent_lines=True)


    route_gen = RouteGenerator(graph)
    print("📍 From:", start_loc)
    print("📍 To:", end_loc)
    start_id = graph.get_closest_node(start_loc)
    end_id = graph.get_closest_node(end_loc)
    route = route_gen.find_shortest_route(
        start_loc, end_loc,
        world=world,
        draw=True,
        draw_failed_path=True
    )
    print(f"Neighbors of start node: {graph.get_neighbors(start_id)}")
    print(f"Neighbors of end node: {graph.get_neighbors(end_id)}")

    if route:
        print(f"✅ Found route with {len(route)} nodes")
        
    else:
        print("❌ No route found.")
        
    
    routes = route_gen.generate_k_shortest_routes(start_loc, end_loc, k=3)

    print(f"✅ Generated {len(routes)} route(s):")

    for r_index, route in enumerate(routes):
        print(f"🛣️ Route {r_index + 1}: {len(route)} nodes, distance={route_gen.compute_route_distance(route):.2f}")
        
        # Visualize each in a different color
        color = [(255, 0, 0), (0, 0, 255), (0, 255, 0)][r_index % 3]
        for i in range(len(route) - 1):
            wp1 = graph.get_waypoint(route[i])
            wp2 = graph.get_waypoint(route[i + 1])
            if wp1 and wp2:
                world.debug.draw_line(
                    wp1.transform.location + carla.Location(z=0.5),
                    wp2.transform.location + carla.Location(z=0.5),
                    thickness=0.2,
                    color=carla.Color(*color),
                    life_time=60.0
                )



if __name__ == "__main__":
    main()
