# carla_interface/taxi_agent.py

import time
import carla

class TaxiAgent:
    def __init__(self, world, vehicle, traffic_manager):
        self.world = world
        self.vehicle = vehicle
        self.map = world.get_map()
        self.tm = traffic_manager

        # TM settings
        self.tm.set_synchronous_mode(True)
        self.tm.set_global_distance_to_leading_vehicle(2.5)


    def drive_and_log_segments(self, waypoints, driving_graph):
        """
        Drives the route segment by segment and logs delay into driving_graph.
        """
        if not waypoints:
            return 0.0

        total_time = 0.0

        self.vehicle.set_autopilot(True, self.tm.get_port())

        for i in range(len(waypoints) - 1):
            wp1 = waypoints[i]
            wp2 = waypoints[i + 1]
            segment_path = [wp1.transform, wp2.transform]

            id1 = self._id(wp1)
            id2 = self._id(wp2)

            self.tm.set_path(self.vehicle, segment_path)

            start = time.time()
            while True:
                self.world.tick()
                current = self.vehicle.get_location()
                if current.distance(wp2.transform.location) < 1.5:
                    break
            end = time.time()

            segment_time = round(end - start, 2)
            total_time += segment_time

            # update driving_graph
            if id1 not in driving_graph:
                driving_graph[id1] = {}
            if id2 not in driving_graph[id1]:
                driving_graph[id1][id2] = {"total_time": 0.0, "samples": 0}

            driving_graph[id1][id2]["total_time"] += segment_time
            driving_graph[id1][id2]["samples"] += 1

        self.vehicle.set_autopilot(False)
        return round(total_time, 2)


    def drive_route(self, waypoints):
        if not waypoints:
            return 0.0

        start_time = time.time()
        self.vehicle.set_autopilot(True, self.tm.get_port())

        for i in range(len(waypoints) - 1):
            current_wp = waypoints[i]
            next_wp = waypoints[i + 1]

            # Detect lane change
            if next_wp.lane_id != current_wp.lane_id:
                direction = next_wp.lane_id > current_wp.lane_id  # True = left, False = right
                print(f"↔️ Forcing lane change {'left' if direction else 'right'}")
                self.tm.force_lane_change(self.vehicle, direction)

            # Wait until close to the next waypoint
            while self._wp_distance(next_wp) > 2.0:
                self.world.tick()

        self.vehicle.set_autopilot(False)
        return round(time.time() - start_time, 2)

    def _wp_distance(self, wp):
        return self.vehicle.get_location().distance(wp.transform.location)
    
    def _reached(self, destination, threshold=3.0):
        vehicle_wp = self.map.get_waypoint(self.vehicle.get_location(), project_to_road=True)
        dest_wp = self.map.get_waypoint(destination, project_to_road=True)

        if not vehicle_wp or not dest_wp:
            return False

        dist = vehicle_wp.transform.location.distance(dest_wp.transform.location)
        return dist < threshold

    def _id(self, wp):
        loc = wp.transform.location
        return (round(loc.x, 1), round(loc.y, 1))
