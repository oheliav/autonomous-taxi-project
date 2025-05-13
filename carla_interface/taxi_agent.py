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
        self.tm.set_percentage_speed_difference(vehicle, 0)

    def drive_route(self, waypoints):
        """
        Drives a full list of waypoints using Traffic Manager and returns total time.
        """
        if not waypoints:
            return 0.0

        start_time = time.time()

        transforms = [wp.transform for wp in waypoints]
        destination = transforms[-1].location

        self.vehicle.set_autopilot(True, self.tm.get_port())
        self.tm.set_path(self.vehicle, transforms)

        while not self._reached(destination):
            self.world.tick()

        self.vehicle.set_autopilot(False)
        return round(time.time() - start_time, 2)

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

    def _reached(self, destination, threshold=3.0):
        current = self.vehicle.get_location()
        return current.distance(destination) < threshold

    def _id(self, wp):
        loc = wp.transform.location
        return (round(loc.x, 1), round(loc.y, 1))
