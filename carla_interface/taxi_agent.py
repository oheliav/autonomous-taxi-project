# carla_interface/taxi_agent.py   (works on CARLA 0.9.10‑0.9.12)
import math, time
import carla


# ---------------------------------------------------------------------------
# Local planner: very small pure‑pursuit controller
# ---------------------------------------------------------------------------

class PurePursuitFollower:
    def __init__(self, vehicle, world, route, lookahead=6.0, target_kph=25):
        self.vehicle    = vehicle
        self.world      = world
        self.route      = route[:]            # list[carla.Waypoint]
        self.lookahead  = lookahead
        self.target_v   = target_kph / 3.6    # m/s

    def _next_waypoint(self):
        if not self.route:
            return None
        loc = self.vehicle.get_transform().location
        while self.route and loc.distance(self.route[0].transform.location) < self.lookahead:
            self.route.pop(0)
        return self.route[0] if self.route else None

    def tick(self):
        wp = self._next_waypoint()
        if not wp:
            return False

        loc = self.vehicle.get_transform().location
        dx, dy = wp.transform.location.x - loc.x, wp.transform.location.y - loc.y

        yaw = math.radians(self.vehicle.get_transform().rotation.yaw)
        x_v =  math.cos(yaw)*dx + math.sin(yaw)*dy
        y_v = -math.sin(yaw)*dx + math.cos(yaw)*dy

        steer = 2.0 * y_v / (self.lookahead ** 2)
        steer = max(-1.0, min(1.0, steer))

        vel = self.vehicle.get_velocity()
        speed = math.hypot(vel.x, vel.y)
        throttle = 0.6 if speed < self.target_v else 0.0

        self.vehicle.apply_control(carla.VehicleControl(throttle=throttle,
                                                        steer=steer))
        return True


# ---------------------------------------------------------------------------
# TaxiAgent: optional TM for background cars, but ego is manual
# ---------------------------------------------------------------------------

class TaxiAgent:
    """
    • If you pass a TrafficManager (recommended) it will still run lights,
      basic collision avoidance, etc., but *not* plan a global route.
    • Ego path‑following is handled purely by PurePursuitFollower.
    """
    def __init__(self, vehicle: carla.Vehicle,
                       world: carla.World,
                       traffic_manager: carla.TrafficManager = None):
        self.vehicle = vehicle
        self.world   = world
        self.tm      = traffic_manager    # can be None

        # Keep ego under manual control; don't enable TM autopilot for it.
        self.vehicle.set_autopilot(False)

        # Tune background‑traffic behaviour if TM provided
        if self.tm:
            self.tm.global_percentage_speed_difference(-15)   # traffic ~15 % faster
            # (add more TM tuning here if needed)

        self.follower = None

    # ----------------------------------------------------------------------

    def drive_route(self, waypoints, do_sync_tick=False):
        if len(waypoints) < 2:
            return 0.0

        self.follower = PurePursuitFollower(self.vehicle, self.world, waypoints)
        t0 = self.world.get_snapshot().timestamp.elapsed_seconds

        while True:
            if do_sync_tick:
                self.world.tick()                     # 💡 advance the simulation manually
            else:
                self.world.wait_for_tick()

            if not self.follower.tick():
                break  # Route finished

        t1 = self.world.get_snapshot().timestamp.elapsed_seconds
        return round(t1 - t0, 2)
