import sys
sys.path.append(r"C:\Users\eliav\Desktop\Uni\Workshop\CARLA_0.9.11\WindowsNoEditor\PythonAPI\carla")

from agents.navigation.basic_agent import BasicAgent

class RouteAwareAgent:
    """Drives through a list of waypoints while obeying traffic rules and speed limits."""
    def __init__(self, vehicle, world):
        self.vehicle = vehicle
        self.agent   = BasicAgent(vehicle)
        self.world   = world
        self.route   = []


    def load_route(self, waypoints):
        STEP = 10  # Every ~10th waypoint becomes a sub-goal
        self.route = [wp.transform.location for idx, wp in enumerate(waypoints) if idx % STEP == 0]

        # Ensure final destination is included
        self.route.append(waypoints[-1].transform.location)

        self.next_subgoal()

    def next_subgoal(self):
        if not self.route:
            return False  # Route complete

        dest = self.route.pop(0)
        self.agent.set_destination((dest.x, dest.y, dest.z))

        speed_limit = self.vehicle.get_speed_limit()

        return True

    def tick(self):
        if self.agent.done():  # Finished current sub-goal
            if not self.next_subgoal():  # Try next
                return False  # All done

        control = self.agent.run_step()
        self.vehicle.apply_control(control)
        return True
