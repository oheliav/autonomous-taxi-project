# main.py
from carla_interface.scenario_controller import ScenarioController
from core.request_manager import RequestManager
from core.dispatcher import Dispatcher
from core.fleet_manager import FleetManager
from core.ai_router import AIRouter
from core.logger import Logger

def main():
    print("🚕 Starting Autonomous Taxi Simulation...")

    # Setup environment and scenario
    scenario = ScenarioController()
    scenario.setup_scenario()

    # Initialize core modules
    fleet_manager = FleetManager()
    ai_router = AIRouter()
    dispatcher = Dispatcher(fleet_manager, ai_router)
    request_manager = RequestManager()
    logger = Logger()

    # Run simulation loop (simplified for now)
    while True:
        request = request_manager.get_next_request()
        if request:
            taxi = dispatcher.dispatch(request)
            if taxi:
                taxi.navigate_to(request.pickup, request.dropoff)
                logger.log_ride(taxi, request)
            else:
                print("No available taxi, queuing request.")
        else:
            break

    print("✅ Simulation complete.")

if __name__ == "__main__":
    main()
