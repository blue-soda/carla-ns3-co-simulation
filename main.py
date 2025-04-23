import time
from src.carla.carla_connector import connect_to_carla, spawn_vehicle, spawn_vehicles, set_autopilot, destroy_actors, follow_vehicle
from src.bridge.carla_ns3_bridge import CarlaNs3Bridge
from src.common.logger import logger
from src.carla.vehicle_data import collect_vehicle_data
from config.settings import CARLA_HOST, CARLA_PORT, CARLA_TIMEOUT

def main():
    logger.info("Connecting to Carla simulator")
    client, world = connect_to_carla(CARLA_HOST, CARLA_PORT, CARLA_TIMEOUT)
    
    if not client or not world:
        logger.error("Failed to connect to Carla. Make sure the simulator is running.")
        return
    
    logger.info("Successfully connected to Carla simulator!")

    all_vehicles = []

    ego_vehicle = spawn_vehicle(world, 'coupe_2020')
    if not ego_vehicle:
        logger.error("Failed to spawn ego vehicle")
        bridge.stop()
        return

    all_vehicles.append(ego_vehicle)
    
    all_vehicles.extend(spawn_vehicles(world, 2, ['cooper_s']))

    set_autopilot(world, True)

    bridge = CarlaNs3Bridge()
    bridge.start()
    
    try:
        while bridge.is_simulation_running():
            vehicle_data = collect_vehicle_data(all_vehicles)
            bridge.send_vehicle_states(vehicle_data)
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        logger.info("Simulation interrupted by user")
    finally:
        try:
            bridge.stop()
            destroy_actors(world, all_vehicles)
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")
    
    logger.info("Simulation ended")

if __name__ == '__main__':
    main()
