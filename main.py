import time

from src.utils.carla_connector import connect_to_carla, spawn_vehicle
from src.models.v2x_messages import CAMGenerator, DENMGenerator
from config.settings import CARLA_HOST, CARLA_PORT, CARLA_TIMEOUT

def main():
    print("Connecting to Carla simulator...")
    client, world = connect_to_carla(CARLA_HOST, CARLA_PORT, CARLA_TIMEOUT)
    
    if not client:
        print("Failed to connect to Carla. Make sure the simulator is running.")
        return
    
    print("Successfully connected to Carla simulator!")

    vehicle = spawn_vehicle(world, 'model3')

    cam_generator = CAMGenerator(vehicle)
    denm_generator = DENMGenerator(vehicle)

    try:
        while True:
            cam_generator.broadcast()

            denm_message = denm_generator.generate_message("accident", (0, 0), "Car crash")
            denm_generator.broadcast(denm_message)
        
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    
    print("Simulation ended")

if __name__ == '__main__':
    main()
