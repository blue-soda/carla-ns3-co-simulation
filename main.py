import time
import threading

from src.utils.carla_connector import connect_to_carla, spawn_vehicle, spawn_vehicles, set_autopilot, destroy_actors, follow_vehicle
from src.models.v2x_messages import CAMGenerator, DENMGenerator
from config.settings import CARLA_HOST, CARLA_PORT, CARLA_TIMEOUT

def main():
    print("Connecting to Carla simulator...")
    client, world = connect_to_carla(CARLA_HOST, CARLA_PORT, CARLA_TIMEOUT)
    
    if not client or not world:
        print("Failed to connect to Carla. Make sure the simulator is running.")
        return
    
    print("Successfully connected to Carla simulator!")

    ego_vehicle = spawn_vehicle(world, 'model3')
    if not ego_vehicle:
        print("Failed to spawn ego vehicle")
        return
    
    spawn_vehicles(world, 50, ['cybertruck'])
    
    set_autopilot(world, True)
    
    cam_generator = CAMGenerator(ego_vehicle)
    denm_generator = DENMGenerator(ego_vehicle)

    stop_event = threading.Event()
    thread = threading.Thread(target=follow_vehicle, args=(world, ego_vehicle, stop_event))
    thread.start()

    try:
        while True:
            cam_generator.broadcast()

            denm_message = denm_generator.generate_message("accident", (0, 0), "Car crash")
            denm_generator.broadcast(denm_message)

            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    finally:
        try:
            # Signal the thread to stop
            stop_event.set()
            # Wait for the thread to finish (with timeout)
            thread.join(timeout=2)
            # Clean up all actors
            destroy_actors(world)
        except Exception as e:
            print(f"Error during cleanup: {e}")
    
    print("Simulation ended")

if __name__ == '__main__':
    main()
