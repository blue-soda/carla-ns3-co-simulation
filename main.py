import time
import threading
import json

from src.utils.carla_connector import connect_to_carla, spawn_vehicle, spawn_vehicles, set_autopilot, destroy_actors, follow_vehicle
from src.models.v2x_messages import CAMGenerator, DENMGenerator
from src.utils.carla_ns3_bridge import CarlaNs3Bridge
from config.settings import CARLA_HOST, CARLA_PORT, CARLA_TIMEOUT

def round_data(value, digits=2):
    if isinstance(value, dict):
        return {k: round_data(v, digits) for k, v in value.items()}
    elif isinstance(value, list):
        return [round_data(v, digits) for v in value]
    elif isinstance(value, float):
        return round(value, digits)
    return value

def collect_vehicle_data(world):
    """Collect position and velocity data for all vehicles"""
    vehicles = world.get_actors().filter('*vehicle*')
    vehicle_data = []
    
    for index, vehicle in enumerate(vehicles):
        transform = vehicle.get_transform()
        velocity = vehicle.get_velocity()
        heading = transform.rotation.yaw
        speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5

        vehicle_data.append({
            "id": index,
            "carla_id": vehicle.id,
            "position": {
                "x": transform.location.x,
                "y": transform.location.y,
                "z": transform.location.z
            },
            "velocity": {
                "x": velocity.x,
                "y": velocity.y,
                "z": velocity.z
            },
            "heading": heading,
            "speed": speed
        })

    vehicle_data = round_data(vehicle_data)
    
    print(f"Collected vehicle data: {json.dumps(vehicle_data, indent=2)}")

    return vehicle_data

def process_ns3_message(message):
    """Process messages received from ns-3"""
    print(f"Received from ns-3: {json.dumps(message, indent=2)}")
    
    # Here you would handle different types of messages from ns-3
    # For example, trigger vehicle behaviors based on received V2X messages

def main():
    print("Connecting to Carla simulator")
    client, world = connect_to_carla(CARLA_HOST, CARLA_PORT, CARLA_TIMEOUT)
    
    if not client or not world:
        print("Failed to connect to Carla. Make sure the simulator is running.")
        return
    
    print("Successfully connected to Carla simulator!")

    # Create bridge to ns-3
    bridge = CarlaNs3Bridge()
    bridge.start_receiver(callback=process_ns3_message)
    
    # Spawn vehicles
    ego_vehicle = spawn_vehicle(world, 'model3')
    if not ego_vehicle:
        print("Failed to spawn ego vehicle")
        bridge.stop()
        return
    
    spawn_vehicles(world, 2, ['cybertruck'])  # Reduced number for testing
    set_autopilot(world, True)
    
    # Initialize V2X message generators
    cam_generator = CAMGenerator(ego_vehicle)
    denm_generator = DENMGenerator(ego_vehicle)

    # Setup camera follow
    # stop_event = threading.Event()
    # thread = threading.Thread(target=follow_vehicle, args=(world, ego_vehicle, stop_event))
    # thread.start()

    try:
        while True:
            # Generate and broadcast CAM message
            # cam_message = cam_generator.broadcast()
            
            # Send CAM to ns-3
            # bridge.send_v2x_message(cam_message)
            
            
            # Collect all vehicle data and send to ns-3
            vehicle_data = collect_vehicle_data(world)
            bridge.send_vehicle_states(vehicle_data)
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    finally:
        try:
            # Clean up
            # stop_event.set()
            # thread.join(timeout=2)
            bridge.stop()
            destroy_actors(world)
            
            # Reset to asynchronous mode
            settings = world.get_settings()
            settings.synchronous_mode = False
            world.apply_settings(settings)
        except Exception as e:
            print(f"Error during cleanup: {e}")
    
    print("Simulation ended")

if __name__ == '__main__':
    main()
