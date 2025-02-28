import carla
import random
from typing import Tuple, Optional

def connect_to_carla(host: str, port: int, timeout: float) -> Tuple[Optional[carla.Client], Optional[carla.World]]:
    """
    Connect to a running Carla simulator
    
    Args:
        host: Carla server host
        port: Carla server port
        timeout: Connection timeout in seconds
        
    Returns:
        Tuple of (client, world) if connection successful, (None, None) otherwise
    """
    try:
        client = carla.Client(host, port)
        client.set_timeout(timeout)
        world = client.get_world()
        return client, world
    except Exception as e:
        print(f"Error connecting to Carla: {e}")
        return None, None
        
def spawn_vehicle(world: carla.World, vehicle_type: str = None) -> Optional[carla.Vehicle]:
    """
    Spawn a vehicle in the world
    
    Args:
        world: Carla world
        vehicle_type: Type of vehicle to spawn (e.g., 'model3')
        
    Returns:
        Vehicle actor if spawned successfully, None otherwise
    """
    try:
        blueprint_library = world.get_blueprint_library()
        
        if vehicle_type:
            vehicle_bp = blueprint_library.filter(vehicle_type)[0]
        else:
            vehicle_bp = random.choice(blueprint_library.filter('vehicle'))
            
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        return vehicle
    except Exception as e:
        print(f"Error spawning vehicle: {e}")
        return None
