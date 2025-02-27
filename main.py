import carla
import random
import time

MAX_VEHICLES = 100

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.get_world()

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('model3')[0]

    for _ in range(MAX_VEHICLES):
        spawn_point = random.choice(world.get_map().get_spawn_points())
        try:
            vehicle = world.spawn_actor(vehicle_bp, spawn_point)
            print(f"Vehicle spawned: {vehicle.id}")
        except:
            print("Error while spawning vehicle")
            continue

        sleep_time = random.uniform(1, 2)
        time.sleep(sleep_time)    

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Script interrupted by user")