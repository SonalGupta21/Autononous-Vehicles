import carla
import random
import time
import math

# --------------------------------------------------------
# Spawn road irregularities (potholes, bumps, dents)
# --------------------------------------------------------
def spawn_road_irregularities(world, blueprint_library):
    irregularities = []
    # Using small barriers to simulate bumps/potholes, but placing them flush with the road
    prop_bp = blueprint_library.find('static.prop.streetbarrier')
    spawn_points = world.get_map().get_spawn_points()

    for i in range(30):  # fewer than before
        loc = random.choice(spawn_points).location
        loc.z = 0.0  # flush with road
        transform = carla.Transform(loc, carla.Rotation(pitch=0, yaw=random.uniform(0, 360), roll=0))
        irregularities.append(world.spawn_actor(prop_bp, transform))
    return irregularities

# --------------------------------------------------------
# Spawn garbage piles
# --------------------------------------------------------
def spawn_garbage(world, blueprint_library):
    garbage_list = []
    garbage_bp = blueprint_library.find('static.prop.bin')
    spawn_points = world.get_map().get_spawn_points()

    for i in range(50):
        loc = random.choice(spawn_points).location
        loc.z = 0.0
        transform = carla.Transform(loc, carla.Rotation(yaw=random.uniform(0, 360)))
        garbage_list.append(world.spawn_actor(garbage_bp, transform))
    return garbage_list

# --------------------------------------------------------
# Spawn walking pedestrians
# --------------------------------------------------------
def spawn_pedestrians(world, blueprint_library, client):
    pedestrians = []
    walker_controllers = []

    spawn_points = []
    for i in range(300):
        spawn_point = carla.Transform()
        loc = world.get_random_location_from_navigation()
        if loc:
            spawn_point.location = loc
            spawn_points.append(spawn_point)

    walker_bp = blueprint_library.filter("walker.pedestrian.*")
    controller_bp = blueprint_library.find('controller.ai.walker')

    batch = []
    for spawn_point in spawn_points:
        walker_bp_choice = random.choice(walker_bp)
        batch.append(carla.command.SpawnActor(walker_bp_choice, spawn_point))

    results = client.apply_batch_sync(batch, True)
    walker_ids = [r.actor_id for r in results if not r.error]

    # Attach AI controllers
    batch = []
    for wid in walker_ids:
        batch.append(carla.command.SpawnActor(controller_bp, carla.Transform(), wid))

    results = client.apply_batch_sync(batch, True)
    controller_ids = [r.actor_id for r in results if not r.error]

    for controller_id in controller_ids:
        walker_controllers.append(world.get_actor(controller_id))

    # Make them walk randomly
    for controller in walker_controllers:
        controller.start()
        controller.go_to_location(world.get_random_location_from_navigation())
        controller.set_max_speed(1.5)  # normal walking speed

    return pedestrians, walker_controllers

# --------------------------------------------------------
# Spawn traffic (with collision avoidance)
# --------------------------------------------------------
def spawn_traffic(world, blueprint_library, traffic_manager):
    spawn_points = world.get_map().get_spawn_points()
    vehicle_bp = blueprint_library.filter('vehicle.*.*')
    vehicles = []

    for i in range(200):
        bp = random.choice(vehicle_bp)
        if bp.has_attribute('color'):
            bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))
        transform = random.choice(spawn_points)
        vehicle = world.try_spawn_actor(bp, transform)
        if vehicle:
            vehicles.append(vehicle)

    traffic_manager.set_global_distance_to_leading_vehicle(3.0)  # keep safe distance
    for vehicle in vehicles:
        vehicle.set_autopilot(True)
        traffic_manager.ignore_lights_percentage(vehicle, 0)  # obey signals
        traffic_manager.auto_lane_change(vehicle, True)
        traffic_manager.vehicle_percentage_speed_difference(vehicle, random.randint(-10, 0))
    return vehicles

# --------------------------------------------------------
# Main simulation
# --------------------------------------------------------
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.load_world('Town10HD_Opt')
    blueprint_library = world.get_blueprint_library()
    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_synchronous_mode(False)

    # Clean world
    for actor in world.get_actors():
        if actor.is_alive and actor.type_id.startswith(('vehicle.', 'walker.', 'static.prop')):
            actor.destroy()

    # Spawn environment
    print("Spawning road irregularities...")
    road_irregularities = spawn_road_irregularities(world, blueprint_library)
    print("Spawning garbage...")
    garbage = spawn_garbage(world, blueprint_library)

    print("Spawning pedestrians...")
    pedestrians, walker_controllers = spawn_pedestrians(world, blueprint_library, client)

    print("Spawning traffic...")
    vehicles = spawn_traffic(world, blueprint_library, traffic_manager)

    print("Simulation running... Press Ctrl+C to quit.")
    try:
        while True:
            world.wait_for_tick()
    except KeyboardInterrupt:
        print("\nCleaning up...")
    finally:
        for controller in walker_controllers:
            controller.stop()
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles + road_irregularities + garbage])

if __name__ == '__main__':
    main()
