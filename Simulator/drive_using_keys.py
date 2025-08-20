import carla
import random
import math
import numpy as np
import cv2

# --------------------------------------------------------
# Spawn road irregularities
# --------------------------------------------------------
def spawn_road_irregularities(world, blueprint_library):
    irregularities = []
    prop_bp = blueprint_library.find('static.prop.streetbarrier')
    spawn_points = world.get_map().get_spawn_points()

    for i in range(30):
        loc = random.choice(spawn_points).location
        loc.z = 0.0
        transform = carla.Transform(
            loc, carla.Rotation(pitch=0, yaw=random.uniform(0, 360), roll=0)
        )
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
    for i in range(50):
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
        controller.set_max_speed(1.5)

    return pedestrians, walker_controllers

# --------------------------------------------------------
# Spawn traffic
# --------------------------------------------------------
def spawn_traffic(world, blueprint_library, traffic_manager):
    spawn_points = world.get_map().get_spawn_points()
    vehicle_bp = blueprint_library.filter('vehicle.*.*')
    vehicles = []

    for i in range(50):  # reduced to avoid lag
        bp = random.choice(vehicle_bp)
        if bp.has_attribute('color'):
            bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))
        transform = random.choice(spawn_points)
        vehicle = world.try_spawn_actor(bp, transform)
        if vehicle:
            vehicles.append(vehicle)

    traffic_manager.set_global_distance_to_leading_vehicle(3.0)
    for vehicle in vehicles:
        vehicle.set_autopilot(True)
        traffic_manager.ignore_lights_percentage(vehicle, 0)
        traffic_manager.auto_lane_change(vehicle, True)
        traffic_manager.vehicle_percentage_speed_difference(vehicle, random.randint(-10, 0))
    return vehicles

# --------------------------------------------------------
# Drive a car with keyboard input
# --------------------------------------------------------
def drive_with_keyboard(world, blueprint_library):
    # Spawn vehicle
    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)

    vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
    vehicle = None
    for spawn_point in spawn_points:
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle:
            print("Vehicle spawned successfully:", vehicle.type_id)
            break
    if not vehicle:
        raise RuntimeError("Could not spawn vehicle")

    # Attach camera
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute("image_size_x", '640')
    camera_bp.set_attribute("image_size_y", '360')
    camera_init_trans = carla.Transform(carla.Location(x=-5, z=2.5))
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)

    camera_data = {"image": np.zeros((360, 640, 4), dtype=np.uint8)}

    def camera_callback(image, data_dict):
        data_dict["image"] = np.reshape(
            np.copy(image.raw_data), (image.height, image.width, 4)
        )

    camera.listen(lambda image: camera_callback(image, camera_data))

    # Enable synchronous mode
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    print("Controls: W=Forward | S=Reverse/Brake | A=Left | D=Right | Q=Quit")

    cv2.namedWindow("Carla Camera", cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            world.tick()

            # Get speed
            v = vehicle.get_velocity()
            speed = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)

            # Default control
            throttle = 0.0
            brake = 0.0
            steer = 0.0
            reverse = False

            # Keyboard control
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            if key == ord("w"):   # forward
                throttle = 0.6
                reverse = False
            elif key == ord("s"): # reverse/brake
                throttle = 0.5
                reverse = True
            if key == ord("a"):   # left
                steer = -0.4
            elif key == ord("d"): # right
                steer = 0.4

            vehicle.apply_control(
                carla.VehicleControl(
                    throttle=throttle,
                    steer=steer,
                    brake=brake,
                    reverse=reverse,
                    hand_brake=False
                )
            )

            # Show camera
            img = camera_data["image"]
            img = cv2.putText(
                img,
                f"Speed: {int(speed)} km/h",
                (30, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
            cv2.imshow("Carla Camera", img)

    finally:
        print("Stopping vehicle and cleaning up...")
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        camera.stop()
        vehicle.destroy()
        camera.destroy()
        cv2.destroyAllWindows()
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)

# --------------------------------------------------------
# Main simulation
# --------------------------------------------------------
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.load_world('Town10HD_Opt')
    blueprint_library = world.get_blueprint_library()
    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_synchronous_mode(True)

    # Clean actors
    for actor in world.get_actors():
        if actor.is_alive and actor.type_id.startswith(('vehicle.', 'walker.', 'static.prop')):
            actor.destroy()

    print("Spawning environment...")
    road_irregularities = spawn_road_irregularities(world, blueprint_library)
    garbage = spawn_garbage(world, blueprint_library)
    pedestrians, walker_controllers = spawn_pedestrians(world, blueprint_library, client)
    vehicles = spawn_traffic(world, blueprint_library, traffic_manager)

    # Now drive with keyboard
    drive_with_keyboard(world, blueprint_library)


if __name__ == '__main__':
    main()
