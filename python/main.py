import sys

sys.path.append('/home/roberto/CARLA/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg')

import carla
import cv2
import numpy as np
import random

def main():
    print("Starting client.")

    # Connect to the server
    client = carla.Client('localhost', 2000)
    
    # The version of the client and server must be the same
    assert client.get_client_version, client.get_server_version

    # Get the basic information from the server
    world = client.get_world()
    map = world.get_map()
    blueprint_library = world.get_blueprint_library()
    map_spawnpoints = map.get_spawn_points()
    
    # Save the original settings
    original_settings = world.get_settings()
    new_settings = original_settings

    # Set the refresh rate to 30 frames per second (fps)
    new_settings.fixed_delta_seconds = 1.0 / 30.0
    world.apply_settings(new_settings)

    # Spawn the vehicle in the world
    vehicle = world.spawn_actor(blueprint_library.find("vehicle.tesla.model3"), random.choice(map_spawnpoints))
    print("Spawned vehicle: {}".format(vehicle.id))

    # ToDo: For now set the autopilot in the vehicle, this will be changed later
    vehicle.set_autopilot()

    # RGB camera that captures the scenario as the car moves in the world
    rgb_camera = blueprint_library.find("sensor.camera.rgb")
    camera_sensor = world.spawn_actor(rgb_camera, carla.Transform(carla.Location(-1.5, 0.0, 2.5)), attach_to=vehicle)

    image_data = {'image': np.zeros((1, 1, 4))}
    # Callback to get a picture from the camera mounted in the vehicle
    def camera_callback(image, data_dict):
        data_dict['image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
    camera_sensor.listen(lambda image: camera_callback(image, image_data))

    # Create a window to display the camera's picture
    window_name = "Python Carla Client"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    key = 0

    while True:
        world.wait_for_tick()
        
        # Render the camera picture on the screen
        cv2.imshow(window_name, image_data['image'])
        # If the key 'q' is pressed or escape (esc), terminate the execution of the client
        key = cv2.waitKey(1)
        if key == ord('q') or key == ord('Q') or key == 27:
            break

    # Remove the vehicle, the sensors and revert the world settings
    camera_sensor.destroy()
    vehicle.destroy()
    world.apply_settings(original_settings)

    cv2.destroyWindow(window_name)

    print("Execution finished")

if __name__ == "__main__":
    sys.exit(main())
