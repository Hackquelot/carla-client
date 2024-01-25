import random
import sys

import numpy
from common import *

sys.path.append(
    "/home/roberto/CARLA/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"
)

import carla


def main():
    print("Starting client.")

    # Connect to the server
    client = carla.Client("localhost", 2000)

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
    vehicle = world.spawn_actor(
        blueprint_library.find("vehicle.tesla.model3"), random.choice(map_spawnpoints)
    )
    print("Spawned vehicle: {}".format(vehicle.id))

    # ToDo: For now set the autopilot in the vehicle, this will be changed later
    vehicle.set_autopilot()

    # Spawn some vehicles to have traffic in the simulation
    counter = 0
    blueprint_vehicles = blueprint_library.filter("vehicle")
    for index in range(TRAFFIC_VEHICLES):
        # Try to spawn a random vehicle and on a random spawn point
        current_actor = world.try_spawn_actor(
            random.choice(blueprint_vehicles), random.choice(map_spawnpoints)
        )
        if None != current_actor:
            current_actor.set_autopilot()
            counter += 1
    print("Spawned {} vehicles for the traffic.".format(counter))

    # RGB camera that captures the scenario as the car moves in the world
    camera_sensor = world.spawn_actor(
        blueprint_library.find("sensor.camera.rgb"),
        carla.Transform(carla.Location(-1.5, 0.0, 2.5)),
        attach_to=vehicle,
    )

    # GPS sensor to know the position of the vehicle
    gnss_sensor = world.spawn_actor(
        blueprint_library.find("sensor.other.gnss"),
        carla.Transform(),
        attach_to=vehicle,
    )

    # Inertial Measurement Unit sensor
    imu_sensor = world.spawn_actor(
        blueprint_library.find("sensor.other.imu"), carla.Transform(), attach_to=vehicle
    )

    sensor_data = {"image": numpy.zeros((1, 1, 4))}

    # Callback to get a picture from the camera mounted in the vehicle
    def camera_callback(image, data_dict):
        data_dict["image"] = numpy.reshape(
            numpy.copy(image.raw_data), (image.height, image.width, 4)
        )

    camera_sensor.listen(lambda image: camera_callback(image, sensor_data))

    # Callback for the gnss data
    def gnss_callback(data, data_dict):
        data_dict["gnss"] = {
            "altitude": data.altitude,
            "latitude": data.latitude,
            "longitude": data.longitude,
        }

    gnss_sensor.listen(lambda data: gnss_callback(data, sensor_data))

    def imu_callback(data, data_dict):
        data_dict["imu"] = {
            "gyroscope": data.gyroscope,
            "accelerometer": data.accelerometer,
            "compass": data.compass,
        }

    # Callback for the imu sensor
    imu_sensor.listen(lambda data: imu_callback(data, sensor_data))

    # Create a window to display the camera's picture
    window_name = "Python Carla Client"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    key = 0

    while True:
        world.wait_for_tick()

        # Add some text to the image that will be displayed in screen
        drawBoxedText(
            sensor_data, "Altitude: " + str(sensor_data["gnss"]["altitude"]), (20, 20)
        )
        drawBoxedText(
            sensor_data, "Latitude: " + str(sensor_data["gnss"]["latitude"]), (20, 40)
        )
        drawBoxedText(
            sensor_data, "Longitude: " + str(sensor_data["gnss"]["longitude"]), (20, 60)
        )
        acceleration = sensor_data["imu"]["accelerometer"] - carla.Vector3D(0, 0, 9.81)
        drawBoxedText(
            sensor_data,
            "Acceleration: " + str(acceleration.length()),
            (20, 80),
        )
        drawBoxedText(
            sensor_data,
            "Gyroscope: " + str(sensor_data["imu"]["gyroscope"].length()),
            (20, 100),
        )
        drawCompass(sensor_data)

        # Render the camera picture on the screen
        cv2.imshow(window_name, sensor_data["image"])
        # If the key 'q' is pressed or escape (esc), terminate the execution of the client
        key = cv2.waitKey(1)
        if key == ord("q") or key == ord("Q") or key == 27:
            break

    # Remove the vehicle, the sensors and revert the world settings
    imu_sensor.destroy()
    gnss_sensor.destroy()
    camera_sensor.destroy()

    current_vehicles = world.get_actors().filter("*vehicle*")
    print("Current valid vehicles: {}".format(len(current_vehicles)))
    for current_vehicle in current_vehicles:
        current_vehicle.destroy()
    world.apply_settings(original_settings)

    cv2.destroyWindow(window_name)

    print("Execution finished")


if __name__ == "__main__":
    sys.exit(main())
