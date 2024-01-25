#include <carla/client/ActorList.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/sensor/data/GnssMeasurement.h>
#include <carla/sensor/data/IMUMeasurement.h>

#include <cmath>
#include <iostream>
#include <random>
#include <unordered_map>

#include "common.h"

using namespace std::chrono_literals;

int main() try {
    carla::time_duration timeout(30s);

    std::cout << "Starting client." << std::endl;

    // Connect to the server
    auto client = cc::Client("localhost", 2000);
    client.SetTimeout(timeout);

    // The version of the client and server must be the same
    EXPECT_TRUE(client.GetClientVersion() == client.GetServerVersion());

    // A device to generate random numbers
    std::mt19937_64 rng((std::random_device())());

    // Get the basic information from the server
    auto world = client.GetWorld();
    auto map = world.GetMap();
    auto blueprint_library = world.GetBlueprintLibrary();
    auto map_spawnpoints = map->GetRecommendedSpawnPoints();

    // Save the original settings
    auto original_settings = world.GetSettings();
    auto new_settings = original_settings;

    // Set the refresh rate to 30 frames per second (fps)
    new_settings.fixed_delta_seconds = 1.0 / 30.0;
    world.ApplySettings(new_settings, timeout);

    // Spawn the vehicle in the world
    auto actor = world.SpawnActor(*(blueprint_library->Find("vehicle.tesla.model3")),
                                  client::RandomChoice(map_spawnpoints, rng));
    std::cout << "Spawned vehicle: " << actor->GetDisplayId() << '\n';
    auto vehicle = boost::static_pointer_cast<cc::Vehicle>(actor);

    // ToDo: For now set the autopilot in the vehicle, this will be changed later
    vehicle->SetAutopilot();

    // Spawn some vehicles to have traffic in the simulation
    {
        unsigned counter = 0;
        auto blueprint_vehicles = blueprint_library->Filter("vehicle");
        for (unsigned index = 0; index < client::TRAFFIC_VEHICLES; ++index) {
            // Try to spawn a random vehicle and on a random spawn point
            auto current_actor = world.TrySpawnActor(client::RandomChoice(*blueprint_vehicles, rng),
                                                     client::RandomChoice(map_spawnpoints, rng));
            if (nullptr != current_actor) {
                boost::static_pointer_cast<cc::Vehicle>(current_actor)->SetAutopilot();
                ++counter;
            }
        }
        std::cout << "Spawned " << counter << " vehicles for the traffic." << std::endl;
    }

    // RGB camera that captures the scenario as the car moves in the world
    auto rgb_camera =
        world.SpawnActor(*blueprint_library->Find("sensor.camera.rgb"),
                         cg::Transform(cg::Location{-1.5f, 0.0f, 2.5f}, cg::Rotation{-5.0f, 0.0f, 0.0f}), actor.get());
    auto camera_sensor = boost::static_pointer_cast<cc::Sensor>(rgb_camera);

    // GPS sensor to know the position of the vehicle
    auto gnss = world.SpawnActor(*blueprint_library->Find("sensor.other.gnss"), cg::Transform(), actor.get());
    auto gnss_sensor = boost::static_pointer_cast<cc::Sensor>(gnss);

    // Inertial Measurement Unit sensor
    auto imu = world.SpawnActor(*blueprint_library->Find("sensor.other.imu"), cg::Transform(), actor.get());
    auto imu_sensor = boost::static_pointer_cast<cc::Sensor>(imu);

    // ToDo: Add a mutex to avoid accessing the sensor's data with several threads

    cv::Mat image_data;
    // Callback to get a picture from the camera mounted in the vehicle
    camera_sensor->Listen([&](auto data) {
        // Transform the image to a format that OpenCV understands
        auto image = boost::static_pointer_cast<csd::Image>(data);
        EXPECT_TRUE(nullptr != image);
        auto view = ci::ImageView::MakeView(*image);
        image_data = cv::Mat(view.height(), view.width(), CV_8UC4, &view(0, 0));
    });

    // Callback for the gnss data
    cg::GeoLocation gnss_data;
    gnss_sensor->Listen([&](auto data) {
        auto gnss_measurement = boost::static_pointer_cast<csd::GnssMeasurement>(data);
        gnss_data = gnss_measurement->GetGeoLocation();
    });

    client::IMUData imu_data;
    // Callback for the imu sensor
    imu_sensor->Listen([&](auto data) {
        auto imu_measurement = boost::static_pointer_cast<csd::IMUMeasurement>(data);
        imu_data = {imu_measurement->GetAccelerometer(), imu_measurement->GetGyroscope(),
                    imu_measurement->GetCompass()};
    });

    // Create a window to display the camera's picture
    std::string window_name("C++ Carla Client");
    cv::namedWindow(window_name);
    int32_t key = 0;

    while (true) {
        world.WaitForTick(timeout);
        // Add some text to the image that will be displayed in screen
        {
            client::drawBoxedText(image_data, "Altitude: " + std::to_string(gnss_data.altitude), {20, 20});
            client::drawBoxedText(image_data, "Latitude: " + std::to_string(gnss_data.latitude), {20, 40});
            client::drawBoxedText(image_data, "Longitude: " + std::to_string(gnss_data.longitude), {20, 60});
            client::drawBoxedText(
                image_data, "Acceleration: " + std::to_string((imu_data.accelerometer - client::gravity).Length()),
                {20, 80});
            client::drawBoxedText(image_data, "Gyroscope: " + std::to_string(imu_data.gyroscope.Length()), {20, 100});
            client::drawCompass(image_data, imu_data);
        }
        // Render the camera picture on the screen
        cv::imshow(window_name, image_data);
        // If the key 'q' is pressed or escape (esc), terminate the execution of the client
        key = cv::waitKey(1);
        if ('q' == key || 'Q' == key || 27 == key) {
            break;
        }
    }

    // Remove the vehicle, the sensors and revert the world settings
    imu_sensor->Destroy();
    gnss_sensor->Destroy();
    camera_sensor->Destroy();
    auto current_vehicles = world.GetActors()->Filter("*vehicle*");
    std::cout << "Current valid vehicles: " << current_vehicles->size() << std::endl;
    for (unsigned index = 0; index < current_vehicles->size(); ++index) {
        current_vehicles->at(index)->Destroy();
    }
    world.ApplySettings(original_settings, timeout);

    cv::destroyWindow(window_name);

    std::cout << "Execution finished." << std::endl;

    return 0;
} catch (const cc::TimeoutException& e) {
    std::cout << std::endl << e.what() << std::endl;

    return 1;
}
