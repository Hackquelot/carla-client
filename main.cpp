#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>

namespace cc = carla::client;
namespace cg = carla::geom;
namespace ci = carla::image;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;

#define EXPECT_TRUE(cond)                \
    if (!(cond)) {                       \
        throw std::runtime_error(#cond); \
    }

// Pick a random element from a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
    EXPECT_TRUE(range.size() > 0u);
    std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
    return range[dist(std::forward<RNG>(generator))];
}

int main() try {
    carla::time_duration timeout(30s);

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

    // Save the original settings
    auto original_settings = world.GetSettings();
    auto new_settings = original_settings;

    // Set the refresh rate to 30 frames per second (fps)
    new_settings.fixed_delta_seconds = 1.0 / 30.0;
    world.ApplySettings(new_settings, timeout);

    // Get the blueprint for a vehicle and a random spawn point
    auto vehicle_blueprint = blueprint_library->Find("vehicle.tesla.model3");
    auto spawn_point = RandomChoice(map->GetRecommendedSpawnPoints(), rng);

    // Spawn the vehicle in the world
    auto actor = world.SpawnActor(*vehicle_blueprint, spawn_point);
    std::cout << "Spawned: " << actor->GetDisplayId() << '\n';
    auto vehicle = boost::static_pointer_cast<cc::Vehicle>(actor);

    // Allow to control the vehicle and make it move
    cc::Vehicle::Control control;
    control.throttle = 0.5f;
    vehicle->ApplyControl(control);

    // RGB camera that captures the scenario as the car moves in the world
    auto rgb_camera_blueprint = blueprint_library->Find("sensor.camera.rgb");
    auto rgb_camera_transform = cg::Transform(cg::Location{-1.5f, 0.0f, 2.5f},
                                              cg::Rotation{-5.0f, 0.0f, 0.0f});
    auto rgb_camera = world.SpawnActor(*rgb_camera_blueprint,
                                       rgb_camera_transform, actor.get());
    auto camera = boost::static_pointer_cast<cc::Sensor>(rgb_camera);

    cv::Mat screen;
    // Call back to get a picture from the camera mounted in the vehicle
    camera->Listen([&](auto data) {
        // Transform the image to a format that OpenCV understands
        auto image = boost::static_pointer_cast<csd::Image>(data);
        EXPECT_TRUE(nullptr != image);
        auto view = ci::ImageView::MakeView(*image);
        screen = cv::Mat(view.height(), view.width(), CV_8UC4, &view(0, 0));
    });

    // Create a window to display the camera picture
    std::string window_name("C++ Carla Client");
    cv::namedWindow(window_name);
    int key = 0;

    while (true) {
        world.WaitForTick(timeout);
        // Render the camera picture on screen
        cv::imshow(window_name, screen);
        // If the key 'q' is pressed, terminate the execution of the client
        key = cv::waitKey(1);
        if ('q' == key || 'Q' == key) {
            break;
        }
    }

    // Remove the actors used and revert the world settings
    camera->Destroy();
    vehicle->Destroy();
    world.ApplySettings(original_settings, timeout);
    cv::destroyWindow(window_name);

    std::cout << "Execution finished." << std::endl;

    return 0;
} catch (const cc::TimeoutException &e) {
    std::cout << std::endl << e.what() << std::endl;

    return 1;
}
