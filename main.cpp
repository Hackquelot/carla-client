#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/GnssMeasurement.h>
#include <carla/sensor/data/IMUMeasurement.h>
#include <carla/sensor/data/Image.h>

#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <unordered_map>

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
    std::cout << "Spawned vehicle: " << actor->GetDisplayId() << '\n';
    auto vehicle = boost::static_pointer_cast<cc::Vehicle>(actor);

    // Allow to control the vehicle and make it move
    cc::Vehicle::Control control;
    control.throttle = 0.5f;
    vehicle->ApplyControl(control);

    // RGB camera that captures the scenario as the car moves in the world
    auto rgb_camera =
        world.SpawnActor(*blueprint_library->Find("sensor.camera.rgb"),
                         cg::Transform(cg::Location{-1.5f, 0.0f, 2.5f},
                                       cg::Rotation{-5.0f, 0.0f, 0.0f}),
                         actor.get());
    auto camera_sensor = boost::static_pointer_cast<cc::Sensor>(rgb_camera);

    // GPS sensor to know the position of the vehicle
    auto gnss = world.SpawnActor(*blueprint_library->Find("sensor.other.gnss"),
                                 cg::Transform(), actor.get());
    auto gnss_sensor = boost::static_pointer_cast<cc::Sensor>(gnss);

    // Inertial Measurement Unit sensor
    auto imu = world.SpawnActor(*blueprint_library->Find("sensor.other.imu"),
                                cg::Transform(), actor.get());
    auto imu_sensor = boost::static_pointer_cast<cc::Sensor>(imu);

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
        auto gnss_measurement =
            boost::static_pointer_cast<csd::GnssMeasurement>(data);
        gnss_data = gnss_measurement->GetGeoLocation();
    });

    cg::Vector3D gravity = {0, 0, 9.81};
    struct {
        cg::Vector3D accelerometer;
        cg::Vector3D gyroscope;
        float compass;
    } imu_data;
    // Callback for the imu sensor
    imu_sensor->Listen([&](auto data) {
        auto imu_measurement =
            boost::static_pointer_cast<csd::IMUMeasurement>(data);
        imu_data = {imu_measurement->GetAccelerometer(),
                    imu_measurement->GetGyroscope(),
                    imu_measurement->GetCompass()};
    });

    // Some OpenCV constants
    const cv::Scalar COLOR_WHITE{255, 255, 255};
    const cv::Scalar COLOR_BLACK{0, 0, 0};

    const cv::HersheyFonts FONT_FACE = cv::FONT_HERSHEY_COMPLEX;
    const cv::LineTypes LINE_TYPE = cv::LINE_8;
    const double FONT_SCALE = 0.5;
    const int32_t FONT_THICKNESS = 1;

    std::pair<int32_t, int32_t> compass_center{700, 100};
    int32_t compass_size = 50;
    std::unordered_map<std::string, std::pair<int32_t, int32_t>>
        cardinal_directions{
            {"N", {0, -1}}, {"E", {1, 0}}, {"S", {0, 1}}, {"W", {-1, 0}}};

    auto drawBoxedText = [&](const std::string &text, const cv::Point &point) {
        auto text_size = cv::getTextSize(text, FONT_FACE, FONT_SCALE,
                                         FONT_THICKNESS, nullptr);
        cv::rectangle(image_data, point,
                      {point.x + text_size.width, point.y + text_size.height},
                      COLOR_BLACK, cv::FILLED);
        cv::putText(
            image_data, text,
            {point.x,
             static_cast<int32_t>(point.y + text_size.height + FONT_SCALE - 1)},
            FONT_FACE, FONT_SCALE, COLOR_WHITE, FONT_THICKNESS, LINE_TYPE);
    };

    auto drawCompass = [&]() {
        for (const auto &cardinal_direction : cardinal_directions) {
            cv::putText(
                image_data, cardinal_direction.first,
                {static_cast<int32_t>(compass_center.first +
                                      1.2 * compass_size *
                                          cardinal_direction.second.first),
                 static_cast<int32_t>(compass_center.second +
                                      1.2 * compass_size *
                                          cardinal_direction.second.second)},
                FONT_FACE, FONT_SCALE, COLOR_WHITE, FONT_THICKNESS, LINE_TYPE);
            cv::Point compass_point(
                static_cast<int32_t>(compass_center.first +
                                     compass_size * std::sin(imu_data.compass)),
                static_cast<int32_t>(compass_center.second +
                                     compass_size *
                                         std::cos(imu_data.compass)));
            cv::line(image_data, {compass_center.first, compass_center.second},
                     compass_point, COLOR_WHITE, 2);
        }
    };

    // Create a window to display the camera picture
    std::string window_name("C++ Carla Client");
    cv::namedWindow(window_name);
    int32_t key = 0;

    while (true) {
        world.WaitForTick(timeout);
        // Add some text to the image that will be displayed in screen
        {
            drawBoxedText("Altitude: " + std::to_string(gnss_data.altitude),
                          {20, 20});
            drawBoxedText("Latitude: " + std::to_string(gnss_data.latitude),
                          {20, 40});
            drawBoxedText("Longitude: " + std::to_string(gnss_data.longitude),
                          {20, 60});
            drawBoxedText(
                "Acceleration: " +
                    std::to_string((imu_data.accelerometer - gravity).Length()),
                {20, 80});
            drawBoxedText(
                "Gyroscope: " + std::to_string(imu_data.gyroscope.Length()),
                {20, 100});
            drawCompass();
        }
        // Render the camera picture on the screen
        cv::imshow(window_name, image_data);
        // If the key 'q' is pressed or escape (esc), terminate the execution of
        // the client
        key = cv::waitKey(1);
        if ('q' == key || 'Q' == key || 27 == key) {
            break;
        }
    }

    // Remove the vehicle and the sensors and revert the world settings
    imu_sensor->Destroy();
    gnss_sensor->Destroy();
    camera_sensor->Destroy();
    vehicle->Destroy();
    world.ApplySettings(original_settings, timeout);

    cv::destroyWindow(window_name);

    std::cout << "Execution finished." << std::endl;

    return 0;
} catch (const cc::TimeoutException &e) {
    std::cout << std::endl << e.what() << std::endl;

    return 1;
}
