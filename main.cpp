#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>

#include <iostream>
#include <random>

namespace cc = carla::client;
namespace cg = carla::geom;

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

    // Adding a dummy collision detector sensor to move the viewpoint
    auto collision_detector_blueprint =
        blueprint_library->Find("sensor.other.collision");
    auto collision_detector_transform = cg::Transform(
        cg::Location{-5.5f, 0.0f, 2.8f}, cg::Rotation{-15.0f, 0.0f, 0.0f});
    auto collision_detector =
        world.SpawnActor(*collision_detector_blueprint,
                         collision_detector_transform, actor.get());

    auto spectator = world.GetSpectator();
    // Show the vehicle for 500 frames
    for (int i = 0; i < 500; ++i) {
        world.WaitForTick(10min);
        // Update the viewport with the help of the spectator
        spectator->SetTransform(collision_detector->GetTransform());
    }

    // Remove the vehicle and set back the original settings
    vehicle->Destroy();
    std::cout << "Actor destroyed." << std::endl;
    world.ApplySettings(original_settings, timeout);

    return 0;
} catch (const cc::TimeoutException &e) {
    std::cout << std::endl << e.what() << std::endl;

    return 1;
}
