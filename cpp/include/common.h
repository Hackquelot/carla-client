#pragma once

#include <carla/client/Client.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>

#include <opencv2/opencv.hpp>

#define EXPECT_TRUE(cond)                \
    if (!(cond)) {                       \
        throw std::runtime_error(#cond); \
    }

// Shorcuts for the carla namespaces
namespace cc = carla::client;
namespace cg = carla::geom;
namespace ci = carla::image;
namespace csd = carla::sensor::data;

namespace client {
constexpr int32_t TRAFFIC_VEHICLES = 10;

// OpenCV constants for some colors
static const cv::Scalar COLOR_WHITE{255, 255, 255};
static const cv::Scalar COLOR_BLACK{0, 0, 0};

// OpenCV constants for the text characteristics
static const cv::HersheyFonts FONT_FACE = cv::FONT_HERSHEY_COMPLEX;
static const double FONT_SCALE = 0.5;
static const int32_t FONT_THICKNESS = 1;
static const cv::LineTypes LINE_TYPE = cv::LINE_8;

// Function to pick a random element from a given range
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
    EXPECT_TRUE(range.size() > 0u);
    std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
    return range[dist(std::forward<RNG>(generator))];
}

// Constant for the gravity
static const cg::Vector3D gravity = {0, 0, 9.81};
// Data type that holds the data from the IMU sensor
struct IMUData {
    cg::Vector3D accelerometer;
    cg::Vector3D gyroscope;
    float compass;
};

// Functions to draw some elements over the camera image
void drawBoxedText(cv::Mat &image_data, const std::string &text, const cv::Point &point);
void drawCompass(cv::Mat &image_data, const IMUData &imu_data);
}  // namespace client
