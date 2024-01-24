#include "common.h"

#include <string>
#include <unordered_map>

namespace carla_client {
// Function to draw some text on screen with a solid background
void drawBoxedText(cv::Mat& image_data, const std::string& text, const cv::Point& point) {
    auto text_size = cv::getTextSize(text, FONT_FACE, FONT_SCALE, FONT_THICKNESS, nullptr);
    cv::rectangle(image_data, point, {point.x + text_size.width, point.y + text_size.height}, COLOR_BLACK, cv::FILLED);
    cv::putText(image_data, text, {point.x, static_cast<int32_t>(point.y + text_size.height + FONT_SCALE - 1)},
                FONT_FACE, FONT_SCALE, COLOR_WHITE, FONT_THICKNESS, LINE_TYPE);
}

// Variables needed to draw a compass in the camera image
static const std::pair<int32_t, int32_t> compass_center{700, 100};
constexpr int32_t compass_size = 50;
static const std::unordered_map<std::string, std::pair<int32_t, int32_t>> cardinal_directions{
    {"N", {0, -1}}, {"E", {1, 0}}, {"S", {0, 1}}, {"W", {-1, 0}}};

// Function to draw a compass over the camera image
void drawCompass(cv::Mat& image_data, const IMUData& imu_data) {
    for (const auto& cardinal_direction : cardinal_directions) {
        cv::putText(
            image_data, cardinal_direction.first,
            {static_cast<int32_t>(compass_center.first + 1.2 * compass_size * cardinal_direction.second.first),
             static_cast<int32_t>(compass_center.second + 1.2 * compass_size * cardinal_direction.second.second)},
            FONT_FACE, FONT_SCALE, COLOR_WHITE, FONT_THICKNESS, LINE_TYPE);
        cv::Point compass_point(
            static_cast<int32_t>(compass_center.first + compass_size * std::sin(imu_data.compass)),
            static_cast<int32_t>(compass_center.second + compass_size * std::cos(imu_data.compass)));
        cv::line(image_data, {compass_center.first, compass_center.second}, compass_point, COLOR_WHITE, 2);
    }
};
}  // namespace carla_client
