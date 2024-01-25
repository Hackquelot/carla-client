import math

import cv2

TRAFFIC_VEHICLES = 10

COLOR_BLACK = (0, 0, 0)
COLOR_WHITE = (255, 255, 255)

FONT_FACE = cv2.FONT_HERSHEY_COMPLEX
FONT_SCALE = 0.5
FONT_THICKNESS = 1
LINE_TYPE = cv2.FILLED


# Function to draw some text on screen with a solid background
def drawBoxedText(sensor_data, text, point):
    w, h = cv2.getTextSize(text, FONT_FACE, FONT_SCALE, FONT_THICKNESS)[0]
    x, y = point
    cv2.rectangle(
        sensor_data["image"],
        point,
        (x + w, y + h),
        COLOR_BLACK,
        LINE_TYPE,
    )
    cv2.putText(
        sensor_data["image"],
        text,
        (x, int(y + h + FONT_SCALE - 1)),
        FONT_FACE,
        FONT_SCALE,
        COLOR_WHITE,
        FONT_THICKNESS,
        cv2.LINE_8,
    )


compass_center = (700, 100)
compass_size = 50
cardinal_directions = (("N", (0, -1)), ("E", (1, 0)), ("S", (0, 1)), ("W", (-1, 0)))


# Function to draw a compass over the camera image
def drawCompass(sensor_data):
    for cardinal_direction in cardinal_directions:
        cv2.putText(
            sensor_data["image"],
            cardinal_direction[0],
            (
                int(compass_center[0] + 1.2 * compass_size * cardinal_direction[1][0]),
                int(compass_center[1] + 1.2 * compass_size * cardinal_direction[1][1]),
            ),
            FONT_FACE,
            FONT_SCALE,
            COLOR_WHITE,
            FONT_THICKNESS,
            LINE_TYPE,
        )
        compass_point = (
            int(
                compass_center[0]
                + compass_size * math.sin(sensor_data["imu"]["compass"])
            ),
            int(
                compass_center[1]
                + compass_size * math.cos(sensor_data["imu"]["compass"])
            ),
        )
        cv2.line(sensor_data["image"], compass_center, compass_point, COLOR_WHITE, 2)
