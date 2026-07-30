#ifndef CONSTANTS_H
#define CONSTANTS_H

#define CAPTURE_WIDTH 1280
#define CAPTURE_HEIGHT 800
#define CANNY_THRESHOLD 400
#define MIN_AREA 15
#define MAX_AREA 500
#define MIN_COLOR_SIMILARITY 130.0
#define CIRCULARITY_THRESHOLD 0.3
#define CONVEXITY_THRESHOLD 0.8
#define AREA_THRESHOLD 0.7
#define RED cv::Scalar(0, 0, 255)
#define YELLOW cv::Scalar(0, 255, 255)
#define MAGENTA cv::Scalar(255, 0, 255)
#define CYAN cv::Scalar(255, 255, 0)
#define GREEN cv::Scalar(0, 255, 0)
#define BLUE cv::Scalar(255, 0, 0)
#define WHITE cv::Scalar(255, 255, 255)
#define ORANGE_HSV cv::Scalar(20, 200, 200)
#define ORANGE_HSV_LOWER cv::Scalar(12, 140, 160)
#define ORANGE_HSV_UPPER cv::Scalar(20, 255, 255)

#define X_TABLE_SIZE 2.74
#define Y_TABLE_SIZE 1.525

#define OBJECT_POINTS                                                          \
  {{0, Y_TABLE_SIZE, 0},                                                       \
   {X_TABLE_SIZE, Y_TABLE_SIZE, 0},                                            \
   {X_TABLE_SIZE, 0, 0},                                                       \
   {0, 0, 0},                                                                  \
   {X_TABLE_SIZE / 2, -0.1525, 0.15},                                          \
   {X_TABLE_SIZE / 2, Y_TABLE_SIZE + 0.1525, 0.15}}

#if defined(_WIN32) || defined(_WIN64)
#define PORT_NAME "COM8"
#elif defined(__linux__)
#define PORT_NAME "/dev/ttyUSB0"
#elif defined(__APPLE__)
#define PORT_NAME "/dev/tty.usbserial-FT0G1Q2B"
#endif

#define SHOULDER_ID 1
#define SHOULDER_PRIME_ID 2
#define SHOULDER_YAW_ID 3
#define ELBOW_ID 4
#define WRIST_ID 5

#endif
