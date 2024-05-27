#ifndef ROBOTIC_DIFFDRIVE_H
#define ROBOTIC_DIFFDRIVE_H

#include <string>

struct Config
{
    std::string left_wheel_name = "left_wheel";
    std::string right_wheel_name = "right_wheel";
    float loop_rate = 30;
    std::string device = "/dev/ttyUSB_STM";
    int baud_rate = 115200;
    int timeout = 500;
};

#endif // ROBOTIC_DIFFDRIVE_H