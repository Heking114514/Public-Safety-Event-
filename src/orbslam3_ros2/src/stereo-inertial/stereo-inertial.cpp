#include <iostream>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-inertial-node.hpp"

#include "System.h"

namespace
{
bool ParseBool(const char *value, bool defaultValue)
{
    if (value == nullptr)
        return defaultValue;

    std::string text(value);
    std::transform(text.begin(), text.end(), text.begin(), ::tolower);
    if (text == "true" || text == "1" || text == "yes" || text == "on")
        return true;
    if (text == "false" || text == "0" || text == "no" || text == "off")
        return false;
    return defaultValue;
}
}

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam3 stereo-inertial path_to_vocabulary path_to_settings "
                     "do_rectify [do_equalize] [visualization] [use_imu]" << std::endl;
        return 1;
    }

    const char* doEqualize = argc >= 5 ? argv[4] : "false";
    const bool visualization = argc >= 6 ? ParseBool(argv[5], true) : true;
    const bool useImu = argc >= 7 ? ParseBool(argv[6], true) : true;

    rclcpp::init(argc, argv);

    try
    {
        const ORB_SLAM3::System::eSensor sensor = useImu
            ? ORB_SLAM3::System::IMU_STEREO
            : ORB_SLAM3::System::STEREO;
        ORB_SLAM3::System slam(argv[1], argv[2], sensor, visualization);
        auto node = std::make_shared<StereoInertialNode>(&slam, argv[2], argv[3], doEqualize, useImu);
        rclcpp::spin(node);
        node.reset();
    }
    catch (const std::exception &exception)
    {
        std::cerr << "ORB-SLAM3 node failed: " << exception.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    if (rclcpp::ok())
        rclcpp::shutdown();

    return 0;
}
