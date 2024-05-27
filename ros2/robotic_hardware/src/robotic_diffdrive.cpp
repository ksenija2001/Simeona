#include "diffdrive_arduino/diffdrive_arduino.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "include/robotic_hardware/definitions.hpp"

RoboticDiffDrive::RoboticDiffDrive()
    : logger_(rclcpp::get_logger("RoboticDiffDrive"))
{
}

return_type RoboticDiffDrive::configure(const hardware_interface::HardwareInfo &info)
{
    if (configure_default(info) != return_type::OK)
    {
        return return_type::ERROR;
    }

    RCLCPP_INFO(logger_, "Configuring...");

    time_ = std::chrono::system_clock::now();

    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);

    // Set up the wheels
    left_.setup(cfg_.left_wheel_name);
    right_.setup(cfg_.right_wheel_name);

    // Set up the Arduino
    nucleo_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);

    RCLCPP_INFO(logger_, "Finished Configuration");

    status_ = hardware_interface::status::CONFIGURED;
    return return_type::OK;
}

std::vector<hardware_interface::StateInterface> RoboticDiffDrive::export_state_interfaces()
{
    // We need to set up a position and a velocity interface for each wheel
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(left_.name, hardware_interface::HW_IF_VELOCITY, &left_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(left_.name, hardware_interface::HW_IF_POSITION, &left_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_.name, hardware_interface::HW_IF_VELOCITY, &right_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_.name, hardware_interface::HW_IF_POSITION, &right_.pos));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoboticDiffDrive::export_command_interfaces()
{
    // We need to set up a velocity command interface for each wheel
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(left_.name, hardware_interface::HW_IF_VELOCITY, &left_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(right_.name, hardware_interface::HW_IF_VELOCITY, &right_.cmd));

    return command_interfaces;
}

return_type RoboticDiffDrive::start()
{
    RCLCPP_INFO(logger_, "Starting Controller...");

    const msg[] = {WHEEL_DIAMETER, WHEEL_TRACK};
    bool response = nucleo_.sendMsg(msg, CONFIG, CONFIG_LEN);

    if (response)
        RCLCPP_INFO(logger_, "Config message received");

    const msg[] = {0, 0, 1.57};
    bool response = nucleo_.sendMsg(msg, INIT, INIT_LEN);

    if (response)
        RCLCPP_INFO(logger_, "Init message received");

    status_ = hardware_interface::status::STARTED;

    return return_type::OK;
}

return_type RoboticDiffDrive::stop()
{
    RCLCPP_INFO(logger_, "Stopping Controller...");
    status_ = hardware_interface::status::STOPPED;

    return return_type::OK;
}

hardware_interface::return_type RoboticDiffDrive::read()
{

    // TODO fix chrono duration

    // Calculate time delta
    auto new_time = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = new_time - time_;
    double deltaSeconds = diff.count();
    time_ = new_time;

    if (!nucleo_.connected())
        return return_type::ERROR;

    const msg[] = {0x01};
    if (!nucleo_.sendMsg(msg, ODOM, 1))
        return return_type::ERROR;

    float receive[7];
    nucleo_.readMsg(receive, ODOM, ODOM_LEN);

    left_.vel = receive[3] / 1000;
    right_.vel = receive[4] / 1000;
    left_.pos = receive[5] / 1000;
    right_.pos = receive[6] / 1000;

    return return_type::OK;
}

hardware_interface::return_type RoboticDiffDrive::write()
{

    if (!nucleo_.connected())
        return return_type::ERROR;

    const msg[] = {left_.cmd * 1000, right_.cmd * 1000};
    nucleo_.sendMsg(msg, SPEED, SPEED_LEN);

    return return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    RoboticDiffDrive,
    hardware_interface::SystemInterface)