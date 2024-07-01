#include "robotic_hardware/robotic_diffdrive.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "robotic_hardware/definitions.hpp"

CallbackReturn RoboticDiffDrive::on_init(
    const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Configuring...");

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

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Finished Configuration");

    return CallbackReturn::SUCCESS;
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

CallbackReturn RoboticDiffDrive::on_configure(
    const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Controller...");

    const float msg[] = {WHEEL_DIAMETER, WHEEL_TRACK};
    bool response = nucleo_.sendMsg(msg, CONFIG, CONFIG_LEN);

    if (response)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Config message received");

    const float msg_reset[] = {0.0, 0.0, 1.57};
    response = nucleo_.sendMsg(msg_reset, INIT, INIT_LEN);

    if (response)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Init message received");

    return CallbackReturn::SUCCESS;
}

CallbackReturn RoboticDiffDrive::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stopping Controller...");

    return CallbackReturn::SUCCESS;
}

return_type RoboticDiffDrive::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
{

    if (!nucleo_.connected())
        return return_type::ERROR;

    const float msg[] = {0x01};
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

return_type RoboticDiffDrive::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

    if (!nucleo_.connected())
        return return_type::ERROR;

    const float msg[] = {left_.cmd * 1000, right_.cmd * 1000};
    nucleo_.sendMsg(msg, SPEED, SPEED_LEN);

    return return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    RoboticDiffDrive,
    hardware_interface::SystemInterface)