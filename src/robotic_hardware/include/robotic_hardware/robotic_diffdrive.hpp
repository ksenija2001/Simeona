#ifndef ROBOTIC_DIFFDRIVE_H
#define ROBOTIC_DIFFDRIVE_H

#include <cstring>
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "config.hpp"
#include "wheel.hpp"
#include "nucleo_comms.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

class RoboticDiffDrive : public hardware_interface::SystemInterface
{

public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    Config cfg_;
    NucleoComms nucleo_;

    Wheel left_;
    Wheel right_;

    std::chrono::time_point<std::chrono::system_clock> time_;
};

#endif // ROBOTIC_DIFFDRIVE_H