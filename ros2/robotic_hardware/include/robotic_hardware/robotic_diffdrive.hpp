#ifndef ROBOTIC_DIFFDRIVE_H
#define ROBOTIC_DIFFDRIVE_H

#include <cstring>
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

#include "config.hpp"
#include "wheel.hpp"
#include "nucleo_comms.hpp"

using hardware_interface::return_type;

class RoboticDiffDrive : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{

public:
    RoboticDiffDrive();
    return_type configure(const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    return_type start() override;
    return_type stop() override;
    return_type read() override;
    return_type write() override;

private:
    Config cfg_;
    NucleoComms nucleo_;

    Wheel left_;
    Wheel right_;

    rclcpp::Logger logger_;

    std::chrono::time_point<std::chrono::system_clock> time_;
};

#endif // ROBOTIC_DIFFDRIVE_H