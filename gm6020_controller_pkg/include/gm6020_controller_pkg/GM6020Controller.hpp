#pragma once

#include "visibility_control.h"
#include "controller_interface/controller_interface.hpp"

#include "rclcpp/rclcpp.hpp"
#include "math_utilities/PID.hpp"


#include <vector>
#include <string>

namespace helios_control {

constexpr auto DEFAULT_COMMAND_TOPIC = "gm6020/cmd";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "gm6020/cmd_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "gm6020/cmd_out";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";


class GM6020Controller : public controller_interface::ControllerInterface {
public:
    GM6020_CONTROLLER_PUBLIC
    GM6020Controller();

    GM6020_CONTROLLER_PUBLIC
    controller_interface::return_type init(const std::string &controller_name);

    GM6020_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration();

    GM6020_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration();

    GM6020_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state);

    GM6020_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);

    GM6020_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

    GM6020_CONTROLLER_PUBLIC
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period);
protected:
    std::vector<std::string> joint_names_;
    std::vector<std::string> interface_names_;
private:
    std::vector<int> motor_number_;
    
    double control_freq_;



};

} // helios_control
