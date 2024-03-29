// created by liuhan on 2023/12/22
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

#include "helios_control_interfaces/msg/gimbal_cmd.hpp"
#include "sensor_interfaces/msg/imu_euler.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

#include "math_utilities/MotorPacket.hpp"

// auto generated by ros2 generate_parameter_library
// https://github.com/PickNikRobotics/generate_parameter_library
#include "gimbal_controller_parameters.hpp"

#include <string>
#include <map>
#include <vector>

namespace helios_control {

typedef enum {
    AUTOAIM = 0,
    CRUISE = 1,
    KEEP = 2,
    UNDEFINED = 3,
} GimbalState;

class BaseGimbal {
public:
    BaseGimbal() = default;

    virtual ~BaseGimbal() = default;

    virtual void set_gimbal_cmd(const helios_control_interfaces::msg::GimbalCmd& gimbal_cmd,
                                const sensor_interfaces::msg::ImuEuler& imu_euler,
                                double chassis_rotation_vel) = 0;

    virtual double caculate_diff_angle_from_imu_to_chassis(double compensation_yaw_diff) = 0;

    virtual void update_params(const gimbal_controller::Params& params) = 0;

    virtual void update_motors(const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces,
                               std::map<std::string, math_utilities::MotorPacket>& cmd_map) = 0;

    virtual GimbalState get_last_state() const {
        return last_state_;
    }
protected:
    GimbalState last_state_;
};

} // namespace helios_control
