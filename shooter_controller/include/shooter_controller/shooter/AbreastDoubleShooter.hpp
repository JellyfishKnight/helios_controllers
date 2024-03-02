// created by liuhan on 2023/12/9
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 *
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <angles/angles.h>
#include <hardware_interface/loaned_state_interface.hpp>

#include "math_utilities/MotorPacket.hpp"
#include "shooter/BaseShooter.hpp"
#include "shooter/SingleShooter.hpp"

#include "helios_control_interfaces/msg/shooter_cmd.hpp"
#include "sensor_interfaces/msg/robot_aim.hpp"

#include <map>
#include <string.h>
#include <vector>


namespace helios_control {


class AbreastDoubleShooter : public BaseShooter {
public:
    AbreastDoubleShooter(const shooter_controller::Params& params);

    ~AbreastDoubleShooter() = default;

    void update_shooter_cmd(helios_control_interfaces::msg::ShooterCmd shooter_cmd, 
                                    sensor_interfaces::msg::RobotAim power_heat_data) override;

    void update_motors(const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces,
                            std::map<std::string, math_utilities::MotorPacket>& cmd_map) override;

    void update_params(const shooter_controller::Params& params) override;

private:

    shooter_controller::Params params_;

    // shooters_
    std::map<std::string, SingleShooter> shooters_;

    // shooter choose flag
    bool is_left_shooter_ = true;

    rclcpp::Logger logger_ = rclcpp::get_logger("Shooter");
};


} // namespace helios_control