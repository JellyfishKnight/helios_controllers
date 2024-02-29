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
#include "shooter/AbreastDoubleShooter.hpp"
#include <cmath>
#include <rclcpp/logging.hpp>

namespace helios_control {

AbreastDoubleShooter::AbreastDoubleShooter(const shooter_controller::Params& params) {
    params_ = params;
    // construct shooters
    // pls name with "shooter_" + "$name" to avoid conflict with the name in file xacro
    shooters_.emplace("left", SingleShooter{params_, "shooter_left"});
    shooters_.emplace("right", SingleShooter{params_, "shooter_right"});
}

void AbreastDoubleShooter::update_motors(const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces,
                                                std::map<std::string, math_utilities::MotorPacket>& cmd_map) {
    // Update shooter state
    for (auto& shooter : shooters_) {
        shooter.second.update_motors(state_interfaces, cmd_map);
    }
}


void AbreastDoubleShooter::update_shooter_cmd(helios_control_interfaces::msg::ShooterCmd shooter_cmd, 
                                    sensor_interfaces::msg::PowerHeatData power_heat_data) {
    // Round robin mode
    // static int cnt = 0;
    // if (is_left_shooter_) {
        // if (cnt < 500) {
        //     cnt++;
        // } else {
        //     cnt = 0;
        //     is_left_shooter_ = false;
        // }
        shooters_.find("left")->second.update_shooter_cmd(shooter_cmd, power_heat_data);
        // shooter_cmd.dial_vel = 0;
        shooters_.find("right")->second.update_shooter_cmd(shooter_cmd, power_heat_data);
        // is_left_shooter_ = false;
    // } else {
        // if (cnt < 500) {
        //     cnt++;
        // } else {
        //     cnt = 0;
        //     is_left_shooter_ = true;
        // }
        // shooters_.find("right")->second.update_shooter_cmd(shooter_cmd, power_heat_data);
        // shooter_cmd.dial_vel = 0;
        // shooters_.find("left")->second.update_shooter_cmd(shooter_cmd, power_heat_data);
        // is_left_shooter_ = true;
    // }
}

void AbreastDoubleShooter::update_params(const shooter_controller::Params& params) {
    params_ = params;
    for (auto& shooter : shooters_) {
        shooter.second.update_params(params);
    }
}


} // namespace helios_control