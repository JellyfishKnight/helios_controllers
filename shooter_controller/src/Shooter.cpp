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
#include "Shooter.hpp"

namespace helios_control {

Shooter::Shooter(const shooter_controller::Params& params) {
    params_ = params;
    last_state_ = SHOOTER_LOCKED;
}

Shooter::~Shooter() {}

void Shooter::update_moto_state(std::map<std::string, math_utilities::MotorPacket>& cmd_map, 
                        std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
    math_utilities::MotorPacket::get_moto_measure(state_interfaces, cmd_map);
    left_up_shooter_ = &cmd_map.find("shooter_left_up")->second;
    left_down_shooter_ = &cmd_map.find("shooter_left_down")->second;
    right_up_shooter_ = &cmd_map.find("shooter_right_up")->second;
    right_down_shooter_ = &cmd_map.find("shooter_right_down")->second;
    dial_down_ = &cmd_map.find("dial_down")->second;
    dial_up_ = &cmd_map.find("dial_up")->second;
}


void Shooter::update_shooter_cmd(helios_control_interfaces::msg::ShooterCmd shooter_cmd, rclcpp::Time now) {
    rclcpp::Time time = shooter_cmd.header.stamp;
    double time_diff = now.seconds() - time.seconds();
    if (time_diff > params_.shooter_cmd_expire_time) {
        if (last_state_ == DIAL_RUNNING) {
            stop_dial();
            return ;
        }
    }
    if (last_state_ == SHOOTER_LOCKED) {
        if ()
        start_shooter(shooter_cmd);
    }
}

void Shooter::update_params(const shooter_controller::Params& params) {
    params_ = params;
}

void Shooter::start_shooter(const helios_control_interfaces::msg::ShooterCmd& shooter_cmd) {

}

void Shooter::stop_shooter() {

}

void Shooter::start_dial(const helios_control_interfaces::msg::ShooterCmd& shooter_cmd) {

}

void Shooter::stop_dial() {

}

void Shooter::handle_undefined() {

}



} // namespace helios_control