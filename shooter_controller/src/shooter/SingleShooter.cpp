// created by liuhan on 2023/12/22
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
#include "shooter/SingleShooter.hpp"


namespace helios_control {

SingleShooter::SingleShooter(const shooter_controller::Params& params, const std::string& shooter_name) :
    shooter_name_(std::move(shooter_name)) {
    params_ = params;
}

void SingleShooter::update_shooter_cmd(helios_control_interfaces::msg::ShooterCmd shooter_cmd, 
                                sensor_interfaces::msg::PowerHeatData power_heat_data) {
    
}

void SingleShooter::update_params(const shooter_controller::Params& params) {
    params_ = params;
}

void SingleShooter::start_shooter(const helios_control_interfaces::msg::ShooterCmd& shooter_cmd) {
    
}

bool SingleShooter::is_shooter_runnning() {

}

void SingleShooter::stop_shooter() {

}

void SingleShooter::start_dial(const helios_control_interfaces::msg::ShooterCmd& shooter_cmd) {

}

bool SingleShooter::is_dial_runnning() {

}

void SingleShooter::stop_dial() {

}

bool SingleShooter::judge_heat(sensor_interfaces::msg::PowerHeatData power_heat_data, double time_diff) {

}

bool SingleShooter::check_dial_blocked() {

}

void SingleShooter::solve_block_mode(int mode) {
    
}



} // namespace helios_control