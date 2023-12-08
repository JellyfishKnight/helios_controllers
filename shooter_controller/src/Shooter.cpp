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


void Shooter::update_shooter_cmd(helios_control_interfaces::msg::ShooterCmd shooter_cmd, 
                                    rclcpp::Time now) {
    // Update time source
    rclcpp::Time time = shooter_cmd.header.stamp;
    double time_diff = now.seconds() - time.seconds();
    // Update rest heat
    caculate_heat();
    // Check if we can start dial
    if (time_diff > params_.shooter_cmd_expire_time || res_heat_ < params_.heat_limit) {
        if (last_state_ == DIAL_RUNNING) {
            if (is_dial_runnning()) {
                stop_dial();
            }
        } else if (last_state_ == DIAL_LOCKED) {
            last_state_ = SHOOTER_RUNNING;
        } else if (last_state_ == SHOOTER_LOCKED) {
            start_shooter(shooter_cmd);
        } else if (last_state_ == UNDEFINED) {
            handle_undefined();
        }
        return ;
    }
    // Update state machine
    // We must strictly limit shooters and dials
    // So we should judge their state every time
    if (shooter_cmd.shooter_speed == STOP) {
        last_state_ = UNDEFINED;
    }
    if (last_state_ == SHOOTER_LOCKED) {
        // Dial is running, turn to undefined
        if (is_dial_runnning()) {
            last_state_ = UNDEFINED;
        }else {
            // We should automatically enter shooter running if there is not any limit
            if (!is_shooter_runnning()) {
                start_shooter(shooter_cmd);
            } else {
                last_state_ = SHOOTER_RUNNING;
            }
        }
    // This is a state to make sure dial is not running
    } else if (last_state_ == SHOOTER_RUNNING) {
        if (is_dial_runnning()) {
            last_state_ = UNDEFINED;
        } else {
            // Shooter stoped, restart shooter
            if (!is_shooter_runnning()) {
                last_state_ = SHOOTER_LOCKED;
            } else {
                last_state_ = DIAL_LOCKED;
            }
        }
    } else if (last_state_ == DIAL_LOCKED) {
        if (!is_shooter_runnning()) {
            // restart shooter
            last_state_ = SHOOTER_LOCKED;
        } else {
            if (is_dial_runnning()) {
                last_state_ = DIAL_RUNNING;
            } else {
                if (shooter_cmd.fire_flag == FIRE) {
                    start_dial(shooter_cmd);
                } else if (shooter_cmd.fire_flag != HOLD) {
                    last_state_ = UNDEFINED;
                }
            }
        }
    } else if (last_state_ == DIAL_RUNNING) {
        if (!is_shooter_runnning()) {
            // Stop both dial and shooter
            last_state_ = UNDEFINED;
        } else {
            if (!is_dial_runnning()) {
                last_state_ = DIAL_LOCKED;
            } else {
                if (shooter_cmd.fire_flag == HOLD) {
                    stop_dial();
                } else if (shooter_cmd.fire_flag != FIRE) {
                    last_state_ = UNDEFINED;
                }

            }
        }
    } else if (last_state_ == UNDEFINED) {
        if (!is_shooter_runnning() && !is_dial_runnning()) {
            last_state_ = SHOOTER_LOCKED;
        } else {
            // Stop dial first
            if (is_dial_runnning()) {
                stop_dial();
            } else {
                if (is_shooter_runnning()) {
                    stop_shooter();
                }
            }
        }
        RCLCPP_ERROR(logger_, "In Undefined state!");
    } else {
        last_state_ = UNDEFINED;
    }
}

void Shooter::update_params(const shooter_controller::Params& params) {
    params_ = params;
}


void Shooter::start_shooter(const helios_control_interfaces::msg::ShooterCmd& shooter_cmd) {

}

bool Shooter::is_shooter_runnning() {

}

void Shooter::stop_shooter() {

}

void Shooter::start_dial(const helios_control_interfaces::msg::ShooterCmd& shooter_cmd) {

}

bool Shooter::is_dial_runnning() {

}

void Shooter::stop_dial() {

}


} // namespace helios_control