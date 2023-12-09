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
#include <cmath>
#include <rclcpp/logging.hpp>

namespace helios_control {

Shooter::Shooter(const shooter_controller::Params& params) {
    params_ = params;
    last_state_ = SHOOTER_LOCKED;
    is_blocked_ = false;
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
                                    sensor_interfaces::msg::PowerHeatData power_heat_data,
                                    rclcpp::Time now) {
    // Update time source
    double time_diff = now.seconds() - last_cmd_stamp_.seconds();
    last_cmd_stamp_ = now;
    // Check if we can start dial
    if (time_diff > params_.shooter_cmd_expire_time ) {//|| judge_heat(power_heat_data, time_diff)) {
        if (last_state_ == DIAL_RUNNING) {
            if (is_dial_runnning()) {
                stop_dial();
            }
        } else if (last_state_ == DIAL_LOCKED) {
            last_state_ = SHOOTER_RUNNING;
        } else if (last_state_ == SHOOTER_LOCKED) {
            start_shooter(shooter_cmd);
        } else if (last_state_ == UNDEFINED) {
            last_state_ = SHOOTER_LOCKED;
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
            if (check_dial_blocked()) {
                RCLCPP_INFO(logger_, "dial is blocked!");
            } else {
                if (shooter_cmd.fire_flag == HOLD) {
                    stop_dial();
                } else if (shooter_cmd.fire_flag != FIRE) {
                    last_state_ = UNDEFINED;
                } else {
                    start_dial(shooter_cmd);
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
    // RCLCPP_INFO(logger_, "%d         %f", dial_down_->real_current_, dial_down_->value_);
    // RCLCPP_WARN(logger_, "last_state_: %d", last_state_);
}

void Shooter::update_params(const shooter_controller::Params& params) {
    params_ = params;
}


void Shooter::start_shooter(const helios_control_interfaces::msg::ShooterCmd& shooter_cmd) {
    if (shooter_cmd.shooter_speed == LOW) {
        left_up_shooter_->value_ = -params_.shooter.low_velocity;
        left_down_shooter_->value_ = params_.shooter.low_velocity;
        right_up_shooter_->value_ = params_.shooter.low_velocity;
        right_down_shooter_->value_ = -params_.shooter.low_velocity;
    } else if (shooter_cmd.shooter_speed == HIGH) {
        left_up_shooter_->value_ = -params_.shooter.high_velocity;
        left_down_shooter_->value_ = params_.shooter.high_velocity;
        right_up_shooter_->value_ = params_.shooter.high_velocity;
        right_down_shooter_->value_ = -params_.shooter.high_velocity;
    }
}

bool Shooter::is_shooter_runnning() {
    if (std::abs(left_up_shooter_->real_current_) < 10 ||
        std::abs(left_down_shooter_->real_current_) < 10 ||
        std::abs(right_up_shooter_->real_current_) < 10 ||
        std::abs(right_down_shooter_->real_current_) < 10) {
        return false;
    } else {
        return true;
    }
}

void Shooter::stop_shooter() {
    left_up_shooter_->value_ = 0;
    left_down_shooter_->value_ = 0;
    right_up_shooter_->value_ = 0;
    right_down_shooter_->value_ = 0;
}

void Shooter::start_dial(const helios_control_interfaces::msg::ShooterCmd& shooter_cmd) {
    double dial_vel = params_.dial.dial_velocity_level[shooter_cmd.dial_vel];
    dial_down_->value_ = dial_vel;
    dial_up_->value_ = dial_vel;
    dial_up_->motor_mode_ = 0x01;
    dial_down_->motor_mode_ = 0x01;
}

bool Shooter::is_dial_runnning() {
    bool is_dial_up_started, is_dial_down_started;
    // is_dial_up_started = std::abs(dial_up_->real_current_) > 10;
    is_dial_up_started = true;
    is_dial_down_started = std::abs(dial_down_->real_current_) > 10;
    if (is_dial_up_started && is_dial_down_started) {
        return true;
    } else {
        return false;
    }
}

void Shooter::stop_dial() {
    dial_down_->value_ = 0;
    dial_up_->value_ = 0;
    dial_up_->motor_mode_ = 0x01;
    dial_down_->motor_mode_ = 0x01;
}

bool Shooter::judge_heat(sensor_interfaces::msg::PowerHeatData power_heat_data,
                         double time_diff) {
    if (is_dial_runnning()) {
        if (!dial_up_init_flag) {
            dial_up_init_heat_ = power_heat_data.shooter_id1_17mm_residual_cooling_heat;
            dial_up_init_flag = true;
        }
        if (!dial_down_init_flag) {
            dial_down_init_heat_ = power_heat_data.shooter_id2_17mm_residual_cooling_heat;
            dial_down_init_flag = true;
        }
        dial_up_now_heat_ += dial_up_->real_current_ * time_diff / 8192 / 3 * 8;
        dial_down_now_heat_ += dial_down_->real_current_ * time_diff / 8192 / 3 * 8;
        if (dial_up_now_heat_ < params_.heat_limit && dial_down_now_heat_ < params_.heat_limit) {
            return true;
        } else {
            return false;
        }

    } else {
        dial_up_init_flag = false;
        dial_down_init_flag = false;
        return true;
    }
}

bool Shooter::check_dial_blocked() {
    if (is_blocked_) {
        is_blocked_ = true;
        solve_block_mode(DOWN_DIAL_BLOCKED);
    } else {
        if ((std::abs(dial_down_->real_current_) < 100 && dial_down_->value_ != 0) || dial_down_->given_current_ > params_.dial.dial_current_limit) {
            down_dial_block_cnt_++;
            if (down_dial_block_cnt_ >= params_.dial.dial_block_cnt_limit) {
                down_dial_block_cnt_ = 0;
                is_blocked_ = true;
            }
        }
        if ((std::abs(dial_up_->real_current_) < 10 && dial_up_->value_ != 0) || dial_down_->given_current_ > params_.dial.dial_current_limit) {
            up_dial_block_cnt_++;
            if (up_dial_block_cnt_ >= params_.dial.dial_block_cnt_limit) {
                up_dial_block_cnt_ = 0;
                // is_blocked_ = true;
            }
        }
    }
    return is_blocked_;
}

void Shooter::solve_block_mode(int mode) {
    if (mode == UP_DIAL_BLOCKED) {
        solve_up_block_cnt++;
        if (solve_up_block_cnt > 400) {
            is_blocked_ = false;
            solve_up_block_cnt = 0;
        }
        dial_up_->value_ = -100;
        dial_up_->motor_mode_ = 0x01;
    } else if (mode == DOWN_DIAL_BLOCKED) {
        solve_down_block_cnt++;
        if (solve_down_block_cnt > 400) {
            is_blocked_ = false;
            solve_down_block_cnt = 0;
        }
        dial_down_->value_ = -100;
        dial_down_->motor_mode_ = 0x01;
    }
}

} // namespace helios_control