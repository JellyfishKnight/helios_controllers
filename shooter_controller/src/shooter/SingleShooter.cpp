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
#include "shooter/BaseShooter.hpp"
#include <rclcpp/logging.hpp>


namespace helios_control {

SingleShooter::SingleShooter(const shooter_controller::Params& params, const std::string& shooter_name) :
    shooter_name_(std::move(shooter_name)) {
    params_ = params;
    last_cmd_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void SingleShooter::update_shooter_cmd(helios_control_interfaces::msg::ShooterCmd shooter_cmd, 
                                sensor_interfaces::msg::PowerHeatData power_heat_data) {
    // Check if we can start dial
    ///TODO: lack of referee system, not judging heat now
    // if (!judge_heat(power_heat_data, time_gap)) {
    //     if (last_state_ == DIAL_RUNNING) {
    //         if (is_dial_runnning()) {
    //             stop_dial();
    //         }
    //     } else if (last_state_ == DIAL_LOCKED) {
    //         last_state_ = SHOOTER_RUNNING;
    //     } else if (last_state_ == SHOOTER_LOCKED) {
    //         start_shooter(shooter_cmd);
    //     } else if (last_state_ == UNDEFINED) {
    //         last_state_ = SHOOTER_LOCKED;
    //     }
    //     return ;
    // }
    // Update state machine
    // We must strictly limit shooters and dials
    // So we should judge their state every time
    
    if (shooter_cmd.shooter_speed == STOP) {
        stop_shooter();
        stop_dial();
    } else {
        start_shooter(shooter_cmd);
        if (shooter_cmd.fire_flag == FIRE) {
            if (is_shooter_runnning()) {
                if (check_dial_blocked()) {
                    RCLCPP_WARN(logger_, "%s's dial is blocked", shooter_name_.c_str());
                } else {
                    RCLCPP_INFO(logger_, "%s's dial is fine", shooter_name_.c_str());
                    start_dial(shooter_cmd);
                }
            } else {
                stop_dial();
            }
        } else if (shooter_cmd.fire_flag == HOLD) {
            stop_dial();
        }
    }
}

void SingleShooter::update_params(const shooter_controller::Params& params) {
    params_ = params;
}

void SingleShooter::update_motors(const std::vector<hardware_interface::LoanedStateInterface> &state_interfaces, 
                                    std::map<std::string, math_utilities::MotorPacket> &cmd_map) {
    math_utilities::MotorPacket::get_moto_measure(state_interfaces, cmd_map);
    shooter_up_moto_ptr_ = &cmd_map.find(shooter_name_ + "_up")->second;
    shooter_down_moto_ptr_ = &cmd_map.find(shooter_name_ + "_down")->second;
    dial_moto_ptr_ = &cmd_map.find(shooter_name_ + "_dial")->second;
}

void SingleShooter::start_shooter(const helios_control_interfaces::msg::ShooterCmd& shooter_cmd) {
    if (shooter_name_ == "shooter_left" || shooter_name_ == "single") {
        shooter_up_moto_ptr_->value_ = shooter_cmd.shooter_speed == LOW ? -params_.shooter.low_velocity : -params_.shooter.high_velocity;
        shooter_down_moto_ptr_->value_ = shooter_cmd.shooter_speed == LOW ? params_.shooter.low_velocity : params_.shooter.high_velocity;
    } else if (shooter_name_ == "shooter_right") {
        shooter_up_moto_ptr_->value_ = shooter_cmd.shooter_speed == LOW ? params_.shooter.low_velocity : params_.shooter.high_velocity;
        shooter_down_moto_ptr_->value_ = shooter_cmd.shooter_speed == LOW ? -params_.shooter.low_velocity: -params_.shooter.high_velocity;
    }
}

bool SingleShooter::is_shooter_runnning() {
    if (std::abs(shooter_up_moto_ptr_->real_current_) < 10 ||
        std::abs(shooter_down_moto_ptr_->real_current_) < 10) {
        return false;
    } else {
        return true;
    }

}

void SingleShooter::stop_shooter() {
    shooter_down_moto_ptr_->value_ = 0;
    shooter_up_moto_ptr_->value_ = 0;
}

void SingleShooter::start_dial(const helios_control_interfaces::msg::ShooterCmd& shooter_cmd) {
    dial_moto_ptr_->value_ = shooter_name_ == "shooter_left" ? -params_.dial.dial_velocity_level[shooter_cmd.dial_vel] : 
                                        params_.dial.dial_velocity_level[shooter_cmd.dial_vel];
}

bool SingleShooter::is_dial_runnning() {
    bool is_dial_started = false;
    is_dial_started = std::abs(dial_moto_ptr_->real_current_) > 10;
    return is_dial_started;
}

void SingleShooter::stop_dial() {
    dial_moto_ptr_->value_ = 0;
    dial_moto_ptr_->motor_mode_ = 0x01;
}

bool SingleShooter::judge_heat(sensor_interfaces::msg::PowerHeatData power_heat_data, double time_diff) {
    if (is_dial_runnning()) {
        if (!dial_init_flag_) {
            dial_init_heat_ = power_heat_data.shooter_id1_17mm_residual_cooling_heat;
            dial_init_flag_ = true;
        }
        dial_now_heat_ += dial_moto_ptr_->real_current_ * time_diff / 8192 / 3 * 8;
        if (dial_now_heat_ < params_.heat_limit && dial_now_heat_ < params_.heat_limit) {
            return true;
        } else {
            return false;
        }
    } else {
        dial_init_flag_ = false;
        return true;
    }
}

bool SingleShooter::check_dial_blocked() {
    if (is_blocked_) {
        is_blocked_ = true;
        solve_block_mode();
    } else {
        if ((std::abs(dial_moto_ptr_->real_current_) < 10 && dial_moto_ptr_->value_ != 0) || 
            dial_moto_ptr_->given_current_ > params_.dial.dial_current_limit) {
            dial_block_cnt_++;
            if (dial_block_cnt_ >= params_.dial.dial_block_cnt_limit) {
                dial_block_cnt_ = 0;
                is_blocked_ = true;
            }
        }
    }
    return is_blocked_;
}

void SingleShooter::solve_block_mode() {
    solve_block_cnt++;
    if (solve_block_cnt > 50) {
        is_blocked_ = false;
        solve_block_cnt = 0;
    }
    if (shooter_name_ == "shooter_left") {
        dial_moto_ptr_->value_ = 400;
    } else {
        dial_moto_ptr_->value_ = -400;
    }
    dial_moto_ptr_->motor_mode_ = 0x01;
}



} // namespace helios_control