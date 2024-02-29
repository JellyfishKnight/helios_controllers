// created by liuhan on 2023/12/8
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */
#include "gimbal/SingleGimbal.hpp"
#include <rclcpp/logging.hpp>

namespace helios_control {

SingleGimbal::SingleGimbal(const gimbal_controller::Params& params) : 
    last_state_(CRUISE), params_(params) {
    imu_round_cnt_ = 0;
    accel_cnt_ = 0;
    pitch_vel_flag_ = 1;
}

SingleGimbal::~SingleGimbal() {

}

void SingleGimbal::update_params(const gimbal_controller::Params& params) {
    params_ = params;
}

void SingleGimbal::set_gimbal_cmd(const helios_control_interfaces::msg::GimbalCmd& gimbal_cmd,
                            const sensor_interfaces::msg::ImuEuler& imu_euler,
                            double chassis_rotation_vel) {
    // Update imu info
    double yaw_diff = imu_euler.yaw - last_imu_yaw_;
    if (yaw_diff < -180) {
        imu_round_cnt_++;
    }
    else if (yaw_diff > 180) {
        imu_round_cnt_--;
    }
    imu_total_yaw_ = imu_euler.yaw + imu_round_cnt_ * 360 - imu_euler.init_yaw;
    last_imu_yaw_ = imu_euler.yaw;
    imu_pitch_= imu_euler.pitch;
    rclcpp::Time now = gimbal_cmd.header.stamp;
    // Update state machine
    if (last_state_ == AUTOAIM) {
        if (gimbal_cmd.gimbal_mode == CRUISE) {
            last_state_ = KEEP;
        } else {
            last_state_ = AUTOAIM;
        }
    } else if (last_state_ == CRUISE) {
        if (gimbal_cmd.gimbal_mode == CRUISE) {
            last_state_ = CRUISE;
        } else if (gimbal_cmd.gimbal_mode == AUTOAIM) {
            last_state_ = AUTOAIM;
        }
    } else if (last_state_ == KEEP) {
        if (gimbal_cmd.gimbal_mode == AUTOAIM) {
            last_state_ = AUTOAIM;
            lost_cnt_ = 0;
        } else if (gimbal_cmd.gimbal_mode == CRUISE) {
            lost_cnt_++;
            if (lost_cnt_ > params_.autoaim_expire_time) {
                last_state_ = CRUISE;
                lost_cnt_ = 0;
                RCLCPP_WARN(logger_, "autoaim cmd expired");
            } else {
                last_state_ = KEEP;
            }
        }
    } else if (last_state_ == UNDEFINED) {
        last_state_ = static_cast<GimbalState>(gimbal_cmd.gimbal_mode);
    } else {
       last_state_ = UNDEFINED; 
    }
    // Update gimbal command
    if (last_state_ == CRUISE) {
        do_cruise(gimbal_cmd.yaw_value, gimbal_cmd.pitch_value, chassis_rotation_vel);
    } else if (last_state_ == AUTOAIM) {
        do_autoaim(gimbal_cmd.yaw_value, gimbal_cmd.pitch_value);
        last_gimbal_cmd_ = gimbal_cmd;
    } else if (last_state_ == KEEP) {
        do_autoaim(last_gimbal_cmd_.yaw_value, last_gimbal_cmd_.pitch_value);
    } else {
        do_undefined(gimbal_cmd, chassis_rotation_vel);
    }
}


void SingleGimbal::update_motors(const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces,
                    std::map<std::string, math_utilities::MotorPacket>& cmd_map) {
    math_utilities::MotorPacket::get_moto_measure(state_interfaces, cmd_map);
    yaw_moto_ptr_ = &cmd_map.find("yaw")->second;
    pitch_moto_ptr_ = &cmd_map.find("pitch")->second;
}

void SingleGimbal::do_undefined(const helios_control_interfaces::msg::GimbalCmd& gimbal_cmd, double chassis_rotation_vel) {
    // Auto turn to cruise mode to prevent serious damage
    RCLCPP_ERROR(logger_, "Gimbal is in undefined state");
    do_cruise(gimbal_cmd.yaw_value, gimbal_cmd.pitch_value, chassis_rotation_vel);
    return;
}


void SingleGimbal::do_cruise(double yaw_vel, double pitch_vel, double chassis_rotation_vel) {
    // Set yaw motor velocity
    // Limit yaw's motor velocity
    if (std::abs(yaw_vel) > params_.yaw_vel_limit) {
        yaw_vel = yaw_vel > 0 ? params_.yaw_vel_limit : -params_.yaw_vel_limit;
    }
    // Limit pitch's motor velocity
    if (std::abs(pitch_vel) > params_.pitch_vel_limit) {
        pitch_vel = pitch_vel > 0 ? params_.pitch_vel_limit : -params_.pitch_vel_limit;
    }
    yaw_moto_ptr_->value_ = yaw_vel + chassis_rotation_vel;
    yaw_moto_ptr_->motor_mode_ = 0x01;
    // Set pitch motor velocity
    if (imu_pitch_ < -15) {
        pitch_vel_flag_ = 1;
    } else if (imu_pitch_ > 10) {
        pitch_vel_flag_ = -1;
    }
    pitch_moto_ptr_->value_ = pitch_vel_flag_ * pitch_vel;
    pitch_moto_ptr_->motor_mode_ = 0x01;
}

void SingleGimbal::do_autoaim(double yaw_angle, double pitch_angle) {
    // Set yaw motor angle
    // Convert absolute yaw and pitch into total angle
    double yaw_diff_from_i2c = angles::shortest_angular_distance(
        angles::from_degrees(std::fmod(imu_total_yaw_, 360.0)),
        angles::from_degrees(std::fmod(yaw_angle, 360.0))
    );
    yaw_diff_from_i2c = (-yaw_diff_from_i2c / 2 / M_PI) * 8192.0;
    // Compute total value
    yaw_moto_ptr_->value_ = yaw_moto_ptr_->total_angle_ + yaw_diff_from_i2c;
    // Set Pitch motor angle
    double pitch_diff = angles::shortest_angular_distance(
        angles::from_degrees(imu_pitch_),
        angles::from_degrees(pitch_angle)
    );
    // RCLCPP_INFO(logger_, "%f %f", imu_pitch_, pitch_angle);
    pitch_diff = (pitch_diff / 2 / M_PI) * 8192.0;
    // RCLCPP_INFO(logger_, "%f", pitch_diff);
    pitch_moto_ptr_->value_ = pitch_moto_ptr_->total_angle_ + pitch_diff;
    // Limit pitch's motor angle
    if (pitch_moto_ptr_->value_ > params_.pitch_max_angle) {
        pitch_moto_ptr_->value_ = params_.pitch_max_angle;
    } else if (pitch_moto_ptr_->value_ < params_.pitch_min_angle) {
        pitch_moto_ptr_->value_ = params_.pitch_min_angle;
    }
    // Set motor mode
    pitch_moto_ptr_->motor_mode_ = 0x02;
    yaw_moto_ptr_->motor_mode_ = 0x02;
}


double SingleGimbal::caculate_diff_angle_from_imu_to_chassis(double compensation_yaw_diff) {
    return -(fmod(yaw_moto_ptr_->total_angle_ - 6380, 8192.0) / 8192) * 2 * M_PI
            - angles::from_degrees(fmod((imu_total_yaw_ + compensation_yaw_diff), 360.0));
}

} // namespace helios_control