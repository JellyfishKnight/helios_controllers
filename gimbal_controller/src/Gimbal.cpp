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
#include "Gimbal.hpp"

namespace helios_control {

Gimbal::Gimbal(const gimbal_controller::Params& params) : 
    last_state_(CRUISE), params_(params){
    last_autoaim_msg_time_ = 0.0;
}

Gimbal::~Gimbal() {

}

void Gimbal::update_params(const gimbal_controller::Params& params) {
    params_ = params;
}

void Gimbal::set_gimbal_cmd(const helios_control_interfaces::msg::GimbalCmd& gimbal_cmd) {
    // Update time source
    rclcpp::Time time = gimbal_cmd.header.stamp;
    // Update state machine
    if (last_state_ == AUTOAIM) {
        if (gimbal_cmd.gimbal_mode == DEBUG) {
            last_state_ = DEBUG;
        } else if (gimbal_cmd.gimbal_mode == CRUISE) {
            if (last_autoaim_msg_time_ + params_.autoaim_timeout < time.seconds()) {
                last_state_ = CRUISE;
            }
        } else {
            last_state_ = AUTOAIM;
            last_autoaim_msg_time_ = time.seconds();
        }
    } else if (last_autoaim_msg_time_ == CRUISE) {
        if (gimbal_cmd.gimbal_mode == DEBUG) {
            last_state_ = DEBUG;
        } else if (gimbal_cmd.gimbal_mode == CRUISE) {
            last_state_ = CRUISE;
        } else {
            last_state_ = AUTOAIM;
            last_autoaim_msg_time_ = time.seconds();
        }
    } else if (gimbal_cmd.gimbal_mode == DEBUG) {
        if (gimbal_cmd.gimbal_mode == DEBUG) {
            last_state_ = DEBUG;
        } else if (gimbal_cmd.gimbal_mode == CRUISE) {
            last_state_ = CRUISE;
        } else {
            last_state_ = AUTOAIM;
            last_autoaim_msg_time_ = time.seconds();
        }
    } else if (gimbal_cmd.gimbal_mode == UNDEFINED) {
        last_state_ = static_cast<GimbalState>(gimbal_cmd.gimbal_mode);
    } else {
       last_state_ = UNDEFINED; 
    }
    // Update gimbal command
    if (last_state_ == DEBUG) {
        do_debug(gimbal_cmd);
    } else if (last_state_ == CRUISE) {
        do_cruise(gimbal_cmd.cruise_yaw_vel, gimbal_cmd.cruise_pitch_vel);
    } else if (last_state_ == AUTOAIM) {
        do_autoaim(gimbal_cmd.yaw, gimbal_cmd.pitch);
    } else {
        do_undefined(gimbal_cmd);
    }
}

void Gimbal::update_moto(std::map<std::string, math_utilities::MotorPacket>& cmd_map, 
                    const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
    math_utilities::MotorPacket::get_moto_measure(state_interfaces, cmd_map);
    yaw_moto_ptr_ = &cmd_map.find("yaw")->second;
    pitch_moto_ptr_ = &cmd_map.find("pitch")->second;
}

void Gimbal::do_undefined(const helios_control_interfaces::msg::GimbalCmd& gimbal_cmd) {
    // Auto turn to cruise mode to prevent serious damage
    RCLCPP_ERROR(logger_, "Gimbal is in undefined state");
    do_cruise(gimbal_cmd.cruise_yaw_vel, gimbal_cmd.cruise_pitch_vel);
    return;
}

void Gimbal::do_debug(const helios_control_interfaces::msg::GimbalCmd& gimbal_cmd) {
    if (gimbal_cmd.gimbal_mode == DEBUG_POSITION) {
        do_autoaim(gimbal_cmd.yaw, gimbal_cmd.pitch);
    } else if (gimbal_cmd.gimbal_mode == DEBUG_VELOCITY) {
        do_cruise(gimbal_cmd.yaw, gimbal_cmd.pitch);
    } else {
        RCLCPP_ERROR(logger_, "Invalid debug mode");
        do_undefined(gimbal_cmd);
    } 
}

void Gimbal::do_cruise(double yaw_vel, double pitch_vel) {
    
}

void Gimbal::do_autoaim(double yaw_angle, double pitch_angle) {

}


} // namespace helios_control