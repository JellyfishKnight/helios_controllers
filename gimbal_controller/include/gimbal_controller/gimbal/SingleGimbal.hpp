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
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <angles/angles.h>
#include <rclcpp/time.hpp>

#include "helios_control_interfaces/msg/gimbal_cmd.hpp"
#include "sensor_interfaces/msg/imu_euler.hpp"

#include "math_utilities/MotorPacket.hpp"

#include "BaseGimbal.hpp"


namespace helios_control {


class SingleGimbal : public BaseGimbal{
public: 
    SingleGimbal(const gimbal_controller::Params& params);

    ~SingleGimbal();

    void set_gimbal_cmd(const helios_control_interfaces::msg::GimbalCmd& gimbal_cmd,
                        const sensor_interfaces::msg::ImuEuler& imu_euler,
                        double chassis_rotation_vel) override;

    double caculate_diff_angle_from_imu_to_chassis() override;

    void update_params(const gimbal_controller::Params& params) override;

    void update_motors(const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces,
                            std::map<std::string, math_utilities::MotorPacket>& cmd_map) override;

private:
    void do_cruise(double yaw_vel, double pitch_vel, double chassis_rotation_vel);

    void do_autoaim(double yaw_angle, double pitch_angle);

    void do_undefined(const helios_control_interfaces::msg::GimbalCmd& gimbal_cmd, double chassis_rotation_vel);

    GimbalState last_state_;
    int pitch_vel_flag_ = 1;
    int accel_cnt_ = 0;
    int last_pitch_angle_ = 0;

    math_utilities::MotorPacket* yaw_moto_ptr_;
    math_utilities::MotorPacket* pitch_moto_ptr_;

    helios_control_interfaces::msg::GimbalCmd last_gimbal_cmd_;

    gimbal_controller::Params params_;

    int lost_cnt_ = 0;
    
    int imu_round_cnt_;
    double imu_total_yaw_;
    double last_imu_yaw_;
    double imu_pitch_;

    rclcpp::Logger logger_ = rclcpp::get_logger("SingleGimbal");
};

} // namespace helios_control