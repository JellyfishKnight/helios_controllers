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
#pragma once

#include <cstdint>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

#include "shooter/BaseShooter.hpp"

namespace helios_control {

class SingleShooter : public BaseShooter {
public:
    SingleShooter(const shooter_controller::Params& params, const std::string& shooter_name = "single");

    ~SingleShooter() = default;

    void update_shooter_cmd(helios_control_interfaces::msg::ShooterCmd shooter_cmd, 
                                    sensor_interfaces::msg::RobotAim heat_data) override;

    void update_motors(const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces,
                                std::map<std::string, math_utilities::MotorPacket>& cmd_map) override;

    void update_params(const shooter_controller::Params& params) override;

private:
    void start_shooter(const helios_control_interfaces::msg::ShooterCmd& shooter_cmd);

    bool is_shooter_runnning();

    void stop_shooter();

    void start_dial(const helios_control_interfaces::msg::ShooterCmd& shooter_cmd);

    bool is_dial_runnning();

    void stop_dial();

    bool judge_heat(sensor_interfaces::msg::RobotAim power_heat_data);

    bool check_dial_blocked();

    void solve_block_mode();

    shooter_controller::Params params_;

    const std::string shooter_name_;

    rclcpp::Clock clock_;
    rclcpp::Time last_cmd_time_;
    
    ShooterSpeed last_speed_;
    int last_shooter_speed_ = 0;
    int last_dial_speed_ = 0;

    math_utilities::MotorPacket* shooter_up_moto_ptr_;
    math_utilities::MotorPacket* shooter_down_moto_ptr_;
    math_utilities::MotorPacket* dial_moto_ptr_;

    // solve block utilities
    bool is_blocked_ = false;
    int dial_block_cnt_ = 0;
    int64_t block_total_angle_;
    int solve_block_cnt_ = 0;

    // heat data
    double dial_init_heat_;
    double dial_now_heat_;
    bool dial_init_flag_;
    double res_heat_;

    rclcpp::Logger logger_ = rclcpp::get_logger("SingleShooter");
};


} // namespace helios_control