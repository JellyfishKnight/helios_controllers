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
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <angles/angles.h>
#include <hardware_interface/loaned_state_interface.hpp>

#include "math_utilities/MotorPacket.hpp"
#include "shooter/BaseShooter.hpp"
#include "shooter/SingleShooter.hpp"

#include "helios_control_interfaces/msg/shooter_cmd.hpp"
#include "sensor_interfaces/msg/power_heat_data.hpp"

#include <map>
#include <string.h>
#include <vector>


namespace helios_control {


class AbreastDoubleShooter : public BaseShooter {
public:
    AbreastDoubleShooter(const shooter_controller::Params& params);

    ~AbreastDoubleShooter() = default;

    void update_shooter_cmd(helios_control_interfaces::msg::ShooterCmd shooter_cmd, 
                                    sensor_interfaces::msg::PowerHeatData power_heat_data) override;

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

    bool judge_heat(sensor_interfaces::msg::PowerHeatData power_heat_data, double time_diff);

    bool check_dial_blocked();

    void solve_block_mode(int mode);


    shooter_controller::Params params_;
    rclcpp::Time last_cmd_stamp_;

    // motors
    math_utilities::MotorPacket* left_up_shooter_;
    math_utilities::MotorPacket* left_down_shooter_;
    math_utilities::MotorPacket* right_up_shooter_;
    math_utilities::MotorPacket* right_down_shooter_;
    math_utilities::MotorPacket* dial_up_;
    math_utilities::MotorPacket* dial_down_;

    bool is_blocked_;
    int up_dial_block_cnt_ = 0;
    int down_dial_block_cnt_ = 0;
    int solve_up_block_cnt = 0;
    int solve_down_block_cnt = 0;

    // heat data
    double dial_up_init_heat_;
    double dial_down_init_heat_;
    double dial_up_now_heat_;
    double dial_down_now_heat_;
    bool dial_up_init_flag;
    bool dial_down_init_flag;
    double res_heat_;

    ShooterState last_state_;

    rclcpp::Logger logger_ = rclcpp::get_logger("Shooter");
};


} // namespace helios_control