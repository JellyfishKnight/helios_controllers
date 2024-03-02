// created by liuhan on 2023/9/22
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

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_publisher.h"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "helios_control_interfaces/msg/shooter_cmd.hpp"
#include "helios_control_interfaces/msg/motor_state.hpp"
#include "helios_control_interfaces/msg/motor_states.hpp"
#include "sensor_interfaces/msg/robot_aim.hpp"

#include "visibility_control.h"

#include <memory>
#include <sensor_interfaces/msg/detail/robot_aim__struct.hpp>
#include <vector>
#include <queue>
#include <string>
#include <map>

#include "math_utilities/MotorPacket.hpp"
#include "shooter/AbreastDoubleShooter.hpp"
#include "shooter/SingleShooter.hpp"
#include "shooter/BaseShooter.hpp"



namespace helios_control {

constexpr auto DEFAULT_COMMAND_TOPIC = "/shooter_cmd";
constexpr auto DEFAULT_HEAT_TOPIC = "/power_heat_data";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "/shooter_cmd_out";

using Params = shooter_controller::Params;
using ParamsListener = shooter_controller::ParamListener;

class ShooterController : public controller_interface::ControllerInterface {
public:
    SHOOTER_CONTROLLER_PUBLIC
    ShooterController() = default;

    SHOOTER_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    SHOOTER_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    SHOOTER_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    SHOOTER_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    SHOOTER_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    SHOOTER_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    SHOOTER_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    SHOOTER_CONTROLLER_PUBLIC
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
protected:
    int motor_number_;
    int state_interface_number_;
    int command_interface_number_;
    std::vector<std::string> motor_names_;
    std::vector<double> motor_commands_;
    // cmd subscriber
    std::shared_ptr<realtime_tools::RealtimePublisher<helios_control_interfaces::msg::MotorStates>> realtime_shooter_state_pub_;
    rclcpp::Publisher<helios_control_interfaces::msg::MotorStates>::SharedPtr state_pub_;
    
    realtime_tools::RealtimeBox<std::shared_ptr<helios_control_interfaces::msg::ShooterCmd>> received_shooter_cmd_ptr_;
    realtime_tools::RealtimeBox<std::shared_ptr<sensor_interfaces::msg::RobotAim>> received_heat_ptr_;

    rclcpp::Subscription<helios_control_interfaces::msg::ShooterCmd>::SharedPtr cmd_sub_;
    rclcpp::Subscription<sensor_interfaces::msg::RobotAim>::SharedPtr heat_sub_;
    // Parameters from ROS for OmnidirectionalController
    std::shared_ptr<ParamsListener> param_listener_;
    Params params_;

    // publish rate limiter
    double publish_rate_ = 1000.0;
    rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
    rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};
    // Timeout to consider cmd commands old
    std::chrono::milliseconds cmd_timeout_{50};

    std::shared_ptr<helios_control_interfaces::msg::ShooterCmd> last_command_msg;
    std::shared_ptr<sensor_interfaces::msg::RobotAim> last_heat_msg;

    bool should_publish_ = false;
    // motor cmds
    std::map<std::string, math_utilities::MotorPacket> cmd_map_;

    std::shared_ptr<BaseShooter> shooter_;

    double last_cmd_time_;
    double time_diff_ = 0;
        
    /**
     * @brief Convert the current state of the chassis from state_interfaces to a ROS message
     * @param state_msg The message to be filled
     */
    bool export_state_interfaces(helios_control_interfaces::msg::MotorStates& state_msg);

    bool is_halted_ = false;
    bool subscriber_is_active_ = false;
    bool reset();
    void halt();

    rclcpp::Logger logger_ = rclcpp::get_logger("ShooterController");
};



} // namespace helios_control