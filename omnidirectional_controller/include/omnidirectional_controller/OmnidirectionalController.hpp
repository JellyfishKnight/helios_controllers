// created by liuhan on 2023/9/21
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
#include "helios_control_interfaces/msg/motor_state.hpp"
#include "helios_control_interfaces/msg/motor_states.hpp"
#include "std_msgs/msg/float64.hpp"

#include <memory>
#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "visibility_control.h"

#include <vector>
#include <queue>
#include <string>
#include <chrono>
#include <map>

#include "math_utilities/OmnidirectionalSolver.hpp"
#include "math_utilities/MotorPacket.hpp"
// auto generated by ros2 generate_parameter_library
// https://github.com/PickNikRobotics/generate_parameter_library
#include "omnidirectional_controller_parameters.hpp"


namespace helios_control {

constexpr auto DEFAULT_COMMAND_TOPIC = "/chassis_cmd_vel";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "/chassis_cmd_vel_out";

using Params = omnidirectional_controller::Params;
using ParamsListener = omnidirectional_controller::ParamListener;

class OmnidirectionalController : public controller_interface::ControllerInterface {
public:
    OMNIDIRECTIONAL_CONTROLLER_PUBLIC
    OmnidirectionalController() = default;

    OMNIDIRECTIONAL_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    OMNIDIRECTIONAL_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    OMNIDIRECTIONAL_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    OMNIDIRECTIONAL_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    OMNIDIRECTIONAL_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    OMNIDIRECTIONAL_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    OMNIDIRECTIONAL_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    OMNIDIRECTIONAL_CONTROLLER_PUBLIC
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
protected:
    int motor_number_;
    int state_interface_number_;
    int command_interface_number_;

    bool is_inited_;

    /**
     * @brief get yaw diff in rad
     * 
     */
    double read_yaw_encoder();

    std::shared_ptr<realtime_tools::RealtimePublisher<helios_control_interfaces::msg::MotorStates>> realtime_gimbal_state_pub_;
    rclcpp::Publisher<helios_control_interfaces::msg::MotorStates>::SharedPtr state_pub_;
    
    realtime_tools::RealtimeBox<std::shared_ptr<geometry_msgs::msg::TwistStamped>> received_gimbal_cmd_ptr_;
    realtime_tools::RealtimeBox<std::shared_ptr<std_msgs::msg::Float64>> received_yaw_diff_ptr_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_position_sub_;

    // visualization tools
    visualization_msgs::msg::MarkerArray marker_array_;
    visualization_msgs::msg::Marker chassis_linear_vel_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    double yaw_diff_ = 0;
    // Parameters from ROS for OmnidirectionalController
    std::shared_ptr<ParamsListener> param_listener_;
    Params params_;

    // publish rate limiter
    double publish_rate_ = 1000.0;
    rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
    rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};
    // Timeout to consider cmd commands old
    std::chrono::milliseconds cmd_timeout_{50};

    // // previous 2 commands
    std::queue<geometry_msgs::msg::TwistStamped> previous_commands_;

    bool should_publish_ = false;
    // PID class
    int pid_cnt_ = 0;
    std::map<std::string, double> wheel_res_;
    std::map<std::string, math_utilities::MotorPacket> cmd_map_;
    // velocity solver
    math_utilities::OmnidirectionalSolver velocity_solver_;
    std::vector<double> wheel_velocities_;
    /**
     * @brief Convert the current state of the chassis from state_interfaces to a ROS message
     * @param state_msg The message to be filled
     */
    bool export_state_interfaces(helios_control_interfaces::msg::MotorStates& state_msg);

    bool is_halted_ = false;
    bool subscriber_is_active_ = false;
    bool reset();
    void halt();
    rclcpp::Logger logger_ = rclcpp::get_logger("OMNIDIRECTIONAL_CONTROLLER");
};



} // namespace helios_control