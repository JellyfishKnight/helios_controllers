// created by liuhan on 2023/9/10
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

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_publisher.h"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "helios_control_interfaces/msg/motor_state.hpp"
#include "helios_control_interfaces/msg/motor_states.hpp"
#include "helios_control_interfaces/msg/gimbal_cmd.hpp"
#include "sensor_interfaces/msg/imu_euler.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tf2/convert.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "angles/angles.h"

#include "visibility_control.h"
#include "math_utilities/MotorPacket.hpp"

#include <helios_control_interfaces/msg/detail/gimbal_cmd__struct.hpp>
#include <map>
#include <memory>
#include <vector>
#include <queue>
#include <string>
#include <chrono>

// auto generated by ros2 generate_parameter_library
// https://github.com/PickNikRobotics/generate_parameter_library
#include "gimbal_controller_parameters.hpp"


namespace helios_control {

#define ANGLE_MODE false
#define SPEED_MODE true

constexpr auto DEFAULT_COMMAND_TOPIC = "gimbal_cmd_angle";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "gimbal_cmd_out";

class GimbalController : public controller_interface::ControllerInterface {
public:
    GIMBAL_CONTROLLER_PUBLIC
    GimbalController() = default;

    GIMBAL_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    GIMBAL_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    GIMBAL_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    GIMBAL_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    GIMBAL_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    GIMBAL_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    GIMBAL_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    GIMBAL_CONTROLLER_PUBLIC
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
protected:
    bool is_inited_;
    int motor_number_;
    int state_interface_number_;
    int command_interface_number_;

    std::shared_ptr<realtime_tools::RealtimePublisher<helios_control_interfaces::msg::MotorStates>> realtime_gimbal_state_pub_;
    rclcpp::Publisher<helios_control_interfaces::msg::MotorStates>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_diff_pub_;

    double total_yaw_{};
    double imu_cnt_{};
    sensor_interfaces::msg::ImuEuler last_imu_msg_{};

    realtime_tools::RealtimeBox<std::shared_ptr<helios_control_interfaces::msg::GimbalCmd>> received_gimbal_cmd_ptr_;
    realtime_tools::RealtimeBox<std::shared_ptr<geometry_msgs::msg::TwistStamped>> received_chassis_cmd_ptr_;
    realtime_tools::RealtimeBox<std::shared_ptr<sensor_interfaces::msg::ImuEuler>> received_imu_ptr_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr chassis_cmd_sub_;
    rclcpp::Subscription<sensor_interfaces::msg::ImuEuler>::SharedPtr imu_euler_sub_;
    rclcpp::Subscription<helios_control_interfaces::msg::GimbalCmd>::SharedPtr cmd_sub_;
    // Parameters from ROS for gimbal_controller
    std::shared_ptr<gimbal_controller::ParamListener> param_listener_;
    gimbal_controller::Params params_;

    // publish rate limiter
    double publish_rate_ = 1000.0;
    rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
    rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};
    // Timeout to consider cmd commands old
    std::chrono::milliseconds cmd_timeout_{50};

    // // previous 2 commands
    // std::queue<helios_rs_interfaces::msg::GIMBALMsg> previous_commands_;

    // pid controllers
    std::map<std::string, math_utilities::MotorPacket> cmd_map_;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;

    bool should_publish_ = false;
    /**
     * @brief Convert the current state of the gimbal from state_interfaces to a ROS message
     * @param state_msg The message to be filled
     */
    bool export_state_interfaces(helios_control_interfaces::msg::MotorStates& state_msg);

    bool is_halted_ = false;
    bool subscriber_is_active_ = false;
    bool reset();
    void halt();

    rclcpp::Logger logger_ = rclcpp::get_logger("Gimbal_Controller");
};



} // namespace helios_control