// created by liuhan on 2023/10/8
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

#include "tf2/convert.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"


#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <vector>
#include <queue>
#include <string>
#include <map>

#include "visibility_control.h"
// auto generated by ros2 generate_parameter_library
// https://github.com/PickNikRobotics/generate_parameter_library
#include "imu_controller_parameters.hpp"



namespace helios_control {

constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "/imu_out";

using Params = imu_controller::Params;
using ParamsListener = imu_controller::ParamListener;


class IMUController : public controller_interface::ControllerInterface {
public:
    IMU_CONTROLLER_PUBLIC
    IMUController() = default;

    IMU_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    IMU_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    IMU_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    IMU_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    IMU_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    IMU_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    IMU_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    IMU_CONTROLLER_PUBLIC
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
private:
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>> realtime_imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // Parameters from ROS for IMU
    std::shared_ptr<ParamsListener> param_listener_;
    Params params_;

    // publish rate limiter
    double publish_rate_ = 1000.0;
    rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
    rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};

    // tf2 utitlities
    geometry_msgs::msg::TransformStamped transform_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_pub_;

    double yaw_, pitch_, roll_;

    /**
     * @brief Convert the current state of the chassis from state_interfaces to a ROS message
     * @param state_msg The message to be filled
     */
    bool export_state_interfaces(sensor_msgs::msg::Imu& state_msg);


    bool should_publish_ = false;
    bool is_halted_ = false;
    bool subscriber_is_active_ = false;
    bool reset();
    void halt();


    rclcpp::Logger logger_ = rclcpp::get_logger("imu_controller");
};


} // namespace helios_control