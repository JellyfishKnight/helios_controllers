#pragma once

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_tools/realtime_publisher.h"
#include "rm_interfaces/msg/gm6020_msg.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "visibility_control.h"
#include "SpeedLimiter.hpp"

#include <vector>
#include <queue>
#include <string>

// auto generated by ros2 generate_parameter_library
// https://github.com/PickNikRobotics/generate_parameter_library
#include "gm6020_controller_parameters.hpp"

namespace helios_control {

constexpr auto DEFAULT_COMMAND_TOPIC = "gm6020/cmd";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "gm6020/cmd_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "gm6020/cmd_out";


class GM6020Controller : public controller_interface::ControllerInterface {
public:
    GM6020_CONTROLLER_PUBLIC
    GM6020Controller();

    GM6020_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    GM6020_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    GM6020_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    GM6020_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    GM6020_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    GM6020_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    GM6020_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    GM6020_CONTROLLER_PUBLIC
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
protected:
    std::shared_ptr<realtime_tools::RealtimePublisher<rm_interfaces::msg::GM6020Msg>> realtime_gm6020_pub_;
    rclcpp::Publisher<rm_interfaces::msg::GM6020Msg>::SharedPtr limited_pub_;

    realtime_tools::RealtimeBox<std::shared_ptr<rm_interfaces::msg::GM6020Msg>> received_gm6020_ptr_;
    
    rclcpp::Subscription<rm_interfaces::msg::GM6020Msg>::SharedPtr cmd_sub_;
    std::shared_ptr<rm_interfaces::msg::GM6020Msg> received_cmd_msg_ptr_;
    // Parameters from ROS for gm6020_controller
    std::shared_ptr<gm6020_controller::ParamListener> param_listener_;
    gm6020_controller::Params params_;

    // publish rate limiter
    double publish_rate_ = 1000.0;
    rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
    rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};
    // Timeout to consider cmd commands old
    std::chrono::milliseconds cmd_timeout_{500};

    // previous 2 commands
    std::queue<rm_interfaces::msg::GM6020Msg> previous_commands_;

    SpeedLimiter limiter_;

    bool is_halted_ = false;
    bool use_stamped_cmd_ = true;
    bool subscriber_is_active_ = false;
    bool reset();

    void halt();

    rclcpp::Logger logger_ = rclcpp::get_logger("GM6020_Controller");
};

} // helios_control
