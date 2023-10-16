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
#include "OmnidirectionalController.hpp"
#include <asm-generic/errno.h>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <helios_rs_interfaces/msg/detail/motor_state__struct.hpp>
#include <limits>
#include <math_utilities/MotorPacket.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <utility>
#include <vector>

namespace helios_control {


controller_interface::CallbackReturn OmnidirectionalController::on_init() {
    // create params
    try {
        param_listener_ = std::make_shared<ParamsListener>(get_node());
        params_ = param_listener_->get_params();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(logger_, "on_init: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    motor_number_ = static_cast<int>(params_.motor_names.size());
    state_interface_number_ = static_cast<int>(params_.motor_state_interfaces.size());
    command_interface_number_ = static_cast<int>(params_.motor_command_interfaces.size());
    wheel_velocities_.resize(motor_number_, std::numeric_limits<double>::quiet_NaN());
    // init params
    for (int i = 0; i < motor_number_; i++) {
        math_utilities::MotorPacket motor_packet(
            params_.motor_names[i],
            params_.motor_commands[i * command_interface_number_ + 4]
        );
        motor_packet.can_id_ = params_.motor_commands[i * command_interface_number_];
        motor_packet.motor_type_ = static_cast<int>(params_.motor_commands[i * command_interface_number_ + 1]);
        motor_packet.motor_id_ = params_.motor_commands[i * command_interface_number_ + 2];
        motor_packet.motor_mode_ = static_cast<uint8_t>(params_.motor_commands[i * command_interface_number_ + 3]);
        motor_packet.value_ = params_.motor_commands[i * command_interface_number_ + 4];
        // RCLCPP_DEBUG the motor packet info
        RCLCPP_DEBUG(logger_, "can_id %d, motor_type: %d, motor_id: %d", motor_packet.can_id_, motor_packet.motor_type_, motor_packet.motor_id_);
        // init map
        cmd_map_.emplace(std::pair<std::string, math_utilities::MotorPacket>(params_.motor_names[i], motor_packet));
    }
    if (params_.motor_names.size() != motor_number_) {
        RCLCPP_ERROR(logger_, "The number of motors is not %d", motor_number_);
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration OmnidirectionalController::command_interface_configuration() const {
    std::vector<std::string> conf_names;
    for (const auto& joint_name : params_.motor_names) {
        for (auto & command_name : params_.motor_command_interfaces) {
            conf_names.push_back(joint_name + "/" + command_name);
        }
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration OmnidirectionalController::state_interface_configuration() const {
    std::vector<std::string> conf_names;
    for (const auto & joint_name : params_.motor_names) {
        for (const auto & state_name : params_.motor_state_interfaces) {
            conf_names.push_back(joint_name + "/" + state_name);
        }
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn OmnidirectionalController::on_configure(const rclcpp_lifecycle::State &previous_state) {
    // update parameters if they have changed
    if (param_listener_->is_old(params_)) {
        params_ = param_listener_->get_params();
        motor_number_ = static_cast<int>(params_.motor_names.size());
        RCLCPP_INFO(logger_, "Parameters were updated");
    }
    cmd_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_timeout)};
    if (!reset()) {
        return controller_interface::CallbackReturn::ERROR;
    }
    // create publisher
    state_pub_ = get_node()->create_publisher<helios_rs_interfaces::msg::MotorStates>(
        DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS()
    );
    realtime_gimbal_state_pub_ = std::make_shared<realtime_tools::RealtimePublisher<helios_rs_interfaces::msg::MotorStates>>(
        state_pub_
    );
    geometry_msgs::msg::TwistStamped empty_gimbal_msg;
    empty_gimbal_msg.twist.linear.x = 0;
    empty_gimbal_msg.twist.linear.y = 0;
    empty_gimbal_msg.twist.linear.z = 0;
    empty_gimbal_msg.twist.angular.x = 0;
    empty_gimbal_msg.twist.angular.y = 0;
    empty_gimbal_msg.twist.angular.z = 0;
    received_gimbal_cmd_ptr_.set(std::make_shared<geometry_msgs::msg::TwistStamped>(empty_gimbal_msg));

    yaw_position_sub_ = get_node()->create_subscription<helios_rs_interfaces::msg::MotorStates>(
        "gimbal_cmd_out", rclcpp::SystemDefaultsQoS(),
        [this](helios_rs_interfaces::msg::MotorStates::SharedPtr msg)->void {
            if (!subscriber_is_active_) {
                RCLCPP_WARN(logger_, "Can't accept yaw state. subscriber is inactive");
                return ;
            }
            if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
                RCLCPP_WARN_ONCE(logger_,
                    "Received MotorStates with zero timestamp, setting it to current "
                    "time, this message will only be shown once"
                );
                msg->header.stamp = get_node()->get_clock()->now();
            }
            for (int i = 0; i < msg->motor_states.size(); i++) {
                if (msg->motor_states[i].full_name == "yaw") {
                    yaw_position_ = msg->motor_states[i].total_angle;
                    break;
                }
            }
        }
    );

    // initialize command subscriber
    cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
        DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(), 
        [this](geometry_msgs::msg::TwistStamped::SharedPtr msg)->void {
            if (!subscriber_is_active_) {
                RCLCPP_WARN(logger_, "Can't accept new commands. subscriber is inactive");
                return ;
            }
            if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
                RCLCPP_WARN_ONCE(logger_,
                    "Received TwistStamped with zero timestamp, setting it to current "
                    "time, this message will only be shown once"
                );
                msg->header.stamp = get_node()->get_clock()->now();
            }
            received_gimbal_cmd_ptr_.set(std::move(msg));
        }
    );
    // set publish rate
    publish_rate_ = params_.publish_rate;
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmnidirectionalController::on_activate(const rclcpp_lifecycle::State & previous_state) {
    is_halted_ = false;
    subscriber_is_active_ = true;
    RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmnidirectionalController::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
    subscriber_is_active_ = false;
    if (!is_halted_) {
        halt();
        is_halted_ = true;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmnidirectionalController::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
    if (!reset()) {
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type OmnidirectionalController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
    if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        if (!is_halted_) {
            halt();
            is_halted_ = true;
        }
        return controller_interface::return_type::OK;
    }
    // update params if they have changed
    if (param_listener_->is_old(params_)) {
        params_ = param_listener_->get_params();
        motor_number_ = static_cast<int>(params_.motor_names.size());
        state_interface_number_ = static_cast<int>(params_.motor_state_interfaces.size());
        command_interface_number_ = static_cast<int>(params_.motor_command_interfaces.size());
        RCLCPP_DEBUG(logger_, "Parameters were updated");
    }
    // check if command message if nullptr
    std::shared_ptr<geometry_msgs::msg::TwistStamped> last_command_msg;
    received_gimbal_cmd_ptr_.get(last_command_msg);
    if (last_command_msg == nullptr) {
        RCLCPP_ERROR(logger_, "command message received was a nullptr");
        return controller_interface::return_type::ERROR;
    }

    const auto age_of_last_command = time - last_command_msg->header.stamp;
    // Brake if cmd has timeout, override the stored command
    if (age_of_last_command > cmd_timeout_) {
        last_command_msg->twist.linear.x = 0.0;
        last_command_msg->twist.linear.y = 0.0;
        last_command_msg->twist.linear.z = 0.0;
        last_command_msg->twist.angular.x = 0.0;
        last_command_msg->twist.angular.y = 0.0;
        last_command_msg->twist.angular.z = 0.0;
        return controller_interface::return_type::OK;
    }

    try {
        if (previous_publish_timestamp_ + publish_period_ < time) {
            previous_publish_timestamp_ += publish_period_;
            should_publish_ = true;
        }
    } catch (std::runtime_error &e) {
        // Handle exceptions when the time source changes and initialize publish timestamp
        previous_publish_timestamp_ = time;
        should_publish_ = true;
    }
    // get motor measured
    math_utilities::MotorPacket::get_moto_measure(state_interfaces_, cmd_map_);
    // publish gimbal states
    if (should_publish_) {
        if (realtime_gimbal_state_pub_->trylock()) {
            auto & state_msg = realtime_gimbal_state_pub_->msg_;
            state_msg.header.stamp = time;
            if (!export_state_interfaces(state_msg)) {
                RCLCPP_WARN(logger_, "Could not find some state interfaces");
            }
            realtime_gimbal_state_pub_->unlockAndPublish();
        }
    }
    // omnidirectional wheels solve
    velocity_solver_.solve(*last_command_msg, read_yaw_encoder());
    // front_left_v_, front_right_v_, back_left_v_, back_right_v_
    velocity_solver_.get_target_values(wheel_velocities_[0], wheel_velocities_[1], wheel_velocities_[2], wheel_velocities_[3]);
    for (int i = 0; i < motor_number_; i++) {
        auto motor = cmd_map_.find(params_.motor_names[i]);
        motor->second.value_ = wheel_velocities_[i];
        motor->second.set_motor_speed(wheel_velocities_[i]);
        RCLCPP_WARN(logger_, "%s: %f", params_.motor_names[i].c_str(), wheel_velocities_[i]);
    }
    // convert into command_interfaces
    for (int i = 0; i < command_interfaces_.size(); i++) {
        auto motor_cmd = cmd_map_.find(command_interfaces_[i].get_prefix_name());
        if (motor_cmd != cmd_map_.end()) {
            if (command_interfaces_[i].get_interface_name() == "can_id") {
                command_interfaces_[i].set_value(motor_cmd->second.can_id_);
            } else if (command_interfaces_[i].get_interface_name() == "motor_type") {
                command_interfaces_[i].set_value(motor_cmd->second.motor_type_);
            } else if (command_interfaces_[i].get_interface_name() == "motor_id") {
                command_interfaces_[i].set_value(motor_cmd->second.motor_id_);
            } else if (command_interfaces_[i].get_interface_name() == "motor_mode") {
                command_interfaces_[i].set_value(motor_cmd->second.motor_mode_);
            } else if (command_interfaces_[i].get_interface_name() == "motor_value") {
                command_interfaces_[i].set_value(motor_cmd->second.value_);
            }
        }
    }
    return controller_interface::return_type::OK;
}

double OmnidirectionalController::read_yaw_encoder() {
    // get yaw diff in rad
    // RCLCPP_INFO(logger_, "yaw: %f", (yaw_position_ - params_.yaw_mid_angle) / 8196.0 * M_PI * 2);
    return (yaw_position_ - params_.yaw_mid_angle) / (8192.0 * 1.5) * M_PI * 2 - 1800.0 / (8192.0 * 1.5) * M_PI * 2;
}

bool OmnidirectionalController::export_state_interfaces(helios_rs_interfaces::msg::MotorStates& state_msg) {
    state_msg.motor_states.resize(motor_number_, std::numeric_limits<helios_rs_interfaces::msg::MotorState>::quiet_NaN());
    state_msg.header.frame_id = "chassis";
    state_msg.header.stamp = this->get_node()->now();
    for (int i = 0; i < motor_number_; i++) {
        const auto & motor_packet = cmd_map_.find(params_.motor_names[i]);
        if (motor_packet != cmd_map_.end()) {
            motor_packet->second.set_state_msg(state_msg.motor_states[i]);
        } else {
            RCLCPP_ERROR(logger_, "%s not found", params_.motor_names[i].c_str());
            return false;
        }
    }
    return true;
}

bool OmnidirectionalController::reset(){
    
    return true;
}

void OmnidirectionalController::halt() {

}

} // namespace helios_control

// register controller class 
#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  helios_control::OmnidirectionalController, controller_interface::ControllerInterface)