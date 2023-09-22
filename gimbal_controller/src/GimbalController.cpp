// created by liuhan on 2023/9/10
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#include "GimbalController.hpp"
#include <helios_rs_interfaces/msg/detail/gimbal_state__struct.hpp>
#include <helios_rs_interfaces/msg/detail/motor_states__struct.hpp>
#include <helios_rs_interfaces/msg/detail/send_data__struct.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace helios_control {


controller_interface::CallbackReturn GimbalController::on_init() {
    try {
        param_listener_ = std::make_shared<gimbal_controller::ParamListener>(get_node());
        params_ = param_listener_->get_params();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(logger_, "on_init: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    for (int i = 0; i < params_.motor_names.size(); i++) {
        MotorCmd cmd = {
            .can_id = params_.motor_commands[i * 4],
            .motor_type = params_.motor_commands[i * 4 + 1],
            .motor_id = params_.motor_commands[i * 4 + 2],
            .value = params_.motor_commands[i * 4 + 3]
        };
        cmd_map_.emplace(std::pair<std::string, MotorCmd>(params_.motor_names[i], cmd));
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GimbalController::command_interface_configuration() const {
    std::vector<std::string> conf_names;
    for (const auto& joint_name : params_.motor_names) {
        for (auto & command_name : params_.motor_command_interfaces) {
            conf_names.push_back(joint_name + "/" + command_name);
        }
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration GimbalController::state_interface_configuration() const {
    std::vector<std::string> conf_names;
    for (const auto & joint_name : params_.motor_names) {
        for (const auto & state_name : params_.motor_state_interfaces) {
            conf_names.push_back(joint_name + "/" + state_name);
        }
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn GimbalController::on_configure(const rclcpp_lifecycle::State &previous_state) {
        // update parameters if they have changed
        if (param_listener_->is_old(params_)) {
            params_ = param_listener_->get_params();
            RCLCPP_INFO(logger_, "Parameters were updated");
        }
        // if (params_.motor_names.size() != 4) {
        //     RCLCPP_ERROR(logger_, "The number of motors is not 4");
        //     return controller_interface::CallbackReturn::ERROR;
        // }

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
        const helios_rs_interfaces::msg::SendData empty_gimbal_msg;
        received_gimbal_cmd_ptr_.set(std::make_shared<helios_rs_interfaces::msg::SendData>(empty_gimbal_msg));

        // initialize command subscriber
        cmd_sub_ = get_node()->create_subscription<helios_rs_interfaces::msg::SendData>(
            DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(), 
            [this](const std::shared_ptr<helios_rs_interfaces::msg::SendData> msg)->void {
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

controller_interface::CallbackReturn GimbalController::on_activate(const rclcpp_lifecycle::State & previous_state) {
    is_halted_ = false;
    subscriber_is_active_ = true;
    RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GimbalController::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
    subscriber_is_active_ = false;
    if (!is_halted_) {
        halt();
        is_halted_ = true;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GimbalController::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
    if (!reset()) {
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GimbalController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
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
        RCLCPP_DEBUG(logger_, "Parameters were updated");
    }
    // check if command message if nullptr
    std::shared_ptr<helios_rs_interfaces::msg::SendData> last_command_msg;
    received_gimbal_cmd_ptr_.get(last_command_msg);
    if (last_command_msg == nullptr) {
        RCLCPP_ERROR(logger_, "command message received was a nullptr");
        return controller_interface::return_type::ERROR;
    }

    const auto age_of_last_command = time - last_command_msg->header.stamp;
    // Brake if cmd has timeout, override the stored command
    // if (age_of_last_command > cmd_timeout_) {
    //     last_command_msg->pitch = 0;
    //     last_command_msg->yaw = 0;
    //     return controller_interface::return_type::OK;
    // }

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
    // publish gimbal states
    // if (should_publish_) {
    //     if (realtime_gimbal_state_pub_->trylock()) {
    //         auto & state_msg = realtime_gimbal_state_pub_->msg_;
    //         state_msg.header.stamp = time;
    //         if (!export_state_interfaces(state_msg)) {
    //             RCLCPP_WARN(logger_, "Could not find some state interfaces");
    //         }
    //         realtime_gimbal_state_pub_->unlockAndPublish();
    //     }
    // }
    // set command values
    // convert into command_interfaces
    for (int i = 0; i < command_interfaces_.size(); i++) {
        auto motor_cmd = cmd_map_.find(command_interfaces_[i].get_prefix_name());
        if (motor_cmd != cmd_map_.end()) {
            if (command_interfaces_[i].get_interface_name() == "can_id") {
                command_interfaces_[i].set_value(motor_cmd->second.can_id);
            } else if (command_interfaces_[i].get_interface_name() == "motor_type") {
                command_interfaces_[i].set_value(motor_cmd->second.motor_type);
            } else if (command_interfaces_[i].get_interface_name() == "motor_id") {
                command_interfaces_[i].set_value(motor_cmd->second.motor_id);
            } else if (command_interfaces_[i].get_interface_name() == "value") {
                command_interfaces_[i].set_value(motor_cmd->second.value);
            }
        }
    }

    return controller_interface::return_type::OK;
}

bool GimbalController::export_state_interfaces(helios_rs_interfaces::msg::MotorStates& state_msg) {
    ///TODO: export state interfaces
    
    return true;
}

bool GimbalController::reset(){

    return true;
}

void GimbalController::halt() {

}

} // namespace helios_control


// register controller class 
#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  helios_control::GimbalController, controller_interface::ControllerInterface)