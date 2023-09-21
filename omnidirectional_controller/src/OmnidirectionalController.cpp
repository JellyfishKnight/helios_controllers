#include "OmnidirectionalController.hpp"
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>

namespace helios_control {

controller_interface::CallbackReturn OmnidirectionalController::on_init() {
    try {
        param_listener_ = std::make_shared<ParamsListener>(get_node());
        params_ = param_listener_->get_params();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(logger_, "on_init: %s", e.what());
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
    const geometry_msgs::msg::TwistStamped empty_gimbal_msg;
    received_gimbal_cmd_ptr_.set(std::make_shared<geometry_msgs::msg::TwistStamped>(empty_gimbal_msg));

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
    velocity_solver_.solve(*last_command_msg);
    velocity_solver_.get_target_values(front_left_v_, front_right_v_, back_left_v_, back_right_v_);
    ///TODO: set command values
    for (auto motor_name : params_.motor_names) {
        // caculate pid
    }
    return controller_interface::return_type::OK;
}

bool OmnidirectionalController::export_state_interfaces(helios_rs_interfaces::msg::MotorStates& state_msg) {
    ///TODO: export state interfaces
    
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