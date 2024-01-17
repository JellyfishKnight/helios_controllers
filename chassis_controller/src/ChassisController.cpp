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
#include "ChassisController.hpp"


namespace helios_control {

controller_interface::CallbackReturn ChassisController::on_init() {
    // create params
    try {
        param_listener_ = std::make_shared<ParamsListener>(get_node());
        params_ = param_listener_->get_params();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(logger_, "on_init: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    // Check the type of chassis
    if (params_.chassis_type == "omnidirectional") {
        chassis_solver_ = std::make_shared<OmnidirectionalSolver>();
        motor_names_ = params_.motor_names.omnidirectional;
        motor_number_ = static_cast<int>(params_.motor_names.omnidirectional.size());
        command_interface_number_ = static_cast<int>(params_.motor_command_interfaces.size());
        // Init Motors
        for (int i = 0; i < motor_number_; i++) {
            math_utilities::MotorPacket motor_packet(
                params_.motor_names.omnidirectional[i]
            );
            motor_packet.can_id_ = params_.motor_commands.omnidirectional[i * command_interface_number_];
            motor_packet.motor_type_ = static_cast<int>(params_.motor_commands.omnidirectional[i * command_interface_number_ + 1]);
            motor_packet.motor_id_ = params_.motor_commands.omnidirectional[i * command_interface_number_ + 2];
            motor_packet.motor_mode_ = 0x01;
            motor_packet.value_ = params_.motor_commands.omnidirectional[i * command_interface_number_ + 4];

            cmd_map_.emplace(std::pair<std::string, math_utilities::MotorPacket>(motor_names_[i], motor_packet));
        }
    } else {
        ///TODO: Complete other chassis type

    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ChassisController::command_interface_configuration() const {
    std::vector<std::string> conf_names;
    for (const auto& joint_name : motor_names_) {
        for (auto & command_name : params_.motor_command_interfaces) {
            conf_names.push_back(joint_name + "/" + command_name);
        }
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};

}

controller_interface::InterfaceConfiguration ChassisController::state_interface_configuration() const {
    std::vector<std::string> conf_names;
    for (const auto & joint_name : motor_names_) {
        for (const auto & state_name : params_.motor_state_interfaces) {
            conf_names.push_back(joint_name + "/" + state_name);
        }
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn ChassisController::on_configure(const rclcpp_lifecycle::State &previous_state) {
    if (!reset()) {
        return controller_interface::CallbackReturn::ERROR;
    }
    cmd_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_timeout)};
    // create publisher
    state_pub_ = get_node()->create_publisher<helios_control_interfaces::msg::MotorStates>(
        DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS()
    );
    realtime_gimbal_state_pub_ = std::make_shared<realtime_tools::RealtimePublisher<helios_control_interfaces::msg::MotorStates>>(
        state_pub_
    );
    geometry_msgs::msg::TwistStamped empty_gimbal_msg{};
    received_gimbal_cmd_ptr_.set(std::make_shared<geometry_msgs::msg::TwistStamped>(empty_gimbal_msg));
    // initialize yaw diff subscriber
    yaw_position_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
        "yaw_diff_i2c", rclcpp::SystemDefaultsQoS(), [this](std_msgs::msg::Float64::SharedPtr msg)->void {
            if (!subscriber_is_active_) {
                RCLCPP_WARN_ONCE(logger_, "Can't accept new commands. subscriber is inactive");
                return ;
            }
            received_yaw_diff_ptr_.set(std::move(msg));
        }
    );
    // initialize command subscriber
    cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
        DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(), 
        [this](geometry_msgs::msg::TwistStamped::SharedPtr msg)->void {
            if (!subscriber_is_active_) {
                RCLCPP_WARN_ONCE(logger_, "Can't accept new commands. subscriber is inactive");
                return ;
            }
            if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
                RCLCPP_WARN_ONCE(logger_,
                    "Received TwistStamped with zero timestamp, setting it to current "
                    "time, this message will only be shown once"
                );
                msg->header.stamp = get_node()->get_clock()->now();
            }
            if (msg->header.frame_id.empty()) {
                RCLCPP_WARN_ONCE(logger_, "Received TwistStamped without frame_id, default set to imu");
                msg->header.frame_id = "imu";
            }
            float total_linear_vel = std::sqrt(
                msg->twist.linear.x * msg->twist.linear.x + msg->twist.linear.y * msg->twist.linear.y
            );
            if (total_linear_vel > params_.max_linear_vel) {
                RCLCPP_WARN_ONCE(logger_, "Received TwistStamped with linear velocity %f, which is over the limit %f",
                    total_linear_vel, params_.max_linear_vel
                );
                msg->twist.linear.x = msg->twist.linear.x / total_linear_vel * params_.max_linear_vel;
                msg->twist.linear.y = msg->twist.linear.y / total_linear_vel * params_.max_linear_vel;
            }
            if (std::abs(msg->twist.angular.z) > params_.max_angular_vel) {
                RCLCPP_WARN_ONCE(logger_, "Received TwistStamped with angular velocity %f, which is over the limit %f",
                    msg->twist.angular.z, params_.max_angular_vel
                );
                msg->twist.angular.z = msg->twist.angular.z / msg->twist.angular.z * params_.max_angular_vel;
            }
            // Convert rpm to velocity
            msg->twist.linear.x = msg->twist.linear.x / 60 * 8192 / 1000;
            msg->twist.linear.y = msg->twist.linear.y / 60 * 8192 / 1000;
            msg->twist.angular.z = msg->twist.angular.z / 60 * 8192 / 1000;
            received_gimbal_cmd_ptr_.set(std::move(msg));
        }
    );
    // set publish rate
    publish_rate_ = params_.publish_rate;
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChassisController::on_activate(const rclcpp_lifecycle::State & previous_state) {
    is_halted_ = false;
    subscriber_is_active_ = true;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChassisController::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
    subscriber_is_active_ = false;
    if (!is_halted_) {
        halt();
        is_halted_ = true;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChassisController::on_cleanup(const rclcpp_lifecycle::State& previous_state) {

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ChassisController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
    if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        if (!is_halted_) {
            halt();
            is_halted_ = true;
        }
        return controller_interface::return_type::OK;
    }
    /// Update params if they have changed
    if (param_listener_->is_old(params_)) {
        params_ = param_listener_->get_params();
    }
    /// Check if command message is null or time out
    // If is null
    std::shared_ptr<geometry_msgs::msg::TwistStamped> last_command_msg;
    std::shared_ptr<std_msgs::msg::Float64> last_yaw_diff_msg;
    received_yaw_diff_ptr_.get(last_yaw_diff_msg);
    received_gimbal_cmd_ptr_.get(last_command_msg);
    if (last_command_msg == nullptr || last_yaw_diff_msg == nullptr) {
        RCLCPP_ERROR_ONCE(logger_, "command message or yaw diff received was a nullptr");
        return controller_interface::return_type::ERROR;
    }
    // If is timeout
    const auto age_of_last_command = time - last_command_msg->header.stamp;
    // Brake if cmd has timeout, override the stored command
    if (age_of_last_command > cmd_timeout_) {
        last_command_msg->twist.linear.x = 0.0;
        last_command_msg->twist.linear.y = 0.0;
        last_command_msg->twist.linear.z = 0.0;
        last_command_msg->twist.angular.x = 0.0;
        last_command_msg->twist.angular.y = 0.0;
        last_command_msg->twist.angular.z = 0.0;
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
    /// Get the current state of the chassis
    math_utilities::MotorPacket::get_moto_measure(state_interfaces_, cmd_map_);
    /// Publish chassis states
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
    /// Chassis solver
    // solve geometry relation
    chassis_solver_->solve_geometry(*last_command_msg, last_yaw_diff_msg->data);
    // get target value
    chassis_solver_->get_target_values(cmd_map_);
    /// Send target value
    for (std::size_t i = 0; i < command_interfaces_.size(); i++) {
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

bool ChassisController::export_state_interfaces(helios_control_interfaces::msg::MotorStates& state_msg) {
    state_msg.motor_states.resize(motor_number_, std::numeric_limits<helios_control_interfaces::msg::MotorState>::quiet_NaN());
    state_msg.header.frame_id = "chassis";
    state_msg.header.stamp = this->get_node()->now();
    for (int i = 0; i < motor_number_; i++) {
        const auto & motor_packet = cmd_map_.find(motor_names_[i]);
        if (motor_packet != cmd_map_.end()) {
            motor_packet->second.set_state_msg(state_msg.motor_states[i]);
        } else {
            RCLCPP_ERROR(logger_, "%s not found", motor_names_[i].c_str());
            return false;
        }
    }
    return true;
}

bool ChassisController::reset() {

    return true;
}


void ChassisController::halt() {

}



} // namespace helios_control

// register controller class 
#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  helios_control::ChassisController, controller_interface::ControllerInterface);