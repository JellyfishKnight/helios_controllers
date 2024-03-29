// created by liuhan on 2023/9/10
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
#include "GimbalController.hpp"
#include "gimbal/BaseGimbal.hpp"
#include <angles/angles.h>
#include <cmath>

namespace helios_control {


controller_interface::CallbackReturn GimbalController::on_init() {
    try {
        param_listener_ = std::make_shared<gimbal_controller::ParamListener>(get_node());
        params_ = param_listener_->get_params();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(logger_, "on_init: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    motor_number_ = static_cast<int>(params_.motor_names.size());
    state_interface_number_ = static_cast<int>(params_.motor_state_interfaces.size());
    command_interface_number_ = static_cast<int>(params_.motor_command_interfaces.size());
    // init params
    for (int i = 0; i < motor_number_; i++) {
        math_utilities::MotorPacket motor_packet(
            params_.motor_names[i]
        );
        motor_packet.can_id_ = params_.motor_commands[i * command_interface_number_];
        motor_packet.motor_type_ = static_cast<int>(params_.motor_commands[i * command_interface_number_ + 1]);
        motor_packet.motor_id_ = params_.motor_commands[i * command_interface_number_ + 2];
        motor_packet.motor_mode_ = static_cast<uint8_t>(params_.motor_commands[i * command_interface_number_ + 3]);
        motor_packet.value_ = params_.motor_commands[i * command_interface_number_ + 4];
        // init map
        cmd_map_.emplace(std::pair<std::string, math_utilities::MotorPacket>(params_.motor_names[i], motor_packet));
    }
    if (params_.motor_names.size() != static_cast<std::size_t>(motor_number_)) {
        RCLCPP_ERROR(logger_, "The number of motors is not %d", motor_number_);
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GimbalController::command_interface_configuration() const {
    std::vector<std::string> conf_names;
    for (const auto& joint_name : params_.motor_names) {
        for (const auto & command_name : params_.motor_command_interfaces) {
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
            motor_number_ = static_cast<int>(params_.motor_names.size());
            state_interface_number_ = static_cast<int>(params_.motor_state_interfaces.size());
            command_interface_number_ = static_cast<int>(params_.motor_command_interfaces.size());
            RCLCPP_DEBUG(logger_, "Parameters were updated");
        }

        cmd_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_timeout)};
        if (!reset()) {
            return controller_interface::CallbackReturn::ERROR;
        }
        // create publisher
        state_pub_ = get_node()->create_publisher<helios_control_interfaces::msg::MotorStates>(
            DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS()
        );
        yaw_diff_pub_ = get_node()->create_publisher<std_msgs::msg::Float64>("yaw_diff_i2c", rclcpp::SystemDefaultsQoS());
        realtime_gimbal_state_pub_ = std::make_shared<realtime_tools::RealtimePublisher<helios_control_interfaces::msg::MotorStates>>(
            state_pub_
        );
        // create tf2 transform broadcaster
        dynamic_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this->get_node());
        // create subscribers
        const helios_control_interfaces::msg::GimbalCmd empty_gimbal_msg;
        const sensor_interfaces::msg::ImuEuler empty_imu_msg;
        const geometry_msgs::msg::TwistStamped empty_chassis_msg;
        const std_msgs::msg::Float64 empty_fast_lio_yaw;
        const sensor_interfaces::msg::RobotAim empty_robot_aim_msg;
        received_gimbal_cmd_ptr_.set(std::make_shared<helios_control_interfaces::msg::GimbalCmd>(empty_gimbal_msg));
        received_imu_ptr_.set(std::make_shared<sensor_interfaces::msg::ImuEuler>(empty_imu_msg));
        received_chassis_cmd_ptr_.set(std::make_shared<geometry_msgs::msg::TwistStamped>(empty_chassis_msg));
        received_compensation_yaw_ptr_.set(std::make_shared<std_msgs::msg::Float64>(empty_fast_lio_yaw));
        received_robot_aim_ptr_.set(std::make_shared<sensor_interfaces::msg::RobotAim>(empty_robot_aim_msg));
        // initialize imu subscriber
        imu_euler_sub_ = get_node()->create_subscription<sensor_interfaces::msg::ImuEuler>(
            "imu_euler_out", rclcpp::SystemDefaultsQoS(), 
            [this](const std::shared_ptr<sensor_interfaces::msg::ImuEuler> msg)->void {
                if (!subscriber_is_active_) {
                    RCLCPP_WARN_ONCE(logger_, "Can't accept new imu_euler. subscriber is inactive");
                    return ;
                }
                if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
                    RCLCPP_WARN_ONCE(logger_,
                        "Received TwistStamped with zero timestamp, setting it to current "
                        "time, this message will only be shown once"
                    );
                    msg->header.stamp = get_node()->get_clock()->now();
                }
                double yaw_diff = msg->yaw - last_imu_msg_.yaw;
                if (yaw_diff < -180) {
                    imu_cnt_++;
                }
                else if (yaw_diff > 180) {
                    imu_cnt_--;
                }
                total_yaw_ = msg->yaw + imu_cnt_ * 360 - msg->init_yaw;
                last_imu_msg_ = *msg;
                received_imu_ptr_.set(std::move(msg));
            }
        );

        compensation_yaw_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
            "/fast_lio_yaw", rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                RCLCPP_INFO_ONCE(logger_, "Received Compensation!");
                received_compensation_yaw_ptr_.set(std::move(msg));
            }
        );

        // initialize chassis subscriber
        chassis_cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
            "chassis_cmd_vel", rclcpp::SystemDefaultsQoS(), 
            [this](const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg)->void {
                if (!subscriber_is_active_) {
                    RCLCPP_WARN_ONCE(logger_, "Can't accept new chassis_cmd. subscriber is inactive");
                    return ;
                }
                if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
                    RCLCPP_WARN_ONCE(logger_,
                        "Received TwistStamped with zero timestamp, setting it to current "
                        "time, this message will only be shown once"
                    );
                    msg->header.stamp = get_node()->get_clock()->now();
                }
                // Convert velocity to rpm
                msg->twist.linear.x = msg->twist.linear.x / 60 * 8192 / 1000;
                msg->twist.linear.y = msg->twist.linear.y / 60 * 8192 / 1000;
                msg->twist.angular.z = msg->twist.angular.z / 60 * 8192 / 1000;
                received_chassis_cmd_ptr_.set(std::move(msg));
            }
        );
        
        // initialize command subscriber
        cmd_sub_ = get_node()->create_subscription<helios_control_interfaces::msg::GimbalCmd>(
            DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(), 
            [this](const std::shared_ptr<helios_control_interfaces::msg::GimbalCmd> msg)->void {
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
                // Convert rpm to velocity
                if (msg->gimbal_mode == CRUISE) {
                    msg->yaw_value = msg->yaw_value / 60 * 8192 / 1000;
                    msg->pitch_value = msg->pitch_value / 60 * 8192 / 1000;
                }
                received_gimbal_cmd_ptr_.set(std::move(msg));
            }
        );

        // initialize robot aim subscriber
        robot_aim_sub_ = get_node()->create_subscription<sensor_interfaces::msg::RobotAim>(
            "robot_aim", rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<sensor_interfaces::msg::RobotAim> msg) {
                if (!subscriber_is_active_) {
                    RCLCPP_WARN_ONCE(logger_, "Can't accept new robot_aim. subscriber is inactive");
                    return ;
                }
                if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
                    RCLCPP_WARN_ONCE(logger_,
                        "Received TwistStamped with zero timestamp, setting it to current "
                        "time, this message will only be shown once"
                    );
                    msg->header.stamp = get_node()->get_clock()->now();
                }
                received_robot_aim_ptr_.set(std::move(msg));
            }
        );

        // set publish rate
        publish_rate_ = params_.publish_rate;
        publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
        // create gimbal
        gimbal_ = std::make_shared<SingleGimbal>(params_);
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
        motor_number_ = static_cast<int>(params_.motor_names.size());
        state_interface_number_ = static_cast<int>(params_.motor_state_interfaces.size());
        command_interface_number_ = static_cast<int>(params_.motor_command_interfaces.size());
        gimbal_->update_params(params_);
        RCLCPP_DEBUG(logger_, "Parameters were updated");
    }
    std::shared_ptr<sensor_interfaces::msg::ImuEuler> imu_msg{};
    received_imu_ptr_.get(imu_msg); // get imu message
    if (imu_msg == nullptr) {
        RCLCPP_ERROR(logger_, "imu message received was a nullptr");
        return controller_interface::return_type::ERROR;
    }
    std::shared_ptr<geometry_msgs::msg::TwistStamped> chassis_msg;
    received_chassis_cmd_ptr_.get(chassis_msg); // get chassis message
    if (chassis_msg == nullptr) {
        RCLCPP_ERROR(logger_, "chassis message received was a nullptr");
        return controller_interface::return_type::ERROR;
    }
    //check if command message if nullptr
    std::shared_ptr<helios_control_interfaces::msg::GimbalCmd> last_command_msg;
    received_gimbal_cmd_ptr_.get(last_command_msg);
    if (last_command_msg == nullptr) {
        RCLCPP_ERROR(logger_, "command message received was a nullptr");
        return controller_interface::return_type::ERROR;
    }
    // Get compensation yaw
    std::shared_ptr<std_msgs::msg::Float64> fast_lio_yaw;
    received_compensation_yaw_ptr_.get(fast_lio_yaw);
    double compensation_yaw_diff = 0;
    if (fast_lio_yaw == nullptr) {
        compensation_yaw_diff = 0;
    } else {
        compensation_yaw_diff = angles::to_degrees(fast_lio_yaw->data) - last_imu_msg_.yaw;
    }
    // Get robot aim
    std::shared_ptr<sensor_interfaces::msg::RobotAim> robot_aim;
    received_robot_aim_ptr_.get(robot_aim);
    if (robot_aim == nullptr) {
        RCLCPP_ERROR(logger_, "robot aim message received was a nullptr");
        return controller_interface::return_type::ERROR;
    }
    const auto age_of_last_command = time - last_command_msg->header.stamp;
    // Brake if cmd has timeout, override the stored command
    if (age_of_last_command > cmd_timeout_) {
        last_command_msg->pitch_value = 0;
        last_command_msg->yaw_value = 0;
        return controller_interface::return_type::OK;
    }
    // Check if robot aim has expired or not 
    if (time - robot_aim->header.stamp > cmd_timeout_) {
        RCLCPP_WARN(logger_, "Robot aim message expired");
        robot_aim.reset();
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
    // Update motor value
    gimbal_->update_motors(state_interfaces_, cmd_map_);
    // Set gimbal commands
    gimbal_->set_gimbal_cmd(*last_command_msg, *imu_msg, chassis_msg->twist.angular.z);
    // get diff yaw from imu to chassis
    double diff_yaw_from_imu_to_chassis = 
        gimbal_->caculate_diff_angle_from_imu_to_chassis(compensation_yaw_diff);
    // Turn to heated direction if is cruising
    if (robot_aim) {
        if (gimbal_->get_last_state() == CRUISE) {
            last_command_msg->yaw_value = imu_msg->yaw + 
                diff_yaw_from_imu_to_chassis + robot_aim->armor_id * M_PI_4;
            last_command_msg->pitch_value = 0;
            last_command_msg->gimbal_mode = AUTOAIM;
            gimbal_->set_gimbal_cmd(*last_command_msg, *imu_msg, chassis_msg->twist.angular.z);
        }
    }
    // publish tf2 transform from imu to chassis
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = this->get_node()->now();
    ts.header.frame_id = "imu";
    ts.child_frame_id = "chassis";
    ts.transform.translation.x = 0;
    ts.transform.translation.y = 0;
    ts.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setEuler(0, 0, diff_yaw_from_imu_to_chassis);
    ts.transform.rotation.w = q.w();
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    dynamic_broadcaster_->sendTransform(ts);
    std_msgs::msg::Float64 yaw_diff_msg;
    yaw_diff_msg.data = diff_yaw_from_imu_to_chassis;
    yaw_diff_pub_->publish(yaw_diff_msg);
    // convert into command_interfaces
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

bool GimbalController::export_state_interfaces(helios_control_interfaces::msg::MotorStates& state_msg) {
    state_msg.motor_states.resize(motor_number_, std::numeric_limits<helios_control_interfaces::msg::MotorState>::quiet_NaN());
    state_msg.header.frame_id = "gimbal";
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