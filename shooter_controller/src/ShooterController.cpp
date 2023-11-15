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
 .
 
 */
#include "ShooterController.hpp"
#include <cstddef>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_interfaces/msg/detail/power_heat_data__struct.hpp>

namespace helios_control {

controller_interface::CallbackReturn ShooterController::on_init() {
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
    if (params_.motor_names.size() != static_cast<unsigned long>(motor_number_)) {
        RCLCPP_ERROR(logger_, "The number of motors is not %d", motor_number_);
        return controller_interface::CallbackReturn::ERROR;
    }
    if (params_.dial.dial_velocity_level.size() != 10) {
        RCLCPP_ERROR(logger_, "The number of velocity level is not 10");
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ShooterController::command_interface_configuration() const {
    std::vector<std::string> conf_names;
    for (const auto& joint_name : params_.motor_names) {
        for (auto & command_name : params_.motor_command_interfaces) {
            conf_names.push_back(joint_name + "/" + command_name);
        }
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}


controller_interface::InterfaceConfiguration ShooterController::state_interface_configuration() const {
    std::vector<std::string> conf_names;
    for (const auto & joint_name : params_.motor_names) {
        for (const auto & state_name : params_.motor_state_interfaces) {
            conf_names.push_back(joint_name + "/" + state_name);
        }
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}


controller_interface::CallbackReturn ShooterController::on_configure(const rclcpp_lifecycle::State &previous_state) {
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
    state_pub_ = get_node()->create_publisher<helios_control_interfaces::msg::MotorStates>(
        DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS()
    );
    realtime_shooter_state_pub_ = std::make_shared<realtime_tools::RealtimePublisher<helios_control_interfaces::msg::MotorStates>>(
        state_pub_
    );
    // initialize subscribers
    const helios_control_interfaces::msg::ShooterCmd empty_gimbal_msg;
    const sensor_interfaces::msg::PowerHeatData empty_heat_msg;
    received_shooter_cmd_ptr_.set(std::make_shared<helios_control_interfaces::msg::ShooterCmd>(empty_gimbal_msg));
    received_heat_ptr_.set(std::make_shared<sensor_interfaces::msg::PowerHeatData>(empty_heat_msg));
    heat_sub_ = get_node()->create_subscription<sensor_interfaces::msg::PowerHeatData>(
        DEFAULT_HEAT_TOPIC, rclcpp::SensorDataQoS(), 
        [this](sensor_interfaces::msg::PowerHeatData::SharedPtr msg) {
            if (!subscriber_is_active_) {
                RCLCPP_WARN(logger_, "Can't accept new states. subscriber is inactive");
                return ;
            }
            if (msg.get() == nullptr) {
                RCLCPP_WARN(logger_, "Received nullptr heat message");
                return ;
            }
            
            received_heat_ptr_.set(std::move(msg));
        }
    );
    cmd_sub_ = get_node()->create_subscription<helios_control_interfaces::msg::ShooterCmd>(
        DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(), 
        [this](helios_control_interfaces::msg::ShooterCmd::SharedPtr msg)->void {
            if (!subscriber_is_active_) {
                RCLCPP_WARN(logger_, "Can't accept new commands. subscriber is inactive");
                return ;
            }
            if (msg.get() == nullptr) {
                RCLCPP_WARN(logger_, "Received nullptr command message");
                return ;
            }
            received_shooter_cmd_ptr_.set(std::move(msg));
        }
    );
    // set publish rate
    publish_rate_ = params_.publish_rate;
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn ShooterController::on_activate(const rclcpp_lifecycle::State & previous_state) {
    is_halted_ = false;
    subscriber_is_active_ = true;
    RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn ShooterController::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
    subscriber_is_active_ = false;
    if (!is_halted_) {
        halt();
        is_halted_ = true;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn ShooterController::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
    if (!reset()) {
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::return_type ShooterController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
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
    math_utilities::MotorPacket::get_moto_measure(state_interfaces_, cmd_map_);
    // block detection and resolve
    auto dial_up = cmd_map_.find("dial_up");
    auto dial_down = cmd_map_.find("dial_down");
    if (dial_up->second.is_blocked(params_.dial.dial_block_cnt_limit, params_.dial.dial_current_limit) || 
        dial_down->second.is_blocked(params_.dial.dial_block_cnt_limit, params_.dial.dial_current_limit)) {
        RCLCPP_WARN(logger_, "Dial is blocked");
        dial_up->second.solve_block_mode(static_cast<uint32_t>(params_.dial.count_clock_wise_angle));
        dial_down->second.solve_block_mode(static_cast<uint32_t>(params_.dial.count_clock_wise_angle));
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
    // check if command message if nullptr
    received_shooter_cmd_ptr_.get(last_command_msg);
    received_heat_ptr_.get(last_heat_msg);
    ///TODO: no referee system for now
    last_heat_msg = std::make_shared<sensor_interfaces::msg::PowerHeatData>();
    last_heat_msg->shooter_id1_17mm_residual_cooling_heat = 10000;
    if (last_command_msg == nullptr) {
    // if (last_command_msg == nullptr || last_heat_msg == nullptr) {
        RCLCPP_ERROR(logger_, "command message received was a nullptr");
        return controller_interface::return_type::ERROR;
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
        if (realtime_shooter_state_pub_->trylock()) {
            auto & state_msg = realtime_shooter_state_pub_->msg_;
            state_msg.header.stamp = time;
            if (!export_state_interfaces(state_msg)) {
                RCLCPP_WARN(logger_, "Could not find some state interfaces");
            }
            realtime_shooter_state_pub_->unlockAndPublish();
        }
    }
    // caculate shooter speed
    double velocity_rpm = 0;
    if (last_command_msg->shooter_mode == SHOOTER_STOP) {
        velocity_rpm = 0;
    } else if (last_command_msg->shooter_mode == SHOOTER_HIGH_VELOCITY) {
        velocity_rpm = params_.shooter.high_velocity;
    }
    auto shooter_left_up = cmd_map_.find("shooter_left_up");
    auto shooter_left_down = cmd_map_.find("shooter_left_down");
    auto shooter_right_up = cmd_map_.find("shooter_right_up");
    auto shooter_right_down = cmd_map_.find("shooter_right_down");
    shooter_left_up->second.value_ = velocity_rpm;
    shooter_left_down->second.value_ = -velocity_rpm;
    shooter_right_up->second.value_ = velocity_rpm;
    shooter_right_down->second.value_ = -velocity_rpm;
    // caculate dial speed or angle
    // check if heat has run out
    if (last_command_msg->dial_mode == DIAL_STOP ||
        last_heat_msg->shooter_id1_17mm_residual_cooling_heat < params_.heat_limit) {
        dial_up->second.value_ = dial_down->second.value_ = 0;
    } else if (last_command_msg->dial_mode == DIAL_CLOCKWISE) {
        double velocity = params_.dial.dial_velocity_level[last_command_msg->dial_velocity_level];
        dial_up->second.value_ = last_command_msg->fire_flag == 1 ? velocity : 0;
        dial_down->second.value_ = last_command_msg->fire_flag == 1 ? velocity : 0;
        dial_up->second.motor_mode_ = dial_down->second.motor_mode_ = 0x01;
    } else if (last_command_msg->dial_mode == DIAL_COUNT_CLOCKWISE) {
        if (last_command_msg->fire_flag == 1) {
            dial_up->second.value_ = dial_up->second.total_angle_ + params_.dial.count_clock_wise_angle;
            dial_down->second.value_ = dial_up->second.total_angle_ + params_.dial.count_clock_wise_angle;
            dial_up->second.motor_mode_ = dial_down->second.motor_mode_ = 0x02;
        }
    }
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

bool ShooterController::export_state_interfaces(helios_control_interfaces::msg::MotorStates& state_msg) {
    state_msg.motor_states.resize(motor_number_, std::numeric_limits<helios_control_interfaces::msg::MotorState>::quiet_NaN());
    state_msg.header.frame_id = "shooter";
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

bool ShooterController::reset() {
    return true;
}

void ShooterController::halt() {

}


} // namespace helios_control

// register controller class 
#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  helios_control::ShooterController, controller_interface::ControllerInterface)