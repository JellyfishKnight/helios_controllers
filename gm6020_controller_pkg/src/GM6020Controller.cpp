#include "GM6020Controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace helios_control {

    controller_interface::CallbackReturn GM6020Controller::on_init() {
        try {
            param_listener_ = std::make_shared<gm6020_controller::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(logger_, "on_init: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration GM6020Controller::command_interface_configuration() const {
        std::vector<std::string> conf_names;
        for (const auto& joint_name : params_.motor_names) {
            conf_names.push_back(joint_name + "/" + "current");
        }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::InterfaceConfiguration GM6020Controller::state_interface_configuration() const {
        std::vector<std::string> conf_names;
        for (const auto & joint_name : params_.motor_names) {
            for (const auto & state_name : params_.motor_state_interfaces) {
                conf_names.push_back(joint_name + "/" + state_name);
            }
        }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::CallbackReturn GM6020Controller::on_configure(const rclcpp_lifecycle::State &previous_state) {
        // update parameters if they have changed
        if (param_listener_->is_old(params_)) {
            params_ = param_listener_->get_params();
            RCLCPP_INFO(logger_, "Parameters were updated");
        }
        if (params_.motor_names.size() != 4) {
            RCLCPP_ERROR(logger_, "The number of motors is not 4");
            return controller_interface::CallbackReturn::ERROR;
        }
        cmd_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_timeout)};
        use_stamped_cmd_ = params_.use_stamped_cmd;
        // init speed limter
        limiter_ = SpeedLimiter(
            params_.linear.x.has_velocity_limits,
            params_.linear.x.has_acceleration_limits,
            params_.linear.x.has_jerk_limits,
            params_.linear.x.min_velocity,
            params_.linear.x.max_velocity,
            params_.linear.x.min_acceleration,
            params_.linear.x.max_acceleration,
            params_.linear.x.min_jerk,
            params_.linear.x.max_jerk
        );
        if (!reset()) {
            return controller_interface::CallbackReturn::ERROR;
        }
        // create publisher
        limited_pub_ = get_node()->create_publisher<rm_interfaces::msg::GM6020Msg>(
            DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS()
        );
        realtime_gm6020_pub_ = std::make_shared<realtime_tools::RealtimePublisher<rm_interfaces::msg::GM6020Msg>>(
            limited_pub_
        );

        const rm_interfaces::msg::GM6020Msg empty_gm6020_msg;
        received_gm6020_ptr_.set(std::make_shared<rm_interfaces::msg::GM6020Msg>(empty_gm6020_msg));

        // initialize command subscriber
        if (use_stamped_cmd_) {
            cmd_sub_ = get_node()->create_subscription<rm_interfaces::msg::GM6020Msg>(
                DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(), 
                [this](const std::shared_ptr<rm_interfaces::msg::GM6020Msg> msg)->void {
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
                    received_cmd_msg_ptr_ = std::move(msg);
                }
            );
        } else {
            RCLCPP_ERROR_ONCE(logger_, "Time stamp of the msg is required!");
            return controller_interface::CallbackReturn::ERROR;
        }
        // set publish rate
        publish_rate_ = params_.publish_rate;
        publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GM6020Controller::on_activate(const rclcpp_lifecycle::State & previous_state) {
        is_halted_ = false;
        subscriber_is_active_ = true;
        RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GM6020Controller::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
        subscriber_is_active_ = false;
        if (!is_halted_) {
            halt();
            is_halted_ = true;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GM6020Controller::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
        if (!reset()) {
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type GM6020Controller::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
        if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            if (!is_halted_) {
                halt();
                is_halted_ = true;
            }
            return controller_interface::return_type::OK;
        }
        std::shared_ptr<rm_interfaces::msg::GM6020Msg> last_command_msg;
        received_gm6020_ptr_.get(last_command_msg);
        if (last_command_msg == nullptr) {
            RCLCPP_WARN(logger_, "Command message received was a nullptr");
            return controller_interface::return_type::ERROR;
        }
        const auto age_of_last_command = time - last_command_msg->header.stamp;
        // Brake if cmd has timeout, override the stored command
        if (age_of_last_command > cmd_timeout_) {
            last_command_msg->motor_speed_1 = 0.0;
            last_command_msg->motor_speed_2 = 0.0;
            last_command_msg->motor_speed_3 = 0.0;
            last_command_msg->motor_speed_4 = 0.0;
        }
        // command may be limited futher by SpeedLimit
        // without affecting the stored command
        rm_interfaces::msg::GM6020Msg command = *last_command_msg;
        double command_motor1 = command.motor_speed_1;
        double command_motor2 = command.motor_speed_2;
        double command_motor3 = command.motor_speed_3;
        double command_motor4 = command.motor_speed_4;

        previous_publish_timestamp_ = time;

        bool should_publish = false;
        try {
            if (previous_publish_timestamp_ + publish_period_ < time) {
                previous_publish_timestamp_ += publish_period_;
                should_publish = true;
            }
        } catch (std::runtime_error &) {
            // Handle exceptions when the time source changes and initialize publish timestamp
            previous_publish_timestamp_ = time;
            should_publish = true;
        }
        
        auto & last_command = previous_commands_.back();
        auto & second_to_last_command = previous_commands_.front();
        limiter_.limit(command_motor1, last_command.motor_speed_1, second_to_last_command.motor_speed_1, period.seconds());
        limiter_.limit(command_motor2, last_command.motor_speed_2, second_to_last_command.motor_speed_2, period.seconds());
        limiter_.limit(command_motor3, last_command.motor_speed_3, second_to_last_command.motor_speed_3, period.seconds());
        limiter_.limit(command_motor4, last_command.motor_speed_4, second_to_last_command.motor_speed_4, period.seconds());
        previous_commands_.pop();
        previous_commands_.emplace(command);

        // Publish limited velocity
        if (realtime_gm6020_pub_->trylock()) {
            auto & limited_velocity_command = realtime_gm6020_pub_->msg_;
            limited_velocity_command.header.stamp = time;
            limited_velocity_command.motor_speed_1 = command_motor1;
            limited_velocity_command.motor_speed_2 = command_motor2;
            limited_velocity_command.motor_speed_3 = command_motor3;
            limited_velocity_command.motor_speed_4 = command_motor4;
            realtime_gm6020_pub_->unlockAndPublish();
        }
        // set output 
        command_interfaces_[0].set_value(command_motor1);
        command_interfaces_[1].set_value(command_motor2);
        command_interfaces_[2].set_value(command_motor3);
        command_interfaces_[3].set_value(command_motor4);

        ///TODO: Compute wheels velocities
        return controller_interface::return_type::OK;
    }

    bool GM6020Controller::reset() {
        // release the old queue
        std::queue<rm_interfaces::msg::GM6020Msg> empty;
        std::swap(previous_commands_, empty);

        subscriber_is_active_ = false;
        cmd_sub_.reset();
        received_cmd_msg_ptr_.reset();
        is_halted_ = false;
        return true;
    }

    void GM6020Controller::halt() {

    }

} // helios_control

// register controller class 
#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  helios_control::GM6020Controller, controller_interface::ControllerInterface)