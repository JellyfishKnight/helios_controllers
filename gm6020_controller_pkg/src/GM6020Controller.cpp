#include "GM6020Controller.hpp"

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
        use_stamped_cmd = params_.use_stamped_cmd;
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
        if (use_stamped_cmd) {
            cmd_sub_ = get_node()->create_subscription<rm_interfaces::msg::GM6020Msg>(
                DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(), 
                [this](const std::shared_ptr<rm_interfaces::msg::GM6020Msg> msg)->void {
                    if (!subscriber_is_active) {
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
        
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GM6020Controller::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
        
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type GM6020Controller::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
        
        return controller_interface::return_type::OK;
    }

} // helios_control