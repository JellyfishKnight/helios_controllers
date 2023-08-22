#include "GM6020Controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

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
            conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
        }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::InterfaceConfiguration GM6020Controller::state_interface_configuration() const {

    }

    controller_interface::CallbackReturn GM6020Controller::on_configure(const rclcpp_lifecycle::State &previous_state) {

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