#include "GM6020Controller.hpp"

namespace helios_control {

    controller_interface::return_type GM6020Controller::init(const std::string &controller_name) {
        
        return controller_interface::return_type::OK;
    }

    controller_interface::InterfaceConfiguration GM6020Controller::command_interface_configuration() {
        
        
    }

    controller_interface::InterfaceConfiguration GM6020Controller::state_interface_configuration() {

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