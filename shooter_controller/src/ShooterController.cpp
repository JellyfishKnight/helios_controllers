#include "ShooterController.hpp"

namespace helios_control {

controller_interface::CallbackReturn ShooterController::on_init() {

}

controller_interface::InterfaceConfiguration ShooterController::command_interface_configuration() const {
    
}


controller_interface::InterfaceConfiguration ShooterController::state_interface_configuration() const {
    
}


controller_interface::CallbackReturn ShooterController::on_configure(const rclcpp_lifecycle::State &previous_state) {
    
}


controller_interface::CallbackReturn ShooterController::on_activate(const rclcpp_lifecycle::State & previous_state) {
    
}


controller_interface::CallbackReturn ShooterController::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
    
}


controller_interface::CallbackReturn ShooterController::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
    
}


controller_interface::return_type ShooterController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
    
}

bool reset() {
    return true;
}

void halt() {

}


} // namespace helios_control