#include "OmnidirectionalController.hpp"

namespace helios_control {

controller_interface::CallbackReturn OmnidirectionalController::on_init() {
    
}

controller_interface::InterfaceConfiguration OmnidirectionalController::command_interface_configuration() const {
    
}

controller_interface::InterfaceConfiguration OmnidirectionalController::state_interface_configuration() const {
    
}

controller_interface::CallbackReturn OmnidirectionalController::on_configure(const rclcpp_lifecycle::State &previous_state) {

}

controller_interface::CallbackReturn OmnidirectionalController::on_activate(const rclcpp_lifecycle::State & previous_state) {
    
}

controller_interface::CallbackReturn OmnidirectionalController::on_deactivate(const rclcpp_lifecycle::State & previous_state) {

}

controller_interface::CallbackReturn OmnidirectionalController::on_cleanup(const rclcpp_lifecycle::State& previous_state) {

}

controller_interface::return_type OmnidirectionalController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {

}


} // namespace helios_control

// register controller class 
#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  helios_control::OmnidirectionalController, controller_interface::ControllerInterface)