// created by liuhan on 2023/10/8
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

#include "IMUController.hpp"

namespace helios_control {
controller_interface::CallbackReturn IMUController::on_init() {
    
}

controller_interface::InterfaceConfiguration IMUController::command_interface_configuration() const {
    
}

controller_interface::InterfaceConfiguration IMUController::state_interface_configuration() const {
    
}

controller_interface::CallbackReturn IMUController::on_configure(const rclcpp_lifecycle::State &previous_state) {

}

controller_interface::CallbackReturn IMUController::on_activate(const rclcpp_lifecycle::State & previous_state) {
    
}

controller_interface::CallbackReturn IMUController::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
    
}

controller_interface::CallbackReturn IMUController::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
    
}

controller_interface::return_type IMUController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
    
}

bool IMUController::reset() {

}
void IMUController::halt() {
    
}

} // namespace helios_control

// register controller class 
#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  helios_control::IMUController, controller_interface::ControllerInterface)