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
  // create params
  try {
    param_listener_ = std::make_shared<ParamsListener>(this->get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration IMUController::command_interface_configuration() const {
  std::vector<std::string> conf_names;
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration IMUController::state_interface_configuration() const {
  std::vector<std::string> conf_names;
  for (const auto & state_name : params_.state_interfaces) {
    conf_names.push_back(params_.sensor_name + "/" + state_name);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn IMUController::on_configure(const rclcpp_lifecycle::State &previous_state) {
  // update parameters if they have changed
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger_, "Parameters were updated");
  }
  if (!reset()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  // create publisher
  imu_pub_ = get_node()->create_publisher<sensor_msgs::msg::Imu>(
    DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS()
  );
  realtime_imu_pub_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>>(
    imu_pub_
  );
  static_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this->get_node());
  dynamic_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this->get_node());
  // set publish rate
  publish_rate_ = params_.publish_rate;
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IMUController::on_activate(const rclcpp_lifecycle::State & previous_state) {
  is_halted_ = false;
  subscriber_is_active_ = true;
  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IMUController::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
  subscriber_is_active_ = false;
  if (!is_halted_) {
    halt();
    is_halted_ = true;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IMUController::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
  if (!reset()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type IMUController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
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
    RCLCPP_DEBUG(logger_, "Parameters were updated");
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
    if (realtime_imu_pub_->trylock()) {
      auto & state_msg = realtime_imu_pub_->msg_;
      geometry_msgs::msg::TransformStamped transform_stamped;
      state_msg.header.stamp = time;
      if (!export_state_interfaces(state_msg)) {
        RCLCPP_WARN(logger_, "Could not find some state interfaces");
      } else {
        double x, y, z, w;
        x = state_msg.orientation.x;
        y = state_msg.orientation.y;
        z = state_msg.orientation.z;
        w = state_msg.orientation.w;
        // publish imu sensor data
        realtime_imu_pub_->unlockAndPublish();
        // transform from imu to yaw
        transform_stamped.header.frame_id = params_.imu_frame_id;
        transform_stamped.child_frame_id = params_.yaw_frame_id;
        transform_stamped.header.stamp = realtime_imu_pub_->msg_.header.stamp;
        tf2::Quaternion q;
        q.setEuler(atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y)), 0, 0);
        transform_stamped.transform.translation.x = params_.imu_to_yaw_joint_tvec[0];
        transform_stamped.transform.translation.y = params_.imu_to_yaw_joint_tvec[1];
        transform_stamped.transform.translation.z = params_.imu_to_yaw_joint_tvec[2];
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();
        dynamic_pub_->sendTransform(transform_stamped);
        // transform from yaw to pitch
        transform_stamped.header.frame_id = params_.yaw_frame_id;
        transform_stamped.child_frame_id = params_.pitch_frame_id;
        q.setEuler(0, std::asin(2 * (w * y - x * z)), 0);
        transform_stamped.transform.translation.x = params_.yaw_joint_to_pitch_joint_tvec[0];
        transform_stamped.transform.translation.y = params_.yaw_joint_to_pitch_joint_tvec[1];
        transform_stamped.transform.translation.z = params_.yaw_joint_to_pitch_joint_tvec[2];
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();
        dynamic_pub_->sendTransform(transform_stamped);
        // transform from pitch to camera
        transform_stamped.header.frame_id = params_.pitch_frame_id;
        transform_stamped.child_frame_id = params_.camera_frame_id;
        q.setEuler(params_.pitch_joint_to_camera_joint_rotate_euler[0], 
          params_.pitch_joint_to_camera_joint_rotate_euler[1],
          params_.pitch_joint_to_camera_joint_rotate_euler[2]);
        transform_stamped.transform.translation.x = params_.yaw_joint_to_pitch_joint_tvec[0];
        transform_stamped.transform.translation.y = params_.yaw_joint_to_pitch_joint_tvec[1];
        transform_stamped.transform.translation.z = params_.yaw_joint_to_pitch_joint_tvec[2];
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();
        dynamic_pub_->sendTransform(transform_stamped);
      }
    }
  }
  return controller_interface::return_type::OK;
}

bool IMUController::reset() {
  static_pub_.reset();
  dynamic_pub_.reset();
  imu_pub_.reset();
  realtime_imu_pub_.reset(); 
  return true;
}
void IMUController::halt() {
    
}

bool IMUController::export_state_interfaces(sensor_msgs::msg::Imu& state_msg) {
  state_msg.header.stamp = this->get_node()->now();
  state_msg.header.frame_id = params_.imu_frame_id;
  // get state interfaces
  for (const auto& state : state_interfaces_) {
    if (state.get_prefix_name() == "imu") {
      if (state.get_interface_name() == "x") {
        state_msg.orientation.x = state.get_value();
      } else if (state.get_interface_name() == "y") {
        state_msg.orientation.y = state.get_value();
      } else if (state.get_interface_name() == "z") {
        state_msg.orientation.z = state.get_value();
      } else if (state.get_interface_name() == "w") {
        state_msg.orientation.w = state.get_value();
      } else if (state.get_interface_name() == "angular_velocity_x") {
        state_msg.angular_velocity.x = state.get_value();
      } else if (state.get_interface_name() == "angular_velocity_y") {
        state_msg.angular_velocity.y = state.get_value();
      } else if (state.get_interface_name() == "angular_velocity_z") {
        state_msg.angular_velocity.z = state.get_value();
      } else if (state.get_interface_name() == "linear_acceleration_x") {
        state_msg.linear_acceleration.x = state.get_value();
      } else if (state.get_interface_name() == "linear_acceleration_y") {
        state_msg.linear_acceleration.y = state.get_value();
      } else if (state.get_interface_name() == "linear_acceleration_z") {
        state_msg.linear_acceleration.z = state.get_value();
      } 
    }
  }
  return true;
}


} // namespace helios_control

// register controller class 
#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  helios_control::IMUController, controller_interface::ControllerInterface)