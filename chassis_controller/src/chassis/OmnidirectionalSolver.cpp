// created by liuhan on 2023/9/21
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#include "chassis/OmnidirectionalSolver.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <math.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace helios_control {

void OmnidirectionalSolver::solve_geometry(geometry_msgs::msg::TwistStamped &twist_stamped, double yaw_diff) {
    // 底盘坐标系如下：     
    /*               battery            
     *                  |+x              
     *              //  |  \\              
     *            +y---------              
     *              \\  |  //             
     *                                     
     * 假设轮子自身顺时针转动为-，逆时针转动为+
     */
    // linear velocity
    double v_lx = twist_stamped.twist.linear.x;
    double v_ly = twist_stamped.twist.linear.y;
    // angular velocity
    // z指向方向视角，顺时针为正，逆时针为负
    double v_z = twist_stamped.twist.angular.z;
    Eigen::Vector2d v_l(v_lx, v_ly);
    Eigen::Matrix2d R;
    // yaw_diff = M_PI * 3 / 4;
    yaw_diff = -yaw_diff;
    R << std::cos(yaw_diff), -std::sin(yaw_diff), 
         std::sin(yaw_diff), std::cos(yaw_diff);
    Eigen::Vector2d v_r = R * v_l; // 转换到实际底盘上
    R << std::cos(M_PI_4), -std::sin(M_PI_4), 
         std::sin(M_PI_4), std::cos(M_PI_4);
    v_r = R * v_r; // 转换到轮子坐标系上
    front_left_v_ = v_r(0) + v_z;
    front_right_v_ = -v_r(1) + v_z;
    back_left_v_ = v_r(1) + v_z;
    back_right_v_ = -v_r(0) + v_z;
}

void OmnidirectionalSolver::get_target_values(
    std::map<std::string, math_utilities::MotorPacket>& cmd_map) {
    for (auto& motor : cmd_map) {
        if (motor.first == "front_left") {
            motor.second.motor_mode_ = 0x01;
            motor.second.value_ = front_left_v_;
        } else if (motor.first == "front_right") {
            motor.second.motor_mode_ = 0x01;
            motor.second.value_ = front_right_v_;
        } else if (motor.first == "back_left") {
            motor.second.motor_mode_ = 0x01;
            motor.second.value_ = back_left_v_;
        } else if (motor.first == "back_right") {
            motor.second.motor_mode_ = 0x01;
            motor.second.value_ = back_right_v_;
        }
    }
}

} // namespace helios_control