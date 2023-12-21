// created by liuhan on 2023/9/21
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
#pragma once

#include "BaseChassisSolver.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>

namespace helios_control {

class OmnidirectionalSolver : public BaseChassisSolver {
public:
    OmnidirectionalSolver() = default;

    /**
     * @brief solve the target values of four motors
     * 
     * @param twist_stamped navg's comman
     * @param yaw_diff yaw diff of gimbal and chassis
     */
    void solve_geometry(geometry_msgs::msg::TwistStamped &twist_stamped, double yaw_diff) override;

    void get_target_values(std::map<std::string, math_utilities::MotorPacket>& cmd_map) override;

    ~OmnidirectionalSolver() = default;
private:
    double front_left_v_;
    double front_right_v_;
    double back_left_v_;
    double back_right_v_;
};


} // namespace helios_control