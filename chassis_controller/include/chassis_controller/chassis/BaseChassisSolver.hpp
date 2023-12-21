// created by liuhan on 2023/12/21
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "math_utilities/MotorPacket.hpp"

#include <string>
#include <vector>
#include <map>

namespace helios_control {

class BaseChassisSolver {
public:
    BaseChassisSolver() = default;

    virtual void solve_geometry(geometry_msgs::msg::TwistStamped& twist_stamped, double yaw_diff) = 0;

    virtual void get_target_values(std::map<std::string, math_utilities::MotorPacket>& cmd_map) = 0;

    virtual ~BaseChassisSolver() = default;
};

} // namespace helios_control