#include <memory>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 生成圆弧轨迹并附加切线方向姿态（Z轴沿切线）
std::vector<geometry_msgs::msg::Pose> generateArcTrajectory()
{
    std::vector<geometry_msgs::msg::Pose> poses;
    const int num_points = 50;

    double theta_start = 68.4 * M_PI / 180.0;
    double theta_end   = 111.6 * M_PI / 180.0;
    double dtheta = (theta_end - theta_start) / (num_points - 1);
    double radius = 6.4;
    double x_val = 1.8;

    for (int i = 0; i < num_points; ++i) {
        double theta = theta_start + i * dtheta;
        double theta_next = theta + dtheta;

        // 当前点位置
        double y = radius * cos(theta);
        double z = radius * sin(theta) - 5;

        geometry_msgs::msg::Pose pose;
        pose.position.x = x_val;
        pose.position.y = y;
        pose.position.z = z;

        // 计算切线方向作为Z轴
        double y_next = radius * cos(theta_next);
        double z_next = radius * sin(theta_next) - 5;
        tf2::Vector3 tangent(0, y_next - y, z_next - z);
        tf2::Vector3 z_axis = tangent.normalized();

        tf2::Vector3 up(0, 0, 1); // 世界坐标系Z轴
        tf2::Vector3 x_axis = up.cross(z_axis).normalized();
        tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();

        tf2::Matrix3x3 rot_matrix;
        rot_matrix[0] = x_axis;
        rot_matrix[1] = y_axis;
        rot_matrix[2] = z_axis;

        tf2::Quaternion quat;
        rot_matrix.getRotation(quat);
        pose.orientation = tf2::toMsg(quat);

        poses.push_back(pose);
    }

    return poses;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("trajectory_control");

    auto logger = node->get_logger();

    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "arm");

    move_group.setPlanningTime(15.0);
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.3);
    move_group.setPoseReferenceFrame("base_link");

    // 生成轨迹点
    auto waypoints = generateArcTrajectory();

    // 笛卡尔路径规划
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(
        waypoints,
        0.01,   // 轨迹步长
        0.0,    // 跳跃阈值
        trajectory
    );

    if (fraction < 0.9) {
        RCLCPP_ERROR(logger, "路径规划失败 (完成度: %.1f%%)", fraction * 100.0);
        return 1;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    RCLCPP_INFO(logger, "开始执行轨迹...");
    auto result = move_group.execute(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(logger, "轨迹执行成功！");
    } else {
        RCLCPP_ERROR(logger, "轨迹执行失败！");
    }

    rclcpp::shutdown();
    return 0;
}