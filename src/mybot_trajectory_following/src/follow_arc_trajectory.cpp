#include <memory>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>


// 圆弧轨迹生成，末端 z 轴朝向法向（垂直外法线）
std::vector<geometry_msgs::msg::Pose> generateArcTrajectory(
    const geometry_msgs::msg::Point& center,
    double radius,
    double start_angle,
    double end_angle,
    int num_points)
{
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.reserve(num_points);
    const double angle_step = (end_angle - start_angle) / (num_points - 1);

    for (int i = 0; i < num_points; ++i)
    {
        double theta = start_angle + i * angle_step;

        geometry_msgs::msg::Pose pose;
        pose.position.x = center.x;
        pose.position.y = center.y + radius * cos(theta);
        pose.position.z = center.z + radius * sin(theta);

        tf2::Vector3 x_axis(0.0, -cos(theta), -sin(theta));  // 法向
        tf2::Vector3 z_axis(0.0, -sin(theta), cos(theta)); // 切向（沿着轨迹前进方向）
        tf2::Vector3 y_axis = z_axis.cross(x_axis).normalize();  // y = z × x
        z_axis = x_axis.cross(y_axis).normalize();    

        tf2::Matrix3x3 rotation_matrix(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z());

        tf2::Quaternion quat;
        rotation_matrix.getRotation(quat);
        quat.normalize();
        pose.orientation = tf2::toMsg(quat);

        poses.push_back(pose);
    }

    return poses;
}

// 发布轨迹 Marker 到 RViz
void publishTrajectoryMarker(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::vector<geometry_msgs::msg::Pose>& poses)
{
    auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    rclcpp::sleep_for(std::chrono::milliseconds(500));  // 等待 RViz 订阅

    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = "base_link";
    line_strip.header.stamp = node->get_clock()->now();
    line_strip.ns = "trajectory_path";
    line_strip.id = 0;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.scale.x = 0.01;
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;

    for (const auto& pose : poses) {
        line_strip.points.push_back(pose.position);
    }
    marker_pub->publish(line_strip);

    // 画每个点
    int id = 1;
    for (const auto& pose : poses) {
        visualization_msgs::msg::Marker dot;
        dot.header.frame_id = "base_link";
        dot.header.stamp = node->get_clock()->now();
        dot.ns = "trajectory_points";
        dot.id = id++;
        dot.type = visualization_msgs::msg::Marker::SPHERE;
        dot.action = visualization_msgs::msg::Marker::ADD;
        dot.pose = pose;
        dot.scale.x = 0.02;
        dot.scale.y = 0.02;
        dot.scale.z = 0.02;
        dot.color.g = 1.0;
        dot.color.a = 1.0;
        marker_pub->publish(dot);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("trajectory_control",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto logger = node->get_logger();

    // 初始化 MoveGroup
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
    move_group.setPlanningTime(10.0);
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.3);
    move_group.setPoseReferenceFrame("base_link");

    // 设置轨迹参数
    geometry_msgs::msg::Point center;
    center.x = 1.85;
    center.y = 0.0;
    center.z = -5.2;

    auto forward_waypoints = generateArcTrajectory(
        center,
        6.4,
        68.4 * M_PI / 180.0,
        111.6 * M_PI / 180.0,
        50
    );

    auto reverse_waypoints = generateArcTrajectory(
        center,
        6.4,
        111.6 * M_PI / 180.0,
        68.4 * M_PI / 180.0,
        50
    );

    // 合并前向和反向轨迹
    forward_waypoints.insert(
        forward_waypoints.end(),
        reverse_waypoints.begin(),
        reverse_waypoints.end()
    );

    // 可视化轨迹
    publishTrajectoryMarker(node, forward_waypoints);

    // 笛卡尔路径规划
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(forward_waypoints, 0.01, 0.0, trajectory);

    if (fraction < 0.9) {
        RCLCPP_WARN(logger, "路径规划完成度仅 %.1f%%，可视化轨迹已生成", fraction * 100.0);
    } else {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group.execute(plan);
        RCLCPP_INFO(logger, "轨迹执行完成");
    }

    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
