#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
// #include <tf2/LinearMath/Quaternion.h>

// // 将RPY转换为四元数
// auto const target_orientation = []{
//     tf2::Quaternion q;
//     double roll = -1.57;  // 设置期望的roll值
//     double pitch = -1.57; // 设置期望的pitch值
//     double yaw = 0.0;   // 设置期望的yaw值
//     q.setRPY(roll, pitch, yaw);
//     q.normalize(); // 确保四元数归一化
//     geometry_msgs::msg::Quaternion orientation;
//     orientation.x = q.x();
//     orientation.y = q.y();
//     orientation.z = q.z();
//     orientation.w = q.w();
//     return orientation;
// }();


int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
    "follow_trajectoy",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("follow_trajectory");

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "arm");

    auto const target_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.position.x = 1.85;
        msg.position.y = 0;
        msg.position.z = 2.06;
        msg.orientation.x = 0.50163;
        msg.orientation.y = 0.49841;
        msg.orientation.z = -0.50158;
        msg.orientation.w = 0.49831;
        return msg;
    }();

    move_group_interface.setPoseTarget(target_pose);

    auto const [sucess, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();
    
    if(sucess) {
        move_group_interface.execute(plan);
    }else{
        RCLCPP_ERROR(logger, "规划失败！");
    }

    rclcpp::shutdown();
    return 0;
}

