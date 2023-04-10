#include "nakanosima_demp/demo.hpp"

namespace demo
{
    Demo::Demo(const rclcpp::NodeOptions & options)
        : rclcpp::Node("demo", options)

    Demo::calcurate_velocity(geometry_msgs::msg::PoseStamped current_pose)
    {
        positioin_x = current_pose.pose.position.x;
        position_y = current_pose.pose.position.y;
        orientation_z = current_pose.pose.orientation.z;
    }
}