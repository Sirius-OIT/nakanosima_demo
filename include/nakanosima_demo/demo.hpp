#ifndef DEMO_HPP_
#define DEMO_HPP_

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nakanosima
{
class Demo : public rclcpp::Node
{
public:
    explicit Demo(const rclcpp::NodeOptions & options);
    geometry_msgs::msg::Twist calcurate_velocity(float x, float y, GoalPose pose);
    void callback();

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr signal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscription_;

    typedef struct
    {
        position_x;
        position_y;
        orientation_z;
    }GoalPose;
}
}