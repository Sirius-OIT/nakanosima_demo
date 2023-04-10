#ifndef DEMO_HPP_
#define DEMO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nakanosima
{
class Demo : public rclcpp::Node
{
public:
    explicit Demo(const rclcpp::NodeOptions & options);
    geometry_msgs::msg::Twist calcurate_velocity(geometry_msgs::msg::PoseStamped current_pose);

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscription_;

}
}