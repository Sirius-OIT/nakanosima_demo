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
    typedef struct
    {
        float position_x;
        float position_y;
        float orientation_z;
        int state;
    }GoalPose;

    explicit Demo(const rclcpp::NodeOptions & options);
    geometry_msgs::msg::Twist calcurate_velocity(float x, float y, GoalPose pose, float timeout);
    void callback(const geometry_msgs::msg::PoseStamped::SharedPtr data);

    virtual ~Demo();

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr signal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscription_;
    GoalPose pose;
};
}

#endif