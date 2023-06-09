#ifndef DEMO_HPP_
#define DEMO_HPP_

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nakanosima
{
class Demo : public rclcpp::Node
{
public:
    typedef struct
    {
        float position_x=2.0;
        float position_y=0.0;
        float orientation_z=0.0;
        int state=0;
    }GoalPose;

    explicit Demo(const rclcpp::NodeOptions & options);
    geometry_msgs::msg::Twist calcurate_velocity(float x, float y, GoalPose pose, float timeout, const geometry_msgs::msg::PoseStamped::SharedPtr current_pose);
    void callback(const geometry_msgs::msg::PoseStamped::SharedPtr data);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr data);

    virtual ~Demo();

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr signal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    GoalPose pose;
};
}

#endif