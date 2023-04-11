#include "nakanosima_demo/demo.hpp"

namespace nakanosima
{
    Demo::Demo(const rclcpp::NodeOptions & options)
        : rclcpp::Node("demo", options)
    {
        signal_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        using namespace std::chrono_literals;
        sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("current_pose",10, std::bind(&nakanosima::Demo::callback, this, _1));
    }

    void Demo::callback(const geometry_msgs::msg::PoseStamped::SharedPtr data)
    {
        RCLCPP_INFO(this->get_logger(), "x : %f", data->pose.position.x);
        RCLCPP_INFO(this->get_logger(), "y : %f", data->pose.position.y);
        geometry_msgs::msg::Twist data = calcurate_velocity(data->pose.position.x, data->pose.position.y, GoalPose pose);
        signal_pub_.publish(data);

    }

    geometry_msgs::msg::Twist Demo::calcurate_velocity(float x, float y, GoalPose pose)
    {
        geometry_msgs::msg::Twist data;
        distance = std::sqrt(std::pow(x - pose.position_x, 2.0)+std::pow(y - pose.position_y, 2.0))
        RCLCPP_INFO(this->get_logger(), "distance : %f", distance);
        data.linear.x = 0.0;
        data.linear.y = 0.0;
        data.linear.z = 0.0;
        data.angular.x = 0.0;
        data.angular.y = 0.0;
        data.angular.z = 0.0;
        return data;
    }
}