#include "nakanosima_demo/demo.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace nakanosima
{
    Demo::Demo(const rclcpp::NodeOptions & options)
        : rclcpp::Node("demo", options)
    {
        signal_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        using namespace std::chrono_literals;
        current_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>("current_pose",10, std::bind(&Demo::callback, this, std::placeholders::_1));
        scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>("scan",10,std::bind(&Demo::scan_callback, this, std::placeholders::_1));
        pose.position_x = 2.0;
        pose.position_y = 0.0;
        pose.orientation_z = 0.0;
        pose.state = 0;
    }

    void Demo::callback(const geometry_msgs::msg::PoseStamped::SharedPtr data)
    {
        RCLCPP_INFO(this->get_logger(), "x : %f", data->pose.position.x);
        RCLCPP_INFO(this->get_logger(), "y : %f", data->pose.position.y);
        float x = data->pose.position.x;
        float y = data->pose.position.y;
        float timeout = 10.0;
        geometry_msgs::msg::Twist signal = calcurate_velocity(x, y, pose, timeout);
        RCLCPP_INFO(this->get_logger(), "linear x : %f", signal.linear.x);
        RCLCPP_INFO(this->get_logger(), "linear y : %f", signal.linear.y);
        signal_pub_->publish(signal);
    }

    void Demo::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr data)
    {
        RCLCPP_INFO(this->get_logger(), "range_min : %f", data->range_min);
        RCLCPP_INFO(this->get_logger(), "range_max : %f", data->range_max);
    }

    geometry_msgs::msg::Twist Demo::calcurate_velocity(float x, float y, GoalPose pose, float timeout)
    {
        geometry_msgs::msg::Twist data;
        float distance;
        distance = std::sqrt(std::pow(x - pose.position_x, 2.0)+std::pow(y - pose.position_y, 2.0));
        RCLCPP_INFO(this->get_logger(), "distance : %f", distance);
        RCLCPP_INFO(this->get_logger(), "pose.position_x : %f", pose.position_x);
        RCLCPP_INFO(this->get_logger(), "state : %d", pose.state);
        if(pose.state == 0){
            if(distance <= 0.0){
                data.linear.x = 0.0;
                data.linear.y = 0.0;
                data.linear.z = 0.0;
                data.angular.x = 0.0;
                data.angular.y = 0.0;
                data.angular.z = 3.14;
                pose.position_x = 0.0;
                pose.position_y = 0.0;
                pose.orientation_z = 0.0;
                pose.state = 1;
            } else {
                data.linear.x = 0.2;
                data.linear.y = 0.0;
                data.linear.z = 0.0;
                data.angular.x = 0.0;
                data.angular.y = 0.0;
                data.angular.z = 0.0;
            }
        }else if(pose.state == 1){
            if(distance <= 0.0){
                data.linear.x = 0.0;
                data.linear.y = 0.0;
                data.linear.z = 0.0;
                data.angular.x = 0.0;
                data.angular.y = 0.0;
                data.angular.z = 3.14;
                pose.position_x = 2.0;
                pose.position_y = 0.0;
                pose.orientation_z = 0.0;
                pose.state = 0;
            } else {
                data.linear.x = 0.2;
                data.linear.y = 0.0;
                data.linear.z = 0.0;
                data.angular.x = 0.0;
                data.angular.y = 0.0;
                data.angular.z = 0.0;
            }
        }else{
            data.linear.x = 0.0;
            data.linear.y = 0.0;
            data.linear.z = 0.0;
            data.angular.x = 0.0;
            data.angular.y = 0.0;
            data.angular.z = 0.0;
        }
        RCLCPP_INFO(this->get_logger(),"data.linear.x5 : %f", data.linear.x);
        return data; 
    }

    Demo::~Demo(){}
}


RCLCPP_COMPONENTS_REGISTER_NODE(nakanosima::Demo)