#include "nakanosima_demo/demo.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace nakanosima
{
    int state = 0;
    int flg = 0;
    Demo::Demo(const rclcpp::NodeOptions & options)
        : rclcpp::Node("demo", options)
    {
        signal_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        using namespace std::chrono_literals;
        current_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>("current_pose",10, std::bind(&Demo::callback, this, std::placeholders::_1));
        scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>("scan",10,std::bind(&Demo::scan_callback, this, std::placeholders::_1));
        // pose.position_x = 2.0;
        // pose.position_y = 0.0;
        // pose.orientation_z = 0.0;
        // pose.state = 0;
    }

    void Demo::callback(const geometry_msgs::msg::PoseStamped::SharedPtr data)
    {
        RCLCPP_INFO(this->get_logger(), "x : %f", data->pose.position.x);
        RCLCPP_INFO(this->get_logger(), "y : %f", data->pose.position.y);
        float x = data->pose.position.x;
        float y = data->pose.position.y;
        float timeout = 10.0;
        geometry_msgs::msg::Twist signal = calcurate_velocity(x, y, pose, timeout, data);
        RCLCPP_INFO(this->get_logger(), "linear x : %f", signal.linear.x);
        RCLCPP_INFO(this->get_logger(), "linear y : %f", signal.linear.y);
        signal_pub_->publish(signal);
    }

    void Demo::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr data)
    {
        RCLCPP_INFO(this->get_logger(), "range_min : %f", data->range_min);
        RCLCPP_INFO(this->get_logger(), "range_max : %f", data->range_max);
        RCLCPP_INFO(this->get_logger(), "range size : %d", data->ranges.size());
        // int center_index = data->ranges.size() / 2;
        int center_index = 897 / 2;

        RCLCPP_INFO(this->get_logger(), "velodyne center index: %d", center_index);
        RCLCPP_INFO(this->get_logger(), "velodyne center: %f", data->ranges[center_index]);

    }

    geometry_msgs::msg::Twist Demo::calcurate_velocity(float x, float y, GoalPose pose, float timeout, const geometry_msgs::msg::PoseStamped::SharedPtr current_pose)
    {
        geometry_msgs::msg::Twist data;
        float distance;
        distance = std::sqrt(std::pow(x - pose.position_x, 2.0)+std::pow(y - pose.position_y, 2.0));
        RCLCPP_INFO(this->get_logger(), "distance : %f", distance);
        RCLCPP_INFO(this->get_logger(), "current pose position x : %f", current_pose->pose.position.x);
        RCLCPP_INFO(this->get_logger(), "pose.position_x : %f", pose.position_x);
        RCLCPP_INFO(this->get_logger(), "state : %d", state);
        RCLCPP_INFO(this->get_logger(), "flg : %d", flg);
        RCLCPP_INFO(this->get_logger(), "timeout : %f", timeout);
        if(state == 0){
            if(current_pose->pose.position.x >= 2.0 || flg == 1){
                if(current_pose->pose.orientation.z > 0.999 && current_pose->pose.orientation.z < 0.9999){
                    state = 1;
                    flg =0;
                }else{
                    data.linear.x = 0.0;
                    data.linear.y = 0.0;
                    data.linear.z = 0.0;
                    data.angular.x = 0.0;
                    data.angular.y = 0.0;
                    data.angular.z = 0.5;
                    pose.position_x = 0.0;
                    pose.position_y = 0.0;
                    pose.orientation_z = 0.0;
                    flg = 1;
                }    
            } else if(flg == 0){
                data.linear.x = 0.2;
                data.linear.y = 0.0;
                data.linear.z = 0.0;
                data.angular.x = 0.0;
                data.angular.y = 0.0;
                data.angular.z = 0.0;
            } else{
                data.linear.x = 0.0;
                data.linear.y = 0.0;
                data.linear.z = 0.0;
                data.angular.x = 0.0;
                data.angular.y = 0.0;
                data.angular.z = 0.0;
            }
        }else if(state == 1){
            if(current_pose->pose.position.x <= 0.0 || flg == 1){
                if(current_pose->pose.orientation.z > 0.00 && current_pose->pose.orientation.z < 0.01){
                    state = 0;
                    flg = 0;
                }else{
                    data.linear.x = 0.0;
                    data.linear.y = 0.0;
                    data.linear.z = 0.0;
                    data.angular.x = 0.0;
                    data.angular.y = 0.0;
                    data.angular.z = 0.5;
                    pose.position_x = 0.0;
                    pose.position_y = 0.0;
                    pose.orientation_z = 0.0;
                    flg = 1;
                } 
            } else if(flg == 0){
                data.linear.x = 0.2;
                data.linear.y = 0.0;
                data.linear.z = 0.0;
                data.angular.x = 0.0;
                data.angular.y = 0.0;
                data.angular.z = 0.0;
            }else{
            data.linear.x = 0.0;
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