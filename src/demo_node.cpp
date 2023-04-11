#include "rclcpp/rclcpp.hpp"
#include "nakanosima_demo/demo.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    const auto demo = std::make_shared<nakanosima::Demo>(rclcpp::NodeOptions());

    exec.add_node(demo);
    exec.spin();
    rclcpp::shutdown();
}