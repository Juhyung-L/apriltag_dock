#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "apriltag_ros/apriltag_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AprilTagNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}