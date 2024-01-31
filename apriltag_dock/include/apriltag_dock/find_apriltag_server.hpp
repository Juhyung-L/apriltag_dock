#ifndef FIND_APRILTAG_HPP_
#define FIND_APRILTAG_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_util/simple_action_server.hpp"
#include "apriltag_dock/action/find_april_tag.hpp"

class FindApriltagServer
{
    using FindAprilTag = apriltag_dock::action::FindAprilTag;
    using GoalHandleFindAprilTag = rclcpp_action::ServerGoalHandle<FindAprilTag>;
public:
    FindApriltagServer(rclcpp::Node* node, 
        std::shared_ptr<tf2_ros::Buffer>& tf_buffer, 
        std::shared_ptr<tf2_ros::TransformListener>& tf_listener);
    ~FindApriltagServer();

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<nav2_util::SimpleActionServer<FindAprilTag>> find_apriltag_action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string cmd_vel_topic_;
    std::string action_name_;
    std::vector<geometry_msgs::msg::Pose> scans_;
    tf2::TimePoint prev_scan_time_;
    
    void execute();
    void resetState();
};

#endif