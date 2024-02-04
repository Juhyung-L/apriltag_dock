#ifndef FIND_APRILTAG_HPP_
#define FIND_APRILTAG_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"

class FindApriltag
{
public:
    FindApriltag(rclcpp::Node* node, 
        std::shared_ptr<tf2_ros::Buffer>& tf_buffer, 
        std::shared_ptr<tf2_ros::TransformListener>& tf_listener,
        std::string map_frame,
        std::string apriltag_frame,
        double transform_timeout);
    void getApriltagPose(geometry_msgs::msg::Pose& pose);
    bool getSucceeded();
    void stopTask();

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string cmd_vel_topic_;
    std::vector<geometry_msgs::msg::Pose> scans_;
    tf2::TimePoint prev_scan_time_;
    std::string map_frame_;
    std::string apriltag_frame_;
    geometry_msgs::msg::Twist spin_cmd_vel_;
    geometry_msgs::msg::Twist stop_cmd_vel_;
    bool succeeded;
    bool continue_task_;
    std::mutex mtx_;
    
    // parameters for finding apriltag
    double spin_speed_;
    double scan_duration_;
    double transform_timeout_;
    double position_tolerance_;

    bool apriltagPositionChanged(const geometry_msgs::msg::Pose& current_apriltag_position);
    geometry_msgs::msg::Pose getAvgPose();
    tf2::Vector3 getAvgPosition();
    tf2::Quaternion getAvgOrientation();
};

#endif