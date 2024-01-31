#include "apriltag_dock/find_apriltag_server.hpp"


FindApriltagServer::FindApriltagServer(rclcpp::Node* node, 
    std::shared_ptr<tf2_ros::Buffer>& tf_buffer, 
    std::shared_ptr<tf2_ros::TransformListener>& tf_listener)
: node_(std::shared_ptr<rclcpp::Node>(node))
, tf_buffer_(tf_buffer)
, tf_listener_(tf_listener)
, cmd_vel_topic_("cmd_vel")
, prev_scan_time_(tf2::TimePointZero)
{
    find_apriltag_action_server_ = std::make_shared<nav2_util::SimpleActionServer<FindAprilTag>>(
        node_,
        "find_apriltag",
        std::bind(&FindApriltagServer::execute, this)
    );
    find_apriltag_action_server_->activate();

    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::SystemDefaultsQoS()
    );
}

FindApriltagServer::~FindApriltagServer()
{
    find_apriltag_action_server_->deactivate();
}

void FindApriltagServer::execute()
{
    auto goal = find_apriltag_action_server_->get_current_goal();
    auto result = std::make_shared<FindAprilTag::Result>();
    
    RCLCPP_INFO(node_->get_logger(), 
        "Spin speed: %f\nSpin duration: %f\nScan duration: %f",
        goal->spin_speed,
        goal->spin_duration,
        goal->scan_duration
    );

    tf2::Duration spin_duration_ = tf2::durationFromSec(goal->spin_duration);
    tf2::Duration scan_duration_ = tf2::durationFromSec(goal->scan_duration);

    while(rclcpp::ok())
    {
        if (find_apriltag_action_server_->is_cancel_requested() ||
            !find_apriltag_action_server_->is_server_active())
        {
            find_apriltag_action_server_->terminate_current(result);
            resetState();
            return;
        }


    }
}

void FindApriltagServer::resetState()
{

}