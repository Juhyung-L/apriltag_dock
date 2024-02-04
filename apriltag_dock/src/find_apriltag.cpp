#include <chrono>
#include <algorithm>

#include "nav2_util/robot_utils.hpp"
#include "apriltag_dock/find_apriltag.hpp"

using namespace std::chrono_literals;

FindApriltag::FindApriltag(rclcpp::Node* node, 
    std::shared_ptr<tf2_ros::Buffer>& tf_buffer, 
    std::shared_ptr<tf2_ros::TransformListener>& tf_listener,
    std::string map_frame,
    std::string apriltag_frame,
    double transform_timeout)
: node_(node)
, tf_buffer_(tf_buffer)
, tf_listener_(tf_listener)
, cmd_vel_topic_("cmd_vel")
, map_frame_(map_frame)
, apriltag_frame_(apriltag_frame)
, succeeded{false}
, continue_task_{true}
, transform_timeout_(transform_timeout)
{
    node_->declare_parameter("spin_speed", rclcpp::ParameterValue(0.5));
    node_->declare_parameter("scan_duration", rclcpp::ParameterValue(5.0));
    node_->declare_parameter("position_tolerance", rclcpp::ParameterValue(0.1));

    spin_speed_ = node_->get_parameter("spin_speed").as_double();
    scan_duration_ = node_->get_parameter("scan_duration").as_double();
    position_tolerance_ = node_->get_parameter("position_tolerance").as_double();

    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::SystemDefaultsQoS()
    );

    spin_cmd_vel_.linear.x = 0.0;
    spin_cmd_vel_.linear.y = 0.0;
    spin_cmd_vel_.linear.z = 0.0;
    spin_cmd_vel_.angular.x = 0.0;
    spin_cmd_vel_.angular.y = 0.0;
    spin_cmd_vel_.angular.z = spin_speed_;

    stop_cmd_vel_.linear.x = 0.0;
    stop_cmd_vel_.linear.y = 0.0;
    stop_cmd_vel_.linear.z = 0.0;
    stop_cmd_vel_.angular.x = 0.0;
    stop_cmd_vel_.angular.y = 0.0;
    stop_cmd_vel_.angular.z = 0.0;
}

void FindApriltag::getApriltagPose(geometry_msgs::msg::Pose& pose)
{
    // reset the state
    tf2::Duration scan_duration = tf2::durationFromSec(scan_duration_);
    prev_scan_time_ = tf2::get_now();
    scans_.clear();
    succeeded = false;
    continue_task_ = true;

    while(rclcpp::ok() && continue_task_)
    {
        if (prev_scan_time_ + scan_duration > tf2::get_now())
        {
            // need to scan longer
            geometry_msgs::msg::PoseStamped apriltag_pose;
            if (nav2_util::getCurrentPose(apriltag_pose, *tf_buffer_, map_frame_, apriltag_frame_, transform_timeout_, node_->now()))
            {
                // found apriltag
                // stop the robot
                cmd_vel_pub_->publish(stop_cmd_vel_);

                // check if the pose move significantly compared to previous poses
                if (apriltagPositionChanged(apriltag_pose.pose))
                {
                    // clear the scans and push the new pose
                    scans_.clear();
                    scans_.push_back(apriltag_pose.pose);

                    // reset time
                    prev_scan_time_ = tf2::get_now();
                }
                else
                {
                    scans_.push_back(apriltag_pose.pose);
                }
            }
            else
            {
                // did not find apriltag
                // turn the robot until it detects an AprilTag
                cmd_vel_pub_->publish(spin_cmd_vel_);

                // reset time
                prev_scan_time_ = tf2::get_now();
            }
            std::this_thread::sleep_for(100ms);
            continue;
        }
        // completed scan
        // get the median pose
        pose = getAvgPose();
        succeeded = true;
        return;
    }
    cmd_vel_pub_->publish(stop_cmd_vel_);
}

void FindApriltag::stopTask()
{
    std::lock_guard<std::mutex> lock(mtx_);
    continue_task_ = false;
}

bool FindApriltag::getSucceeded()
{
    return succeeded;
}

bool FindApriltag::apriltagPositionChanged(const geometry_msgs::msg::Pose& current_apriltag_position)
{
    // if first scan, return true
    if (scans_.empty())
    {
        return true;
    }
    
    // get the average position from scans
    tf2::Vector3 avg_position = getAvgPosition();

    // only check if position changed
    if (avg_position.getX() - current_apriltag_position.position.x > position_tolerance_ ||
        avg_position.getY() - current_apriltag_position.position.y > position_tolerance_ ||
        avg_position.getZ() - current_apriltag_position.position.z > position_tolerance_)
    {
        return true;
    }
    return false;
}

geometry_msgs::msg::Pose FindApriltag::getAvgPose()
{
    tf2::Vector3 avg_position = getAvgPosition();
    tf2::Quaternion avg_orientation = getAvgOrientation();

    std::vector<std::pair<double, geometry_msgs::msg::Pose>> scans_with_deviation;
    
    for (int i=0; i<scans_.size(); ++i)
    {
        tf2::Vector3 position(scans_[i].position.x, scans_[i].position.y, scans_[i].position.z);
        tf2::Quaternion orientation(scans_[i].orientation.x, scans_[i].orientation.y, scans_[i].orientation.z, scans_[i].orientation.w);

        double deviation = position.distance(avg_position) + orientation.angleShortestPath(avg_orientation);
        scans_with_deviation.emplace_back(deviation, scans_[i]);
    }
    
    std::sort(scans_with_deviation.begin(), scans_with_deviation.end(),
        [&](const std::pair<double, geometry_msgs::msg::Pose>& pair1, const std::pair<double, geometry_msgs::msg::Pose>& pair2)
        {
            return pair1.first < pair2.first;
        }
    );

    return scans_with_deviation.front().second;
}

tf2::Vector3 FindApriltag::getAvgPosition()
{
    tf2::Vector3 avg_position;
    avg_position.setZero();

    for (auto& scan : scans_)
    {
        avg_position += tf2::Vector3(scan.position.x, scan.position.y, scan.position.z);
    }
    avg_position /= scans_.size();

    return avg_position;
}

tf2::Quaternion FindApriltag::getAvgOrientation()
{
    tf2::Quaternion avg_orientation;
    avg_orientation.setValue(0.0, 0.0, 0.0, 0.0);

    for (auto& scan : scans_)
    {
        avg_orientation += tf2::Quaternion(scan.orientation.x, scan.orientation.y, scan.orientation.z, scan.orientation.w);
    }
    avg_orientation /= scans_.size();

    return avg_orientation;
}