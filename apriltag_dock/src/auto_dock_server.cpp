#include <string>
#include <memory>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "apriltag_dock_msgs/action/auto_dock.hpp"
#include "apriltag_dock/find_apriltag.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class AutoDockServer : public rclcpp::Node
{
public:
    using AutoDock = apriltag_dock_msgs::action::AutoDock;
    using GoalHandleAutoDock = rclcpp_action::ServerGoalHandle<AutoDock>;

    explicit AutoDockServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("auto_dock_action_server", options),
    map_frame_("map"),
    robot_frame_("base_footprint"),
    apriltag_frame_("apriltag"),
    apriltag_pose_set_(false)
    {
        // parameters
        this->declare_parameter("transform_timeout", rclcpp::ParameterValue(0.3));
        this->declare_parameter("max_far_detection_zone_dist", rclcpp::ParameterValue(3.0));
        this->declare_parameter("min_far_detection_zone_dist", rclcpp::ParameterValue(2.0));
        this->declare_parameter("max_near_detection_zone_dist", rclcpp::ParameterValue(1.0));
        this->declare_parameter("min_near_detection_zone_dist", rclcpp::ParameterValue(0.0));
        this->declare_parameter("detection_zone_angle_offset", rclcpp::ParameterValue(30.0));
        
        transform_timeout_ = this->get_parameter("transform_timeout").as_double();
        max_far_detection_zone_dist_ = this->get_parameter("max_far_detection_zone_dist").as_double();
        min_far_detection_zone_dist_ = this->get_parameter("min_far_detection_zone_dist").as_double();
        max_near_detection_zone_dist_ = this->get_parameter("max_near_detection_zone_dist").as_double();
        min_near_detection_zone_dist_ = this->get_parameter("min_near_detection_zone_dist").as_double();
        detection_zone_angle_offset_ = this->get_parameter("detection_zone_angle_offset").as_double();

        // convert angle to radians
        detection_zone_angle_offset_ = detection_zone_angle_offset_ * M_PI / 180.0;

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        apriltag_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "apriltag_approx_pose", rclcpp::SystemDefaultsQoS(), 
            [&](const geometry_msgs::msg::PoseStamped& msg)
            {
                RCLCPP_INFO(this->get_logger(), "Set AprilTag approximate pose.");
                setApriltagPose(msg.pose, 'x');
                apriltag_pose_set_ = true;
            }
        );
        
        auto_dock_action_server_ = std::make_shared<nav2_util::SimpleActionServer<AutoDock>>(
            this,
            "auto_dock",
            std::bind(&AutoDockServer::execute, this)
        );
        auto_dock_action_server_->activate();

        find_apriltag_ = std::make_shared<FindApriltag>(
            this,
            tf_buffer_,
            tf_listener_,
            map_frame_,
            apriltag_frame_,
            transform_timeout_
        );

        nav_to_pose_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose"
        );
    }

    ~AutoDockServer()
    {
        auto_dock_action_server_->deactivate();
    }

private:
    std::shared_ptr<nav2_util::SimpleActionServer<AutoDock>> auto_dock_action_server_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_action_client_;
    std::shared_ptr<FindApriltag> find_apriltag_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr apriltag_pose_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::string map_frame_;
    std::string robot_frame_;
    std::string apriltag_frame_;
    bool apriltag_pose_set_;
    tf2::Vector3 apriltag_position_;
    tf2::Quaternion apriltag_orientation_;
    tf2::Vector3 apriltag_principle_axis_;

    // parameters for docking
    double transform_timeout_;
    double max_far_detection_zone_dist_;
    double min_far_detection_zone_dist_;
    double max_near_detection_zone_dist_;
    double min_near_detection_zone_dist_;
    double detection_zone_angle_offset_;
    
    void execute()
    {
        auto result = std::make_shared<AutoDock::Result>();

        // abort if apriltag's approximate position is not set
        if (!apriltag_pose_set_)
        {
            RCLCPP_ERROR(this->get_logger(), "Approximate position of the AprilTag is not set");
            auto_dock_action_server_->terminate_current(result);
            return;
        }

        // navigate to far detection zone
        RCLCPP_INFO(this->get_logger(), "Traveling to far detection zone");
        sendFeedback("Traveling to far detection zone");
        auto nav_to_pose_goal = nav2_msgs::action::NavigateToPose::Goal();
        nav_to_pose_goal.pose.pose = getDetectionZoneGoal(max_far_detection_zone_dist_, min_far_detection_zone_dist_);
        nav_to_pose_goal.pose.header.stamp = this->now();
        nav_to_pose_goal.pose.header.frame_id = map_frame_;
        auto nav_to_pose_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        
        auto nav_to_pose_future = nav_to_pose_action_client_->async_send_goal(nav_to_pose_goal, nav_to_pose_goal_options);

        while (true)
        {
            geometry_msgs::msg::PoseStamped robot_pose;
            if (!rclcpp::ok() ||
                auto_dock_action_server_->is_cancel_requested() ||
                !nav2_util::getCurrentPose(robot_pose, *tf_buffer_, map_frame_, robot_frame_, transform_timeout_, this->now()))
            {
                nav_to_pose_action_client_->async_cancel_all_goals();
                auto_dock_action_server_->terminate_current(result);
                return;
            }
            
            // stop the robot when it is inside the far detection zone
            if (isInsideDetectionZone(robot_pose.pose,
                max_far_detection_zone_dist_,
                min_far_detection_zone_dist_,
                detection_zone_angle_offset_))
            {
                RCLCPP_INFO(this->get_logger(), "Inside far docking zone");
                nav_to_pose_action_client_->async_cancel_all_goals();
                break;
            }
            std::this_thread::sleep_for(100ms); // avoid busy looping
        }

        // find apriltag while inside far detection zone
        RCLCPP_INFO(this->get_logger(), "Searching for AprilTag");
        sendFeedback("Searching for AprilTag");
        geometry_msgs::msg::Pose pose;
        auto find_apriltag_future = std::async(std::launch::async, [&]() {find_apriltag_->getApriltagPose(pose);});
        while (true)
        {
            if (!rclcpp::ok() || auto_dock_action_server_->is_cancel_requested())
            {
                auto_dock_action_server_->terminate_current(result);
                find_apriltag_->stopTask();
                return;
            }

            if (find_apriltag_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
            {
                if (find_apriltag_->getSucceeded())
                {
                    setApriltagPose(pose, 'z');
                    break;
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Searching for AprilTag failed");
                    auto_dock_action_server_->terminate_current(result);
                    return;
                }
            }
            std::this_thread::sleep_for(100ms);
        }

        // navigate to near detection zone
        RCLCPP_INFO(this->get_logger(), "Traveling to near detection zone");
        sendFeedback("Traveling to near detection zone");
        nav_to_pose_goal.pose.pose = getDetectionZoneGoal(max_near_detection_zone_dist_, min_near_detection_zone_dist_);
        nav_to_pose_goal.pose.header.stamp = this->now();
        
        nav_to_pose_future = nav_to_pose_action_client_->async_send_goal(nav_to_pose_goal, nav_to_pose_goal_options);

        while (true)
        {
            geometry_msgs::msg::PoseStamped robot_pose;
            if (!rclcpp::ok() ||
                auto_dock_action_server_->is_cancel_requested() ||
                !nav2_util::getCurrentPose(robot_pose, *tf_buffer_, map_frame_, robot_frame_, transform_timeout_, this->now()))
            {
                nav_to_pose_action_client_->async_cancel_all_goals();
                auto_dock_action_server_->terminate_current(result);
                return;
            }
            
            // stop the robot when it is inside the far detection zone
            if (isInsideDetectionZone(robot_pose.pose,
                max_near_detection_zone_dist_,
                min_near_detection_zone_dist_,
                detection_zone_angle_offset_))
            {
                RCLCPP_INFO(this->get_logger(), "Inside near docking zone");
                nav_to_pose_action_client_->async_cancel_all_goals();
                break;
            }
            std::this_thread::sleep_for(100ms); // avoid busy looping
        }

        // find apriltag while inside near detection zone
        RCLCPP_INFO(this->get_logger(), "Searching for AprilTag");
        sendFeedback("Searching for AprilTag");
        find_apriltag_future = std::async(std::launch::async, [&]() {find_apriltag_->getApriltagPose(pose);});
        while (true)
        {
            if (!rclcpp::ok() || auto_dock_action_server_->is_cancel_requested())
            {
                auto_dock_action_server_->terminate_current(result);
                find_apriltag_->stopTask();
                return;
            }

            if (find_apriltag_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
            {
                if (find_apriltag_->getSucceeded())
                {
                    setApriltagPose(pose, 'z');
                    break;
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Searching for AprilTag failed");
                    auto_dock_action_server_->terminate_current(result);
                    return;
                }
            }
            std::this_thread::sleep_for(100ms);
        }

        RCLCPP_INFO(this->get_logger(), "Docking");
        sendFeedback("Docking");
        nav_to_pose_goal.pose.pose = getDetectionZoneGoal(max_near_detection_zone_dist_, min_near_detection_zone_dist_);
        nav_to_pose_goal.pose.header.stamp = this->now();
        
        nav_to_pose_future = nav_to_pose_action_client_->async_send_goal(nav_to_pose_goal, nav_to_pose_goal_options);
        // travel to center of near detection zone and face the apriltag
        while (true)
        {
            if (!rclcpp::ok() || auto_dock_action_server_->is_cancel_requested())
            {
                auto_dock_action_server_->terminate_current(result);
                nav_to_pose_action_client_->async_cancel_all_goals();
                return;
            }

            if (nav_to_pose_future.valid() &&
                nav_to_pose_future.get()->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED)
            {
                break;
            }
        }
        auto_dock_action_server_->succeeded_current(result);
        RCLCPP_INFO(this->get_logger(), "Docking complete!");
    }

    // return the pose to the middle of the detection zone
    geometry_msgs::msg::Pose getDetectionZoneGoal(double max_detection_zone_dist, double min_detection_zone_dist)
    {
        geometry_msgs::msg::Pose goal_pose;
        double mid_far_detection_zone_dist = (max_detection_zone_dist + min_detection_zone_dist) / 2.0;

        goal_pose.position.x = apriltag_position_.x() + (apriltag_principle_axis_.x() * mid_far_detection_zone_dist);
        goal_pose.position.y = apriltag_position_.y() + (apriltag_principle_axis_.y() * mid_far_detection_zone_dist);
        goal_pose.position.z = 0.0; // cuz 2D

        // get orientation that faces the apriltag
        goal_pose.orientation.x = 0.0;
        goal_pose.orientation.y = 0.0;
        goal_pose.orientation.z = 0.0;
        goal_pose.orientation.w = 1.0;

        return goal_pose;
    }
    
    // return true if the robot_pose is inside the detection zone
    bool isInsideDetectionZone(const geometry_msgs::msg::Pose& robot_pose,
        double max_detection_zone_dist,
        double min_detection_zone_dist,
        double max_angle_offset)
    {
        // check the distance
        tf2::Vector3 pose(robot_pose.position.x, robot_pose.position.y, robot_pose.position.z);
        double distance = pose.distance(apriltag_position_);
        if (distance < min_detection_zone_dist || distance > max_detection_zone_dist)
        {
            return false;
        }

        // check angle
        double m = apriltag_principle_axis_.y() / apriltag_principle_axis_.x();
        double mp = -1.0 / m;
        double x = (m*apriltag_position_.x() - mp*robot_pose.position.x - apriltag_position_.y() + robot_pose.position.y) / (m - mp);
        double y = m*x - m*apriltag_position_.x() + apriltag_position_.y();
        double adj = apriltag_position_.distance(tf2::Vector3(x, y, 0.0));
        double opp = tf2::Vector3(x, y, 0.0).distance(tf2::Vector3(robot_pose.position.x, robot_pose.position.y, 0.0));
        double angle = std::abs(std::atan2(opp, adj));
        if (angle > max_angle_offset)
        {
            return false;
        }

        return true;
    }

    void setApriltagPose(const geometry_msgs::msg::Pose& pose, const char principle_axis)
    {
        apriltag_position_.setX(pose.position.x);
        apriltag_position_.setY(pose.position.y);
        apriltag_position_.setZ(pose.position.z);

        apriltag_orientation_.setX(pose.orientation.x);
        apriltag_orientation_.setY(pose.orientation.y);
        apriltag_orientation_.setZ(pose.orientation.z);
        apriltag_orientation_.setW(pose.orientation.w);

        tf2::Matrix3x3 rotation_matrix(apriltag_orientation_);
        if (principle_axis == 'x')
        {
            apriltag_principle_axis_ = rotation_matrix.getColumn(0);
        }
        else if (principle_axis == 'z')
        {
            apriltag_principle_axis_ = rotation_matrix.getColumn(2);
        }
        
        apriltag_principle_axis_ = apriltag_principle_axis_.normalize();
    }

    void sendFeedback(const std::string& current_state)
    {
        auto feedback = std::make_shared<AutoDock::Feedback>();

        feedback->state = current_state;
        
        geometry_msgs::msg::PoseStamped robot_pose;
        nav2_util::getCurrentPose(robot_pose, *tf_buffer_, map_frame_, robot_frame_, transform_timeout_, this->now());
        tf2::Vector3 pose(robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z);
        feedback->distance_to_apriltag = apriltag_position_.distance(pose);
        
        auto_dock_action_server_->publish_feedback(feedback);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoDockServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}