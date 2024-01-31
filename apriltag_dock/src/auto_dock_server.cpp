#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "apriltag_dock/action/auto_dock.hpp"
#include "apriltag_dock/find_apriltag_server.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class AutoDockServer : public rclcpp::Node
{
public:
    using AutoDock = apriltag_dock::action::AutoDock;
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
        this->declare_parameter("far_detection_zone_angle_offset", rclcpp::ParameterValue(30.0));
        this->declare_parameter("spin_speed", rclcpp::ParameterValue(0.5));
        this->declare_parameter("spin_duration", rclcpp::ParameterValue(5.0));
        this->declare_parameter("scan_duration", rclcpp::ParameterValue(5.0));
        
        transform_timeout_ = this->get_parameter("transform_timeout").as_double();
        max_far_detection_zone_dist_ = this->get_parameter("max_far_detection_zone_dist").as_double();
        min_far_detection_zone_dist_ = this->get_parameter("min_far_detection_zone_dist").as_double();
        far_detection_zone_angle_offset_ = this->get_parameter("far_detection_zone_angle_offset").as_double();
        spin_speed_ = this->get_parameter("spin_speed").as_double();
        spin_duration_ = this->get_parameter("spin_duration").as_double();
        scan_duration_ = this->get_parameter("scan_duration").as_double();

        // convert angle to radians
        far_detection_zone_angle_offset_ = far_detection_zone_angle_offset_ * M_PI / 180.0;

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        apriltag_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "apriltag_pose", rclcpp::SystemDefaultsQoS(), std::bind(&AutoDockServer::setApriltagApproxPose, this, _1)
        );
        
        auto_dock_action_server_ = std::make_shared<nav2_util::SimpleActionServer<AutoDock>>(
            this,
            "auto_dock",
            std::bind(&AutoDockServer::execute, this)
        );
        auto_dock_action_server_->activate();

        find_apriltag_action_server_ = std::make_shared<FindApriltagServer>(
            this,
            tf_buffer_,
            tf_listener_
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
    std::shared_ptr<FindApriltagServer> find_apriltag_action_server_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr apriltag_pose_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::string map_frame_;
    std::string robot_frame_;
    std::string apriltag_frame_;
    bool apriltag_pose_set_;
    tf2::Vector3 apriltag_position_;
    tf2::Quaternion apriltag_orientation_;
    tf2::Vector3 apriltag_x_axis_;

    // parameters for docking
    double transform_timeout_;
    double max_far_detection_zone_dist_;
    double min_far_detection_zone_dist_;
    double far_detection_zone_angle_offset_;

    // parameters for finding apriltag
    double spin_speed_;
    double spin_duration_;
    double scan_duration_;
    
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

        // first navigate to far detection zone
        RCLCPP_INFO(this->get_logger(), "Traveling to far detection zone");
        sendFeedback("Traveling to far detection zone");
        auto nav_to_pose_goal = nav2_msgs::action::NavigateToPose::Goal();
        nav_to_pose_goal.pose.pose = getDetectionZoneGoal(max_far_detection_zone_dist_, min_far_detection_zone_dist_);
        nav_to_pose_goal.pose.header.stamp = this->now();
        nav_to_pose_goal.pose.header.frame_id = map_frame_;
        auto nav_to_pose_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        
        auto future = nav_to_pose_action_client_->async_send_goal(nav_to_pose_goal, nav_to_pose_goal_options);

        while (!future.valid() || 
                future.get()->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
                future.get()->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING)
        {
            if (!rclcpp::ok() || 
                auto_dock_action_server_->is_cancel_requested() || 
                !auto_dock_action_server_->is_server_active())
            {
                nav_to_pose_action_client_->async_cancel_all_goals();
                auto_dock_action_server_->terminate_current(result);
                return;
            }

            geometry_msgs::msg::PoseStamped robot_pose;
            if (!nav2_util::getCurrentPose(robot_pose, *tf_buffer_, map_frame_, robot_frame_, transform_timeout_, this->now()))
            {
                nav_to_pose_action_client_->async_cancel_all_goals();
                auto_dock_action_server_->terminate_current(result);
                return;
            }
            
            // stop the robot when it is inside the far detection zone
            if (isInsideDetectionZone(robot_pose.pose,
                max_far_detection_zone_dist_,
                min_far_detection_zone_dist_,
                far_detection_zone_angle_offset_))
            {
                RCLCPP_INFO(this->get_logger(), "Inside far docking zone");
                nav_to_pose_action_client_->async_cancel_all_goals();
                break;
            }

            std::this_thread::sleep_for(100ms); // avoid busy waiting
        }

        // spin until apriltag is found
        RCLCPP_INFO(this->get_logger(), "Searching for AprilTag");
        sendFeedback("Searching for AprilTag");
        auto find_apriltag_goal = apriltag_dock::action::FindAprilTag::Goal();
        find_apriltag_goal.spin_speed = spin_speed_;
        find_apriltag_goal.spin_duration = spin_duration_;
        find_apriltag_goal.scan_duration = scan_duration_;

    }

    bool samePosition(const geometry_msgs::msg::Vector3& pos1,
        const geometry_msgs::msg::Vector3& pos2, const geometry_msgs::msg::Vector3& tolerance)
    {
        if (std::abs(pos1.x - pos2.x) < tolerance.x &&
            std::abs(pos1.y - pos2.y) < tolerance.y &&
            std::abs(pos1.z - pos2.z) < tolerance.z
        )
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    geometry_msgs::msg::Transform getAvgTransform(const std::vector<geometry_msgs::msg::Transform>& tfs)
    {
        geometry_msgs::msg::Transform avg_tf;
        int num_tfs = tfs.size();

        for (auto& tf : tfs)
        {
            avg_tf.translation.x += tf.translation.x;
            avg_tf.translation.y += tf.translation.y;
            avg_tf.translation.z += tf.translation.z;
            avg_tf.rotation.x += tf.rotation.x;
            avg_tf.rotation.y += tf.rotation.y;
            avg_tf.rotation.z += tf.rotation.z;
            avg_tf.rotation.w += tf.rotation.w;
        }
        avg_tf.translation.x /= num_tfs;
        avg_tf.translation.y /= num_tfs;
        avg_tf.translation.z /= num_tfs;
        avg_tf.rotation.x /= num_tfs;
        avg_tf.rotation.y /= num_tfs;
        avg_tf.rotation.z /= num_tfs;
        avg_tf.rotation.w /= num_tfs;

        return avg_tf;
    }

    // return the pose to the middle of the detection zone
    geometry_msgs::msg::Pose getDetectionZoneGoal(double max_far_detection_zone_dist, double min_far_detection_zone_dist)
    {
        geometry_msgs::msg::Pose goal_pose;
        double mid_far_detection_zone_dist = (max_far_detection_zone_dist + min_far_detection_zone_dist) / 2.0;

        goal_pose.position.x = apriltag_position_.x() + (apriltag_x_axis_.x() * mid_far_detection_zone_dist);
        goal_pose.position.y = apriltag_position_.y() + (apriltag_x_axis_.y() * mid_far_detection_zone_dist);
        
        // these don't matter
        goal_pose.position.z = 0.0;
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
        double m = apriltag_x_axis_.y() / apriltag_x_axis_.x();
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
    
    void setApriltagApproxPose(const geometry_msgs::msg::PoseStamped& msg)
    {
        RCLCPP_INFO(this->get_logger(), "Set AprilTag approximate pose.");
        apriltag_position_.setX(msg.pose.position.x);
        apriltag_position_.setY(msg.pose.position.y);
        apriltag_position_.setZ(msg.pose.position.z);

        apriltag_orientation_.setX(msg.pose.orientation.x);
        apriltag_orientation_.setY(msg.pose.orientation.y);
        apriltag_orientation_.setZ(msg.pose.orientation.z);
        apriltag_orientation_.setW(msg.pose.orientation.w);

        tf2::Matrix3x3 rotation_matrix(apriltag_orientation_);
        apriltag_x_axis_ = rotation_matrix.getColumn(0);
        apriltag_x_axis_ = apriltag_x_axis_.normalize();

        apriltag_pose_set_ = true;
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