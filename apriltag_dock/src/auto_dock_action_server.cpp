#include <thread>
#include <mutex>

#include "apriltag_dock/action/auto_dock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

using namespace std::placeholders;

class AutoDockActionServer : public rclcpp::Node
{
public:
    using AutoDock = apriltag_dock::action::AutoDock;
    using GoalHandleAutoDock = rclcpp_action::ServerGoalHandle<AutoDock>;

    explicit AutoDockActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("auto_dock_action_server", options)
    {
        action_server_ = rclcpp_action::create_server<AutoDock>(
            this,
            "auto_dock",
            std::bind(&AutoDockActionServer::handle_goal, this, _1, _2),
            std::bind(&AutoDockActionServer::handle_cancel, this, _1),
            std::bind(&AutoDockActionServer::handle_accepted, this, _1)
        );

        detections_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "apriltag/detections", rclcpp::SystemDefaultsQoS(), std::bind(&AutoDockActionServer::on_detection, _1)
        );

        // set them equal so that 
        dets_.header.stamp = this->now();
        last_dets_time = dets_.header.stamp;
    }

private:
    rclcpp_action::Server<AutoDock>::SharedPtr action_server_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
    apriltag_msgs::msg::AprilTagDetectionArray dets_;
    std::mutex dets_mtx_;
    builtin_interfaces::msg::Time last_dets_time;
    rclcpp::Time spin_start_time;
    
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const AutoDock::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleAutoDock> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleAutoDock> goal_handle)
    {
        // spin a new thread to avoid blocking this function
        std::thread(std::bind(&AutoDockActionServer::execute, this), goal_handle).detach();
    }

    void execute(const std::shared_ptr<GoalHandleAutoDock> goal_handle)
    {
        auto feedback = std::make_shared<AutoDock::Feedback>();
        auto result = std::make_shared<AutoDock::Result>();

        // main event loop
        while (goal_handle->is_executing())
        {
            if (goal_handle->is_canceling())
            {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Auto docking canceled");
                continue;
            }

            
            std::unique_lock<std::mutex> lock(dets_mtx_);
            lock.lock();
            if (last_dets_time == dets_.header.stamp)
            {
                // no apriltag detected 
                // spin for 5 seconds until a tag is detected
                // send action failed after 5 seconds

                continue;
            }
            
        }
    }

    void on_detection(const apriltag_msgs::msg::AprilTagDetectionArray& msg)
    {
        // constantly update dets_
        std::lock_guard<std::mutex> lock(dets_mtx_);
        dets_ = msg;
    }
    
};
