#ifndef AUTO_DOCK_CLIENT_HPP_
#define AUTO_DOCK_CLIENT_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "apriltag_dock_msgs/action/auto_dock.hpp"

using namespace std::placeholders;

class AutoDockClient : public rclcpp::Node
{
public:
    using AutoDock = apriltag_dock_msgs::action::AutoDock;
    using GoalHandleAutoDock = rclcpp_action::ClientGoalHandle<AutoDock>;

    explicit AutoDockClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    void sendGoal();
    void sendCancel();
    bool isGoalExecuting();

private:
    rclcpp_action::Client<AutoDock>::SharedPtr action_client_;
    std::shared_future<GoalHandleAutoDock::SharedPtr> goal_handle_;
    std::chrono::seconds timeout_;
    std::string action_name_;

    void goalResponseCallback(const GoalHandleAutoDock::SharedPtr& goal_handle);
    void feedbackCallback(const GoalHandleAutoDock::SharedPtr,
        const std::shared_ptr<const AutoDock::Feedback> feedback);
    void resultCallback(const GoalHandleAutoDock::WrappedResult& result);
};

#endif