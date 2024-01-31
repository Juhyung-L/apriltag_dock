#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "apriltag_dock/action/auto_dock.hpp"
#include "apriltag_dock/auto_dock_client.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

AutoDockClient::AutoDockClient(const rclcpp::NodeOptions& options)
: Node("auto_dock_action_client", options),
timeout_(5s),
action_name_("auto_dock")
{
    action_client_ = rclcpp_action::create_client<AutoDock>(
        this,
        action_name_
    );
}

void AutoDockClient::sendGoal()
{
    if (!action_client_->wait_for_action_server(timeout_))
    {
        throw std::runtime_error(action_name_ + " server not yet ready");
        return;
    }

    auto goal = AutoDock::Goal();
    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto goal_options = rclcpp_action::Client<AutoDock>::SendGoalOptions();
    goal_options.goal_response_callback = 
        std::bind(&AutoDockClient::goalResponseCallback, this, _1);
    
    goal_options.feedback_callback =
        std::bind(&AutoDockClient::feedbackCallback, this, _1, _2);

    goal_options.result_callback =
        std::bind(&AutoDockClient::resultCallback, this, _1);

    goal_handle_ = action_client_->async_send_goal(goal, goal_options);
    while (!isGoalExecuting() && rclcpp::ok()) {std::this_thread::sleep_for(100ms);} // wait until action is in executing state
}

void AutoDockClient::sendCancel()
{
    if (isGoalExecuting())
    {
        RCLCPP_INFO(this->get_logger(), "Sending cancel goal");
        // action_client_->async_cancel_goal(goal_handle_.get());
        action_client_->async_cancel_all_goals();
        while (isGoalExecuting() && rclcpp::ok()) {std::this_thread::sleep_for(100ms);}  // wait until goal is not in executing state
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No goal to cancel");
    }
}

bool AutoDockClient::isGoalExecuting()
{
    if (!goal_handle_.valid())
    {
        return false;
    }

    if (goal_handle_.get()->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
        goal_handle_.get()->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void AutoDockClient::goalResponseCallback(const GoalHandleAutoDock::SharedPtr& goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } 
    else 
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void AutoDockClient::feedbackCallback(const GoalHandleAutoDock::SharedPtr,
    const std::shared_ptr<const AutoDock::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), 
        "Current docking state: %s",
        "Distance to apriltag: %f",
        feedback->state,
        feedback->distance_to_apriltag);
}

void AutoDockClient::resultCallback(const GoalHandleAutoDock::WrappedResult& result)
{
    switch (result.code) 
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }
}