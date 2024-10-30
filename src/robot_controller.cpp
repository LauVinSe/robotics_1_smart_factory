#include "robot_controller.h"

/**
 * Constructor for the RobotController class.
 * Initializes the ROS node and the action client for "navigate_to_pose".
 * Waits for the action server to be available.
 */
RobotController::RobotController(const std::shared_ptr<LocationManager> &location_manager)
    : Node("robot_controller"), location_manager_(location_manager) {
    // Initialize action client for "navigate_to_pose" action
    nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Wait for the action server to be available
    if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    }
}

/**
 * Sends the robot to the specified location by retrieving the pose from LocationManager
 * and sending it as a goal to the Nav2 action server.
 * This method now waits for the task to complete before returning.
 */
// Function to send the robot to a location and wait for the result
bool RobotController::send_robot_to_location(const std::string &location_name) {
    auto pose = location_manager_->get_location(location_name);
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose = pose;

    RCLCPP_INFO(this->get_logger(), "Sending robot to location: %s", location_name.c_str());

    auto goal_handle_future = nav_to_pose_client_->async_send_goal(goal_msg);
    // Wait for the server to accept the goal
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Send goal call failed : %s", location_name.c_str());
        return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        return false;
    }

    // Wait for the result of the goal
    auto result_future = nav_to_pose_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Getting result failed : %s", location_name.c_str());
        return false;
    }

    auto result = result_future.get();
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully: %s", location_name.c_str());
            return true;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted: %s", location_name.c_str());
            return false;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal was canceled: %s", location_name.c_str());
            return false;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code for the goal: %s", location_name.c_str());
            return false;
    }
}


/**
 * Callback function that is called when the action server responds to the goal request.
 * It checks whether the goal has been accepted or rejected.
 */
void RobotController::goal_response_callback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server, waiting for result...");
    }
}

/**
 * Callback function that provides feedback during goal execution.
 * This is called by the action server as the robot moves towards the goal.
 */
void RobotController::feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Received feedback: %.2f meters remaining to goal", feedback->distance_remaining);
}

/**
 * Callback function that is called when the goal has completed execution.
 * It handles success, failure, or cancellation of the goal.
 */
void RobotController::result_callback(const GoalHandleNavigateToPose::WrappedResult &result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted by the action server");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal was canceled by the client");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
    }
}

