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
 */
void RobotController::send_robot_to_location(const std::string &location_name) {
    // Get the pose for the specified location
    auto pose = location_manager_->get_location(location_name);

    // Create a goal message and set the pose
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";    // or appropriate frame
    goal_msg.pose.header.stamp = this->now(); // Set the timestamp
    goal_msg.pose.pose = pose;

    RCLCPP_INFO(this->get_logger(), "Pose to send: x=%.2f, y=%.2f", pose.position.x, pose.position.y);


    RCLCPP_INFO(this->get_logger(), "Sending goal to location: %s", location_name.c_str());

    // Set up goal options and send the goal
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&RobotController::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&RobotController::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&RobotController::result_callback, this, std::placeholders::_1);

    // Asynchronously send the goal to the action server
    nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
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
