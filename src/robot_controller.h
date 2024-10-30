// #ifndef ROBOT_CONTROLLER_H
// #define ROBOT_CONTROLLER_H

// #include <rclcpp/rclcpp.hpp>
// #include <nav2_msgs/action/navigate_to_pose.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include "location_manager.h"

// /**
//  * @class RobotController
//  * @brief Controls the robot by sending navigation goals using the Nav2 stack.
//  * 
//  * The RobotController class is responsible for sending navigation goals to the robot
//  * based on location data obtained from the LocationManager. It communicates with the
//  * Nav2 stack through an action client, handling goal feedback, status, and results.
//  */
// class RobotController : public rclcpp::Node {
// public:
//     using NavigateToPose = nav2_msgs::action::NavigateToPose;
//     using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

//     /**
//      * @brief Constructor for RobotController.
//      * 
//      * Initializes the RobotController node and connects it to the Nav2 action server.
//      * 
//      * @param location_manager A shared pointer to the LocationManager which provides
//      *        the poses of locations to which the robot can navigate.
//      */
//     RobotController(const std::shared_ptr<LocationManager> &location_manager);

//     /**
//      * @brief Sends the robot to a specific location.
//      * 
//      * This method retrieves the pose corresponding to the provided location name from
//      * the LocationManager and sends it as a navigation goal to the Nav2 action server.
//      * 
//      * @param location_name The name of the location to which the robot should navigate.
//      */
//     void send_robot_to_location(const std::string &location_name);

// private:
//     /// Action client for interacting with the Nav2 stack and sending goals to the robot.
//     rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;

//     /// A reference to the LocationManager that provides poses for the robot's goals.
//     std::shared_ptr<LocationManager> location_manager_;

//     /**
//      * @brief Callback for when the goal is accepted or rejected by the action server.
//      * 
//      * This callback is triggered when the Nav2 action server responds to the goal request.
//      * It checks whether the goal was accepted or rejected.
//      * 
//      * @param future A future object representing the goal handle.
//      */
//     void goal_response_callback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

//     /**
//      * @brief Callback for feedback during goal execution.
//      * 
//      * This callback is triggered as the robot navigates to the goal. It provides
//      * real-time feedback, such as the distance remaining to the goal.
//      * 
//      * @param goal_handle The goal handle associated with the current goal.
//      * @param feedback A shared pointer to the feedback provided by the action server.
//      */
//     void feedback_callback(GoalHandleNavigateToPose::SharedPtr goal_handle, const std::shared_ptr<const NavigateToPose::Feedback> feedback);

//     /**
//      * @brief Callback for when the goal result is received.
//      * 
//      * This callback is triggered once the robot has reached the goal or if the goal
//      * was aborted or canceled. It provides the result status of the goal execution.
//      * 
//      * @param result The result of the goal execution.
//      */
//     void result_callback(const GoalHandleNavigateToPose::WrappedResult &result);
// };

// #endif // ROBOT_CONTROLLER_H

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "location_manager.h"

/**
 * @class RobotController
 * @brief Controls the robot by sending navigation goals using the Nav2 stack.
 * 
 * The RobotController class is responsible for sending navigation goals to the robot
 * based on location data obtained from the LocationManager. It communicates with the
 * Nav2 stack through an action client, handling goal feedback, status, and results.
 */
class RobotController : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    /**
     * @brief Constructor for RobotController.
     * 
     * Initializes the RobotController node and connects it to the Nav2 action server.
     * 
     * @param location_manager A shared pointer to the LocationManager which provides
     *        the poses of locations to which the robot can navigate.
     */
    RobotController(const std::shared_ptr<LocationManager> &location_manager);

    /**
     * @brief Sends the robot to a specific location and waits for the task to complete.
     * 
     * This method retrieves the pose corresponding to the provided location name from
     * the LocationManager and sends it as a navigation goal to the Nav2 action server.
     * The method will block until the navigation task is completed or fails.
     * 
     * @param location_name The name of the location to which the robot should navigate.
     * @return bool True if the navigation to the location was successful, false otherwise.
     */
    bool send_robot_to_location(const std::string &location_name);

private:
    /// Action client for interacting with the Nav2 stack and sending goals to the robot.
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;

    /// A reference to the LocationManager that provides poses for the robot's goals.
    std::shared_ptr<LocationManager> location_manager_;

    /**
     * @brief Callback for when the goal is accepted or rejected by the action server.
     * 
     * This callback is triggered when the Nav2 action server responds to the goal request.
     * It checks whether the goal was accepted or rejected.
     * 
     * @param goal_handle A shared pointer to the goal handle.
     */
    void goal_response_callback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

    /**
     * @brief Callback for feedback during goal execution.
     * 
     * This callback is triggered as the robot navigates to the goal. It provides
     * real-time feedback, such as the distance remaining to the goal.
     * 
     * @param goal_handle The goal handle associated with the current goal.
     * @param feedback A shared pointer to the feedback provided by the action server.
     */
    void feedback_callback(GoalHandleNavigateToPose::SharedPtr goal_handle, const std::shared_ptr<const NavigateToPose::Feedback> feedback);

    /**
     * @brief Callback for when the goal result is received.
     * 
     * This callback is triggered once the robot has reached the goal or if the goal
     * was aborted or canceled. It provides the result status of the goal execution.
     * 
     * @param result The wrapped result of the goal execution.
     */
    void result_callback(const GoalHandleNavigateToPose::WrappedResult &result);
};

#endif // ROBOT_CONTROLLER_H

