#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include "alley.h"
#include "robot_controller.h"
#include "warehouse_manager.h"
#include "location_manager.h"

using namespace std::chrono_literals;

class TurtleBotMoveToGoal : public rclcpp::Node
{
public:
    TurtleBotMoveToGoal(const geometry_msgs::msg::Pose &goal_pose_1, const geometry_msgs::msg::Pose &goal_pose_2) 
        : Node("turtlebot_move_to_goal"), goal_pose_1_(goal_pose_1), goal_pose_2_(goal_pose_2), current_goal_(goal_pose_1), move_backward_(false), is_aligned_(false), reached_goal_1_(false), reached_goal_2_(false)
    {
        goal_x_ = current_goal_.position.x;
        goal_y_ = current_goal_.position.y;

        // Create a publisher to send velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscribe to odometry to get the current position and orientation
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TurtleBotMoveToGoal::odom_callback, this, std::placeholders::_1));

        // Timer to control movement towards the goal
        timer_ = this->create_wall_timer(
            100ms, std::bind(&TurtleBotMoveToGoal::move_toward_goal, this));
    }

    void set_move_backward(bool backward)
    {
        move_backward_ = backward;
    }

    // New function to check if both goals have been reached
    bool is_goal_completed() const
    {
        return reached_goal_1_ && reached_goal_2_;
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
    }

    void move_toward_goal()
    {
        if (!is_aligned_)
        {
            align_to_goal();
            return; // Wait until alignment is done before moving forward
        }

        double distance = std::sqrt(std::pow(goal_x_ - current_x_, 2) + std::pow(goal_y_ - current_y_, 2));
        auto message = geometry_msgs::msg::Twist();

        if (distance > 0.1) // Move if more than 0.1 meters away
        {
            double speed = std::min(0.2, distance); 
            message.linear.x = move_backward_ ? -speed : speed;
            // RCLCPP_INFO(this->get_logger(), "Moving %s to goal. Distance: %.2f", move_backward_ ? "backward" : "forward", distance);
        }
        else
        {
            message.linear.x = 0.0;
            velocity_publisher_->publish(message);

            if (!reached_goal_1_) {
                // Switch to the second goal after reaching the first
                reached_goal_1_ = true;
                RCLCPP_INFO(this->get_logger(), "Reached first goal. Switching to second goal.");
                set_new_goal(goal_pose_2_);
            } else if (!reached_goal_2_) {
                reached_goal_2_ = true;
                RCLCPP_INFO(this->get_logger(), "Reached final goal. Stopping.");
                timer_->cancel(); // Stop the timer after reaching the final goal
            }
            return;
        }

        velocity_publisher_->publish(message);
    }

    void align_to_goal()
    {
        double target_yaw = std::atan2(goal_y_ - current_y_, goal_x_ - current_x_);
        double yaw_error = target_yaw - current_yaw_;

        while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

        if (std::abs(yaw_error) > 0.05)
        {
            auto message = geometry_msgs::msg::Twist();
            message.angular.z = (yaw_error > 0) ? 0.3 : -0.3;
            velocity_publisher_->publish(message);
            // RCLCPP_INFO(this->get_logger(), "Aligning to goal. Yaw error: %.2f", yaw_error);
        }
        else
        {
            is_aligned_ = true;
            RCLCPP_INFO(this->get_logger(), "Alignment complete. Moving towards goal.");
        }
    }

    void set_new_goal(const geometry_msgs::msg::Pose &new_goal)
    {
        current_goal_ = new_goal;
        goal_x_ = current_goal_.position.x;
        goal_y_ = current_goal_.position.y;
        is_aligned_ = false; // Reset alignment for the new goal
        timer_->reset(); // Restart the timer
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Pose goal_pose_1_;
    geometry_msgs::msg::Pose goal_pose_2_;
    geometry_msgs::msg::Pose current_goal_;
    double goal_x_;
    double goal_y_;
    double current_x_;
    double current_y_;
    double current_yaw_;
    bool move_backward_;
    bool is_aligned_;
    bool reached_goal_1_;
    bool reached_goal_2_; // Flag to indicate both goals are reached
};




int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Define initial shelf statuses for each alley
    std::unordered_map<ItemType, std::vector<bool>> initial_shelf_statuses = {
        {ItemType::RAW,     {false, true, false, true, false}},
        {ItemType::HALF,    {true, false, true, false, true}},
        {ItemType::FINISHED, {false, false, false, false, false}}
    };

    WarehouseManager warehouse_manager(initial_shelf_statuses);

    // Repeat the main process twice
    for (int loop_count = 0; loop_count < 2; ++loop_count) {
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting loop iteration: %d", loop_count + 1);

        // Create a shared pointer for the LocationManager with the correct YAML file
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("warehouse_world");
        std::string yaml_filename = package_share_directory + "/config/locations.yaml";
        auto location_manager = std::make_shared<LocationManager>(yaml_filename);

        // Create the RobotController node
        auto robot_controller = std::make_shared<RobotController>(location_manager);


        // Initialize the WarehouseManager with the initial shelf statuses
        

        // Select an item type for this run (e.g., RAW)
        ItemType selected_item_type = ItemType::HALF;
        std::vector<std::string> FBroute;

        // Retrieve the dynamic route for the selected item type
        std::vector<std::string> route = warehouse_manager.get_route_for_item(selected_item_type, FBroute);

        if (route.empty()) {
            RCLCPP_ERROR(robot_controller->get_logger(), "No route found for the selected item type.");
            continue; // Skip to the next loop iteration if route is empty
        }

        const auto& entry_location = route[0];

        // Call the send_robot_to_location method and check for success
        bool success = robot_controller->send_robot_to_location(entry_location);
        rclcpp::spin_some(robot_controller);

        std::vector<std::string> points = warehouse_manager.get_point_for_item(selected_item_type);
        geometry_msgs::msg::Pose shelves_pose = location_manager->get_location(points[0]);
        geometry_msgs::msg::Pose shelves_pose_2 = location_manager->get_location(points[1]);

        auto node1 = std::make_shared<TurtleBotMoveToGoal>(shelves_pose, shelves_pose_2);
        node1->set_move_backward(false);

        while (rclcpp::ok()) {
            rclcpp::spin_some(node1);
            if (node1->is_goal_completed()) break;
        }

        // Loop through the remaining route
        for (size_t i = 1; i < route.size(); ++i) {
            const auto& location = route[i];
            bool success = robot_controller->send_robot_to_location(location);
            if (!success) {
                RCLCPP_ERROR(robot_controller->get_logger(), "Failed to navigate to %s. Exiting route.", location.c_str());
                break;
            }
            rclcpp::spin_some(robot_controller);
        }

        // Final navigation task using FBroute
        if (!FBroute.empty()) {
            geometry_msgs::msg::Pose goal_pose = location_manager->get_location(FBroute[0]);
            geometry_msgs::msg::Pose goal_pose_2 = location_manager->get_location(FBroute[1]);

            auto node = std::make_shared<TurtleBotMoveToGoal>(goal_pose, goal_pose_2);
            node->set_move_backward(false);

            while (rclcpp::ok()) {
                rclcpp::spin_some(node);
                if (node->is_goal_completed()) break;
            }
        }

        // Display warehouse status after each loop iteration
        warehouse_manager.display_warehouse_status();

        RCLCPP_INFO(rclcpp::get_logger("main"), "Completed loop iteration: %d", loop_count + 1);
    }

    rclcpp::shutdown();
    return 0;
}
