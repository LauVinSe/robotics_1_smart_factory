#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "robot_controller.h"
#include "location_manager.h"

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a shared pointer for the LocationManager (with the correct YAML file)
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("warehouse_world");
    std::string yaml_filename = package_share_directory + "/config/locations.yaml";
    auto location_manager = std::make_shared<LocationManager>(yaml_filename);

    // Create the RobotController node
    auto robot_controller = std::make_shared<RobotController>(location_manager);

    // Target location to send the robot to
    std::string target_location = "entry_shelf_half";

    // Send the robot to the target location
    RCLCPP_INFO(robot_controller->get_logger(), "Sending robot to location: %s", target_location.c_str());
    robot_controller->send_robot_to_location(target_location);

    // Keep the node alive to receive feedback and results from the action server
    rclcpp::spin(robot_controller);

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
