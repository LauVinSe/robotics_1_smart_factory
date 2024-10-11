// These headers are essential for the test
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// Include the header file for the class that you are testing
#include "../src/location_manager.h"

// We state the test suite name and the test name
TEST(LocationManagerTest, GetShelf1Location) {
    // Locate the YAML file from the config directory of the package
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("warehouse_world"); // Adjust the package name if necessary
    std::string yaml_filename = package_share_directory + "/config/locations.yaml"; // Update the path if necessary

    // Instantiate the LocationManager class with the YAML file
    LocationManager location_manager(yaml_filename);

    // Fetch the location for "shelf_1"
    geometry_msgs::msg::Pose pose = location_manager.get_location("entry_shelf_half");

    // Expected values for shelf_1 (update these according to your YAML file)
    geometry_msgs::msg::Pose expected_pose;
    expected_pose.position.x = -0.5;
    expected_pose.position.y = -4.5;
    expected_pose.position.z = 0.0;

    // The expected quaternion from yaw=3.14 (180 degrees)
    expected_pose.orientation.x = 0.0;
    expected_pose.orientation.y = 0.0;
    expected_pose.orientation.z = 1.0;  // For a yaw of 3.14 radians
    expected_pose.orientation.w = 0.0;  // For a yaw of 3.14 radians

    // Print the extracted location and expected for debugging
    std::cout << "Expected Location: " << expected_pose.position.x << ", " << expected_pose.position.y << ", " << expected_pose.position.z << std::endl;
    std::cout << "Extracted Location: " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << std::endl;
    
    // Assert that the fetched pose matches the expected pose
    ASSERT_DOUBLE_EQ(pose.position.x, expected_pose.position.x);
    ASSERT_DOUBLE_EQ(pose.position.y, expected_pose.position.y);
    ASSERT_DOUBLE_EQ(pose.position.z, expected_pose.position.z);

    // Print the extracted orientation and expected for debugging
    std::cout << "Expected orientation: " << expected_pose.orientation.x << ", " << expected_pose.orientation.y << ", " << expected_pose.orientation.z << ", " << expected_pose.orientation.w << std::endl;
    std::cout << "Extracted orientation: " << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << ", " << pose.orientation.w << std::endl;

    // Use EXPECT_NEAR to account for small floating-point differences
    EXPECT_NEAR(pose.orientation.x, expected_pose.orientation.x, 1e-6);
    EXPECT_NEAR(pose.orientation.y, expected_pose.orientation.y, 1e-6);
    EXPECT_NEAR(pose.orientation.z, expected_pose.orientation.z, 1e-6);
    EXPECT_NEAR(pose.orientation.w, expected_pose.orientation.w, 1e-6);
}

// We state the test suite name and the test name for invalid location
TEST(LocationManagerTest, GetInvalidLocation) {
    // Locate the YAML file from the config directory of the package
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("warehouse_world"); // Adjust the package name if necessary
    std::string yaml_filename = package_share_directory + "/config/locations.yaml"; // Update the path if necessary

    // Instantiate the LocationManager class with the YAML file
    LocationManager location_manager(yaml_filename);

    // Try fetching an invalid location
    geometry_msgs::msg::Pose pose = location_manager.get_location("invalid_shelf");

    // Print the extracted location and expected for debugging
    std::cout << "Extracted Location: " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << std::endl;

    // Print the extracted orientation and expected for debugging
    std::cout << "Extracted orientation: " << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << ", " << pose.orientation.w << std::endl;

    // Assert that the returned pose is the default pose (all zeros)
    geometry_msgs::msg::Pose default_pose;
    ASSERT_DOUBLE_EQ(pose.position.x, default_pose.position.x);
    ASSERT_DOUBLE_EQ(pose.position.y, default_pose.position.y);
    ASSERT_DOUBLE_EQ(pose.position.z, default_pose.position.z);
    ASSERT_DOUBLE_EQ(pose.orientation.x, default_pose.orientation.x);
    ASSERT_DOUBLE_EQ(pose.orientation.y, default_pose.orientation.y);
    ASSERT_DOUBLE_EQ(pose.orientation.z, default_pose.orientation.z);
    ASSERT_DOUBLE_EQ(pose.orientation.w, default_pose.orientation.w);
}