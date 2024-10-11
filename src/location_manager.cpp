#include "location_manager.h"
#include <iostream>

LocationManager::LocationManager(const std::string &file_path) {
  load_locations(file_path);
}

// Function to convert yaw in degrees to quaternion
geometry_msgs::msg::Quaternion LocationManager::toQuaternionFromDegrees(double yaw_degrees) {
    // Convert degrees to radians
    double yaw_radians = yaw_degrees * (M_PI / 180.0);

    // Create a tf2 quaternion and set the yaw (roll = 0, pitch = 0, yaw = yaw_radians)
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_radians);

    // Convert tf2 quaternion to geometry_msgs quaternion
    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();
    quat_msg.w = q.w();

    return quat_msg;
}

void LocationManager::load_locations(const std::string &file_path) {
  YAML::Node locations = YAML::LoadFile(file_path);

  for (const auto& location : locations["locations"]) {
    std::string name = location.first.as<std::string>();
    geometry_msgs::msg::Pose pose;

    // Extract position
    pose.position.x = location.second["position"]["x"].as<double>();
    pose.position.y = location.second["position"]["y"].as<double>();
    pose.position.z = location.second["position"]["z"].as<double>();

    // Extract orientation (yaw in degrees) and convert to radians
    double yaw_degrees = location.second["orientation"]["yaw"].as<double>();
    double yaw_radians = yaw_degrees * (M_PI / 180.0);  // Convert degrees to radians

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw_radians);

    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    // Store the pose in the map
    locations_[name] = pose;
  }
}

geometry_msgs::msg::Pose LocationManager::get_location(const std::string &location_name) {
  if (locations_.find(location_name) != locations_.end()) {
    return locations_[location_name];
  } else {
    std::cerr << "Location " << location_name << " not found!" << std::endl;
    return geometry_msgs::msg::Pose();  // Return a default Pose if not found
  }
}
