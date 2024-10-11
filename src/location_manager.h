#ifndef LOCATION_MANAGER_H
#define LOCATION_MANAGER_H

#include <string>
#include <map>
#include <geometry_msgs/msg/pose.hpp>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h> // To use getYaw function from the quaternion of orientation
#include <tf2/transform_datatypes.h>
#include <cmath>  // For M_PI and degree-to-radian conversion

class LocationManager {
public:
  LocationManager(const std::string &file_path);
  geometry_msgs::msg::Pose get_location(const std::string &location_name);

private:
  std::map<std::string, geometry_msgs::msg::Pose> locations_;
  void load_locations(const std::string &file_path);
  geometry_msgs::msg::Quaternion toQuaternionFromDegrees(double yaw_degrees);
};

#endif // LOCATION_MANAGER_H