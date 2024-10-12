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
#include <cmath> // For M_PI and degree-to-radian conversion

/**
 * @class LocationManager
 * @brief Manages locations by loading them from a file and providing access to their poses.
 *
 * This class is responsible for loading location data from a specified file and
 * providing methods to retrieve the pose of a given location. It also includes
 * utility functions to handle pose transformations.
 */

class LocationManager
{
public:
  /**
   * @brief Constructs a LocationManager object and loads locations from the specified file.
   *
   * @param file_path The path to the file containing location data.
   */
  LocationManager(const std::string &file_path);

  /**
   * @brief Retrieves the pose of a specified location.
   *
   * @param location_name The name of the location whose pose is to be retrieved.
   * @return geometry_msgs::msg::Pose The pose of the specified location.
   */
  geometry_msgs::msg::Pose get_location(const std::string &location_name);

private:
  std::map<std::string, geometry_msgs::msg::Pose> locations_;
  /**
   * @brief Loads locations from the specified file.
   *
   * @param file_path The path to the file containing location data.
   */
  void load_locations(const std::string &file_path);

  /**
   * @brief Converts a yaw angle in degrees to a quaternion.
   *
   * @param yaw_degrees The yaw angle in degrees.
   * @return geometry_msgs::msg::Quaternion The corresponding quaternion.
   */
  geometry_msgs::msg::Quaternion toQuaternionFromDegrees(double yaw_degrees);
};

#endif // LOCATION_MANAGER_H