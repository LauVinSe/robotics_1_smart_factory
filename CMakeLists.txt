cmake_minimum_required(VERSION 3.8)
project(warehouse_world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

## This is the executable

# For Sprint
add_executable(map_overlay src/sprint/map_overlay.cpp)
add_executable(object_detection src/sprint/object_detection.cpp)
add_executable(compare_maps src/sprint/compare_maps.cpp)

ament_target_dependencies(map_overlay rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge image_transport OpenCV)
ament_target_dependencies(object_detection rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge image_transport OpenCV visualization_msgs)
ament_target_dependencies(compare_maps rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge image_transport OpenCV visualization_msgs)

# Project focused
add_library(location_manager src/location_manager.cpp)
target_link_libraries(location_manager yaml-cpp)
ament_target_dependencies(location_manager rclcpp geometry_msgs tf2 tf2_geometry_msgs)


link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "tf2"
)

# Install world files
install(DIRECTORY
  config  
  launch
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  map_overlay
  object_detection
  compare_maps
  location_manager
  DESTINATION lib/${PROJECT_NAME})

  if(BUILD_TESTING)

  # Google Test
  find_package(ament_cmake_gtest REQUIRED)

  # GTest for LocationManager
  ament_add_gtest(test_location_manager test/test_location_manager.cpp)
  target_link_libraries(test_location_manager yaml-cpp location_manager)
  ament_target_dependencies(test_location_manager rclcpp geometry_msgs tf2 tf2_geometry_msgs)
 


endif()

ament_package()
