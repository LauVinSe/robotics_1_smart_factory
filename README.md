# Robotics 1 Smart Factory

This repository contains a ROS2 project focused on simulating a smart factory using autonomous mobile robots (TurtleBot). The project includes navigation, path planning, and interaction with the environment in a simulated Gazebo world.

## Getting Started

Follow the instructions below to clone this repository, build it using `colcon`, and run the package.

### Prerequisites

- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) installed
- [Gazebo simulator](http://gazebosim.org/)

### Clone the Repository

First, clone this repository to your ROS2 workspace:

If you don't have ROS2 workspace:
```bash
mkdir ros2_ws
```
then clone the repository:
```bash
cd ros2_ws/src
git clone https://github.com/LauVinSe/robotics_1_smart_factory.git
```
or using SSH
```bash
git remote add origin git@github.com:LauVinSe/robotics_1_smart_factory.git
```

### Build the package
WARNING: Make sure you are in the root of your workspace (ros2_ws)!!!
```bash
cd ..
colcon build --packages-select warehouse_world
```
After the build process is complete, source the workspace:
```bash
source install/setup.bash 
```
### Build the package
Once the build is successful and the environment is sourced, you can run the package with the following commands.
```bash
ros2 launch warehouse_world warehouse_world.launch.py
```
