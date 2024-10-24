# Robotics 1 Smart Factory

This repository contains a ROS2 project focused on simulating a smart factory using autonomous mobile robots (TurtleBot). The project includes navigation, path planning, and interaction with the environment in a simulated Gazebo world.

![warehouse](images/warehouse_index.png)
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
## Launch the Warehouse World
Once the build is successful and the environment is sourced, you can launch the package with the following commands.
```bash
ros2 launch warehouse_world warehouse_world.launch.py
```

## Using map_overlay
### Prerequisites
- [RTAB_ROS](https://github.com/introlab/rtabmap_ros/tree/humble-devel)
- [Gazebo Map Creator](https://github.com/arshadlab/gazebo_map_creator)

Create the required ground truth map using Gazebo map creator (follow the instruction in the link above).

Tips: only take the 2D map and make sure the map fit closely.

Then, modify the path inside the source code (**Might be updated later) to your path to the ground truth map. 

For example:
```bash
/home/user/git/warehouse_world/map/map.png
```
Now you can build the package again. 
```bash
cd ros2_ws/
colcon build --packages-select warehouse_world
```
Launch the world and then run map_overlay node

In terminal 1:
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch warehouse_world warehouse_world.launch.py
```
In terminal 2:
```bash
ros2 run warehouse_world map_overlay
```
Also, launch the RTAB as specified in the link above or as bellow (in seperate terminal):
```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
   visual_odometry:=false \
   frame_id:=base_footprint \
   subscribe_scan:=true depth:=false \
   approx_sync:=true \
   odom_topic:=/odom \
   scan_topic:=/scan \
   qos:=2 \
   args:="-d --RGBD/NeighborLinkRefining true --Reg/Strategy 1" \
   use_sim_time:=true \
   rviz:=true
```
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```
```bash
ros2 launch nav2_bringup rviz_launch.py
```

Saving the map from rtabmap use:
```bash
ros2 run nav2_map_server map_saver_cli -f my_saved_map --ros-args -r /map:=/rtabmap/map
```

## Using Path Planning
In terminal 1 open the world:
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch warehouse_world warehouse_world.launch.py
```
In terminal 2 open the map file. For example:
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/git/warehouse_world/map/rtbmap/rtab_warehouse_map.yaml 
```

## Using Object Detection
Run the path Planning command in terminals. Then:
```bash
ros2 run warehouse_world object_detection
```

