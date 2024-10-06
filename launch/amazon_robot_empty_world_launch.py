import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set the path to the Gazebo ROS package
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    # Include the robot description launch file
    robot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            FindPackageShare('warehouse_world').find('warehouse_world'),
            'launch', 'robot_amazon_description.launch.py'))  # Replace with the exact file name
    )

    # Include the spawner launch file
    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            FindPackageShare('warehouse_world').find('warehouse_world'),
            'launch', 'spawn_amazon_model.launch.py'))  # Replace with the exact file name
    )

    # Start Gazebo server with an empty world
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': 'empty.world'}.items()
    )

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add actions to launch Gazebo, robot description, and the spawner
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_description_cmd)
    ld.add_action(spawn_robot_cmd)

    return ld