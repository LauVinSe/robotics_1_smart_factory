import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
def generate_launch_description():
 
  # Set the path to the Gazebo ROS package
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
   
  # Set the path to this package.
  pkg_share = FindPackageShare(package='warehouse_world').find('warehouse_world')

  # Set the path to the world file
  world_file_name = 'amazon_robot.world'
  world_path = os.path.join(pkg_share, 'worlds', world_file_name)
   
  # Set the path to the SDF model files.
  gazebo_models_path = os.path.join(pkg_share, 'models')
  os.environ["GAZEBO_MODEL_PATH"] = f"{os.environ.get('GAZEBO_MODEL_PATH', '')}:{':'.join(gazebo_models_path)}"

  # Set robot Arguments
  x_robot_position = '0'
  y_robot_position = '0'
  z_robot_position = '1'

  declare_x_position_cmd = DeclareLaunchArgument(
    '-x', default_value=x_robot_position,
    description="Position on the axis x of robot"
  )
  declare_y_position_cmd = DeclareLaunchArgument(
    '-y', default_value=y_robot_position,
    description="Position on the axis y of robot"
  )
  declare_z_position_cmd = DeclareLaunchArgument(
    '-z', default_value=z_robot_position,
    description="Position on the axis z of robot"
  )

  ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
 
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')
 
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')
 
  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
    
  # Specify the actions

  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())
 
  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
  
  start_robot_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'spawn_amazon_model.launch.py')),
    launch_arguments = {'-x': x_robot_position, '-y': y_robot_position, '-z': z_robot_position}.items()
  )
 
  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Declare the launch options
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)
  #ld.add_action(declare_x_position_cmd)
  #ld.add_action(declare_y_position_cmd)
  #ld.add_action(declare_z_position_cmd)
 
  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  #ld.add_action(start_robot_cmd)
  ld.add_action(start_gazebo_client_cmd)

 
  return ld