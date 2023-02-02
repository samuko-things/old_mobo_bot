import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, EmitEvent, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro





def generate_launch_description():
    # delare any path variable
    robot_decription_path = get_package_share_directory('mobo_bot_description')
    robot_cam_path = get_package_share_directory('mobo_bot_line_following')
    

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_gui = LaunchConfiguration('use_joint_state_gui') # used by rsp node
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')    
    
    
    # declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='use gazebo simulation clock if True'
    )
    
    declare_use_joint_state_gui_cmd = DeclareLaunchArgument(
        'use_joint_state_gui',
        default_value='False',
        description='whether to use the joint-state-publisher-gui package. use with rviz only'
    )
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(robot_decription_path, 'world', 'simple_line_track.world'),
        description='Full path to world model file to load')
    
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='mobo_bot',
        description='name of the robot')

  
  
  
    
    
    # create needed nodes or launch files
    sim_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(robot_decription_path,'launch','sim.launch.py')]
                    ), 
                launch_arguments={'use_sim_time': use_sim_time, 
                                  'use_joint_state_gui': use_joint_state_gui,
                                  'world': world,
                                  'robot_name': robot_name}.items()
    )
    
    
    read_camera_node = Node(
        package='mobo_bot_line_following',
        executable='read_camera',
        output='screen'
    )






     # Create the launch description and populate
    ld = LaunchDescription()


    # add the necessary declared launch arguments to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_joint_state_gui_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    

    # Add the nodes to the launch description
    ld.add_action(sim_launch)
    ld.add_action(read_camera_node)
    
    return ld      # return (i.e send) the launch description for excecution

