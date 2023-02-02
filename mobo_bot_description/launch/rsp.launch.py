import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro



def generate_launch_description():
    # delare any path variable
    pkg_path = get_package_share_directory('mobo_bot_description')

    xacro_file = os.path.join(pkg_path,'urdf','urdf_description.xacro')
    robot_description_doc = xacro.parse(open(xacro_file))
    xacro.process_doc(robot_description_doc)
    
    

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_gui = LaunchConfiguration('use_joint_state_gui')
    
    
    
    # declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='use gazebo simulation clock if True'
    )
    
    declare_use_joint_state_gui_cmd = DeclareLaunchArgument(
        'use_joint_state_gui',
        default_value='False',
        description='whether you want to use only rviz or with gazebo simulation'
    )

  
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_doc.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(use_joint_state_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )






     # Create the launch description and populate
    ld = LaunchDescription()


    # add the necessary declared launch arguments to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_joint_state_gui_cmd)

    # Add the nodes to the launch description
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)

    
    return ld      # return (i.e send) the launch description for excecution
