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





def generate_launch_description():
    # delare any path variable
    my_pkg_path = get_package_share_directory('mobo_bot_description')
    
    rviz_config_file = os.path.join(my_pkg_path,'config','robot_sim.rviz')

    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )




     # Create the launch description and populate
    ld = LaunchDescription()
    
    ld.add_action(rviz_node)

    
    return ld      # return (i.e send) the launch description for excecution

