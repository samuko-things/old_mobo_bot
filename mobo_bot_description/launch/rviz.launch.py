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
    my_pkg_path = get_package_share_directory('mobo_bot_description')
    

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_gui = LaunchConfiguration('use_joint_state_gui')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz_only = LaunchConfiguration('use_rviz_only')
    
    
    
    # declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='use gazebo simulation clock if True'
    )
    
    declare_use_joint_state_gui_cmd = DeclareLaunchArgument(
        'use_joint_state_gui',
        default_value='True',
        description='whether to use the joint-state-publisher-gui package'
    )
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(my_pkg_path,'config','view_urdf.rviz'),
        description='full path of the rviz configuration file'
    )
    
    declare_use_rviz_only_cmd = DeclareLaunchArgument(
        'use_rviz_only',
        default_value='True',
        description='whether you want to run only rviz or run both rviz and gazebo'
    )

  
  
    # create needed nodes or launch files
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(my_pkg_path,'launch','rsp.launch.py')]
            ), 
        launch_arguments={'use_sim_time': use_sim_time, 
                          'use_joint_state_gui': use_joint_state_gui}.items(),
        condition=IfCondition(use_rviz_only),
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')






     # Create the launch description and populate
    ld = LaunchDescription()


    # add the necessary declared launch arguments to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_joint_state_gui_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_only_cmd)
    

    # Add the nodes to the launch description
    ld.add_action(rsp_launch)
    ld.add_action(rviz_node)

    
    return ld      # return (i.e send) the launch description for excecution

