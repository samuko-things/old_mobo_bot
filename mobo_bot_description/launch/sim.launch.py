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
    my_pkg_path = get_package_share_directory('mobo_bot_description')
    

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_gui = LaunchConfiguration('use_joint_state_gui')
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
        default_value=os.path.join(my_pkg_path, 'world', 'test_world.world'),
        description='Full path to world model file to load')
    
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='mobo_bot',
        description='name of the robot')

  
  
  
  
  
  
    # create needed nodes or launch files
    rsp_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(my_pkg_path,'launch','rsp.launch.py')]
                    ), 
                launch_arguments={'use_sim_time': use_sim_time, 
                                  'use_joint_state_gui': use_joint_state_gui}.items()
    )
    

    # entity_name = 'mobo_bot'
    # initial spawn position
    x_pos = 0; y_pos = 0; z_pos = 0
    #initial spawn orientation
    roll = 0; pitch = 0; yaw = 0
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', robot_name,
            '-x', str(x_pos), '-y', str(y_pos), '-z', str(z_pos),
            '-R', str(roll), '-P', str(pitch), '-Y', str(yaw)
            ],
        output='screen')
    
    start_gazebo_server_cmd = ExecuteProcess(
        # condition=IfCondition(use_simulator),
        cmd=['gazebo', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
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
    ld.add_action(rsp_launch)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_spawner_cmd)

    
    return ld      # return (i.e send) the launch description for excecution

