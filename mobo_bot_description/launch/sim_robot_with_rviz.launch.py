import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='mobo_bot_description' #<--- CHANGE ME


    is_sim_time = 'true'
    is_ros2_control = 'false'
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

     # declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value=is_sim_time,
        description='Use sim time if true'
    )
    
    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        'use_ros2_control',
        default_value=is_ros2_control,
        description='Use ros2_control if true'
    )




    sim_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory(package_name),'launch','sim_robot.launch.py')]
            ), 
            launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items()
    )


    rviz_config_file = os.path.join(get_package_share_directory(package_name),'config','sim_robot_with_rviz.config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )


   

    
    # Create the launch description and populate
    ld = LaunchDescription()


    # add the necessary declared launch arguments to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)
    
    # Add the nodes to the launch description
    ld.add_action(sim_robot)
    ld.add_action(rviz_node)
    
    return ld      # return (i.e send) the launch description for excecution
