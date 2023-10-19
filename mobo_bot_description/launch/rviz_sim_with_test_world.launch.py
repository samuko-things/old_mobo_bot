import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='mobo_bot_description' #<--- CHANGE ME

    rviz_config_file = os.path.join(get_package_share_directory(package_name),'config','sim_with_test_world.config.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    
    # Create the launch description and populate
    ld = LaunchDescription()

    
    # Add the nodes to the launch description
    ld.add_action(rviz_node)
    
    return ld      # return (i.e send) the launch description for excecution
