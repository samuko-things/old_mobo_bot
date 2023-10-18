import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='mobo_bot_description' #<--- CHANGE ME

    # Check if we're told to use sim time
    is_sim_time = 'true'
    use_sim_time = LaunchConfiguration('use_sim_time')
    
     # declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value=is_sim_time,
        description='Use sim time if true'
    )
    
    # Start robot localization using an Extended Kalman filter
    ekf_config_file = os.path.join(get_package_share_directory(package_name),'config', 'ekf_fuse_imu_odom.yaml')
    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}])

    
    # Create the launch description and populate
    ld = LaunchDescription()


    # add the necessary declared launch arguments to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add the nodes to the launch description
    ld.add_action(node_ekf)
        
    return ld      # return (i.e send) the launch description for excecution