import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node



def generate_launch_description():

    my_nav_pkg_path = get_package_share_directory('mobo_bot_nav2d')
    description_pkg_path = get_package_share_directory('mobo_bot_description')
    

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    
     # declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )
    
    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='false',
        description='Use ros2_control if true'
    )



    sim_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(description_pkg_path,'launch','sim_robot.launch.py')]
            ), 
            launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items()
    )


    rviz_config_file = os.path.join(my_nav_pkg_path,'config','build_map_with_slam.config.rviz')
    # rviz_config_file = os.path.join(description_pkg_path,'config','sim_robot_with_rviz.config.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    
    slam_mapping_param_file = os.path.join(my_nav_pkg_path,'config','my_slam_mapping_params_online_async.yaml')
    slam_mapping_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_mapping_param_file],
    #     remappings=[
    #     # ('/odom', '/turtlesim1/turtle1/pose'),
    #     # ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
    # ]
    )





    # Create the launch description and populate
    ld = LaunchDescription()


    # add the necessary declared launch arguments to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)
    
    # Add the nodes to the launch description
    ld.add_action(sim_robot)
    ld.add_action(rviz_node)
    ld.add_action(slam_mapping_node)

    return ld




    