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
    
    world_file_name = 'test_world.world'
    world_path = os.path.join(get_package_share_directory(package_name), 'world', world_file_name)
    
    
    declare_world_cmd = DeclareLaunchArgument(
            'world',
            default_value=world_path,
            description='SDF world file',
        )


    use_sim_time = 'true'
    use_ros2_control = 'false'
    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory(package_name),'launch','rsp.launch.py')]
            ), 
            launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items()
    )


    rviz_config_file = os.path.join(get_package_share_directory(package_name),'config','robot_view_sim.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
                ),
                launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )


    robot_name = 'mobo_bot'
    # initial spawn position
    x_pos = 0; y_pos = 0; z_pos = 0
    #initial spawn orientation
    roll = 0; pitch = 0; yaw = 0
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', robot_name,
            '-x', str(x_pos), '-y', str(y_pos), '-z', str(z_pos),
            '-R', str(roll), '-P', str(pitch), '-Y', str(yaw)
            ],
        output='screen')


    # # Start robot localization using an Extended Kalman filter
    # ekf_file_path = os.path.join(get_package_share_directory(package_name),'config', 'ekf.yaml')
    # node_ekf = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_file_path, {'use_sim_time': 'true'}])


    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    
    # Then add the following below the current diff_drive_controller_spawner
    delayed_diff_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_controller_spawner],
        )
    )
    # Replace the diff_drive_controller_spawner in the final return with delayed_diff_drive_controller_spawner


    
    # Create the launch description and populate
    ld = LaunchDescription()


    # add the necessary declared launch arguments to the launch description
    ld.add_action(declare_world_cmd)
    
    # Add the nodes to the launch description
    ld.add_action(rsp)
    ld.add_action(rviz_node)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    
    if use_ros2_control=='true':
        ld.add_action(joint_state_broadcaster_spawner)
        # ld.add_action(diff_drive_controller_spawner)
        ld.add_action(delayed_diff_drive_controller_spawner)

    
    return ld      # return (i.e send) the launch description for excecution