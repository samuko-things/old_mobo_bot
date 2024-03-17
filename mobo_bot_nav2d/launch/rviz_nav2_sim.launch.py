# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    # bringup_dir = get_package_share_directory('nav2_bringup')

    my_nav_pkg_path = get_package_share_directory('mobo_bot_nav2d')
    description_pkg_path = get_package_share_directory('mobo_bot_description')

    # rviz_config_file = os.path.join(description_pkg_path,'config','sim_robot_with_rviz.config.rviz')
    # rviz_config_file = os.path.join(my_nav_pkg_path,'config','localize_with_amcl.config.rviz')
    rviz_config_file = os.path.join(my_nav_pkg_path,'config','navigate_to_goal.config.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )



    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(rviz_node)

    return ld