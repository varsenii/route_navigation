# Copyright (c) 2025 Open Navigation LLC
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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    route_navigation_dir = get_package_share_directory('route_navigation')

    nav2_params_file = os.path.join(route_navigation_dir, 'config', 'nav2_params.yaml')
    map_yaml_file = os.path.join(route_navigation_dir, 'maps', 'apartment.yaml')
    graph_filepath = os.path.join(route_navigation_dir, 'graphs', 'apartment_z_shape.geojson')
    rviz_config_file = os.path.join(route_navigation_dir, 'rviz', 'nav.rviz')

    # start the visualization
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False',
                          'rviz_config': rviz_config_file}.items())

    # start navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(route_navigation_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file, 
            'graph': graph_filepath, 
            'use_sim_time': 'false',
            'params_file': nav2_params_file}.items())

    twist_converter_cmd = Node(
        package='route_navigation',
        executable='twist_converter',
        output='screen')

    # start the demo autonomy task
    route_nav_cmd = Node(
        package='route_navigation',
        executable='route_nav',
        emulate_tty=True,
        output='screen')

    ld = LaunchDescription()
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(twist_converter_cmd)
    # ld.add_action(route_nav_cmd)
    return ld
