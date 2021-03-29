# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Launch a talker and a listener."""

import os

from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    preprocess_params= os.path.join(
        get_package_share_directory('ros_av_toolkit'),
        'config',
        'preprocess_params.yaml'
    )

    ground_filter_params = os.path.join(
        get_package_share_directory('ros_av_toolkit'),
        'config',
        'ground_filter_params.yaml'
    )

    cluster_params = os.path.join(
        get_package_share_directory('ros_av_toolkit'),
        'config',
        'cluster_params.yaml'
    )

    bounding_box_params = os.path.join(
        get_package_share_directory('ros_av_toolkit'),
        'config',
        'bounding_box_params.yaml'
    )

    return LaunchDescription([
        launch_ros.actions.Node(package='ros_av_toolkit', executable='perception_lidar_preprocess', name='perception_lidar_preprocess',output='screen', parameters=[preprocess_params]),
        launch_ros.actions.Node(package='ros_av_toolkit', executable='perception_lidar_ground_filter', name='perception_lidar_ground_filter', output='screen', parameters=[ground_filter_params]),
        launch_ros.actions.Node(package='ros_av_toolkit', executable='perception_lidar_cluster', name='perception_lidar_cluster', output='screen', parameters=[cluster_params]),
        launch_ros.actions.Node(package='ros_av_toolkit', executable='perception_lidar_bounding_box', name='perception_lidar_bounding_box', output='screen', parameters=[bounding_box_params])
    ])
