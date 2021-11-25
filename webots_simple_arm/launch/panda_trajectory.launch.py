#!/usr/bin/env python

# Copyright 1996-2019 Cyberbotics Ltd.
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

"""Launch Webots and the controller."""

import os
import launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from webots_ros2_core.utils import ControllerLauncher
from webots_ros2_core.webots_launcher import WebotsLauncher
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('webots_simple_arm')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'panda.wbt'),
        mode="realtime",
        gui="True"
    )

    # Driver node
    controller = ControllerLauncher(
        package="webots_simple_arm",
        executable="panda_trajectory",
        ## If you give it no matter the value it will be interpreted as TRUE
        # parameters=[
        #     {
        #         'use_joint_state_publisher': "False",

        #     }],
        output='screen',
        arguments=[
            '--webots-robot-name', "panda",
        ],
    )
    return LaunchDescription([
        webots,controller,
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=launch.events.Shutdown())],
            )
        )    ])
