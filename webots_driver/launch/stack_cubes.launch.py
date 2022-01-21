import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    pkg_name = "webots_driver"
    pkg_share = get_package_share_directory(pkg_name)


    panda = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'panda.launch.py'),
        ),)


    run_move_group_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'run_move_group.launch.py'),
        ),)

    collision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'collision.launch.py'),
        ),)

    moveit_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'moveit_controller.launch.py'),
        ),)

    timer_2 = IncludeLaunchDescription(LaunchDescriptionSource(LaunchDescription([  # hack since you can't have recursive timer
        moveit_controller,
        TimerAction(
            period=10.,
            actions=[
                collision,
            ])
    ])))

    timer_1 = IncludeLaunchDescription(LaunchDescriptionSource(LaunchDescription([  # hack since you can't have recursive timer
        run_move_group_node,
        TimerAction(
            period=10.,
            actions=[
                timer_2
            ])
    ])))

    return LaunchDescription([
        panda,
        TimerAction(
            period=5.,
            actions=[
                timer_1
            ]
        )
    ])
