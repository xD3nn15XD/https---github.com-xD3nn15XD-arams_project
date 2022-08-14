import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='teleop',
            executable='teleop',
            name='teleop'
        ),

        Node(
            package='rqt_joy',
            executable='rqt_joy',
            name='joystick'
        )

    ])