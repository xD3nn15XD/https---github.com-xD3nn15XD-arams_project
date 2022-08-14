import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the launch directory
    bringup_dir_apriltag_ros = get_package_share_directory('apriltag_ros')

    return LaunchDescription([

        # Apriltag
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir_apriltag_ros, 'launch', 'tag_16h5_all.launch.py')
            ),
        ),

        # apriltag_listener
        Node(
            package='apriltag_listener',
            executable='apriltag_listener',
            name='apriltag_listener',
            output = 'screen',
        ),
    ])