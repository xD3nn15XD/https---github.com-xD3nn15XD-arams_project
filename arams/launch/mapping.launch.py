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
    # bringup_dir = get_package_share_directory('arams')
    bringup_dir_tb3_gazebo = get_package_share_directory('tb3_gazebo')
    bringup_dir_my_robot_slam = get_package_share_directory('my_robot_slam')

    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([

        # RViz
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(bringup_dir_my_robot_slam, 'config', 'rviz_config.rviz'),
            description='Full path to the RViz parameters file to use'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=f'-d {rviz_config}'
        ),

        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir_tb3_gazebo, 'launch', 'arams.launch.py')
            ),
        ),     

        # Joystick
        Node(
            package='teleop',
            executable='teleop',
            name='teleop'
        ),

        Node(
            package='rqt_joy',
            executable='rqt_joy',
            name='joystick'
        ),

        # SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir_my_robot_slam, 'launch', 'slam_toolbox.launch.py')
            ),
        ),
    ])