import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

detector_node_params = {
    # Yolo-tiny
    'network.config' : os.path.join( get_package_share_directory('openrobotics_darknet_ros'), 'config', 'yolov3-tiny.cfg'),
    'network.weights' : os.path.join( get_package_share_directory('openrobotics_darknet_ros'), 'config', 'yolov3-tiny.weights'),
    # Yolo
    # 'network.config' : os.path.join( get_package_share_directory('openrobotics_darknet_ros'), 'config', 'yolov3.cfg'),
    # 'network.weights' : os.path.join( get_package_share_directory('openrobotics_darknet_ros'), 'config', 'yolov3.weights'),

    'network.class_names' : os.path.join( get_package_share_directory('openrobotics_darknet_ros'), 'config', 'coco.names'),
    'detection.threshold' : 0.25,
    'detection.nms_threshold' : 0.45,
}


def generate_launch_description():
 
    return LaunchDescription([
        # Darknet
        Node(
            package='openrobotics_darknet_ros',
            executable='detector_node',
            name = 'detector_node',
            parameters=[detector_node_params],
            output = 'screen',
        ),

        # detector_conector
        Node(
            package='img_pub',
            executable='img_pub',
            name = 'detector_conector',
            output = 'screen',
        )
    ])