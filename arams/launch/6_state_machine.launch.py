from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
 
    return LaunchDescription([
        # detector_conector
        Node(
            package='arams',
            executable='state_machine',
            name = 'state_machine',
            output = 'screen',        
        )
    ])
