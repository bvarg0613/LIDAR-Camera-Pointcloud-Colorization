from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rig_tf',
            executable='static_rig_tf',
            name='static_rig_tf',
            output='screen',
        ),
    ])
