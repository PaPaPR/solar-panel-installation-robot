from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spir_control',
            executable='state_publisher',
            name='state_publisher',
            parameters=[{'hostname': '192.168.4.4'}, {'hostport': 9760}]
        ),
    ])