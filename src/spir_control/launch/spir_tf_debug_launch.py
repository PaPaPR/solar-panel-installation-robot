from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='install_static_transform_publisher',
            arguments = ['-0.715', '0', '0', '3.14159265', '0', '0', 'base', 'installation']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='detect_static_transform_publisher',
            arguments = ['0', '0', '1', '0', '0', '0', 'camera_optical_frame', 'base']
        )
    ])