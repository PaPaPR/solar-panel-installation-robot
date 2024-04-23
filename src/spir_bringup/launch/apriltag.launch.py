import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('spir_bringup'),
        'cfg',
        'tags_36h11.yaml'
        )
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            parameters=[config],
            remappings=[
                ('/image_rect', '/image_raw'),
            ]
        ),
    ])