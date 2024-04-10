import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_pkg_share = FindPackageShare('spir_description').find('spir_description')
    urdf_file = os.path.join(urdf_pkg_share, 'urdf', 'spir_description.urdf')
    robot_desc = open(urdf_file).read()
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        )
    ])