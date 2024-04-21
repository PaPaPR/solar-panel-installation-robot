import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('spir_bringup'), 'launch'),
            '/mv_camera.launch.py'])
        )
    static_tf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('spir_control'), 'launch'),
            '/static_transform.launch.py'])
        )
    spir_control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('spir_control'), 'launch'),
            '/spir_control.launch.py'])
        )
    robot_state_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('spir_description'), 'launch'),
            '/spir_state.launch.py'])
        )
    return LaunchDescription([
        camera_node,
        static_tf_node,
        spir_control_node,
        robot_state_node,
    ])