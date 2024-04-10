from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_pkg_share = FindPackageShare('spir_bringup').find('spir_bringup')
    return LaunchDescription([
        Node(
            package='spir_control',
            executable='state_publisher',
            name='state_publisher',
            # parameters=[{'hostname': '172.17.0.1'}, {'hostport': 9760}, {'recvport': 9761}]
            parameters=[{'hostname': '192.168.4.4'}, {'hostport': 9760}, {'recvport': 9760}]
        ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', urdf_pkg_share+'/rviz_view.rviz']
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_transform_publisher',
            arguments = ['0.00507', '-0.715', '0.2444', '0', '0', '0', 'end_effector', 'camera_optical_frame']
        ),
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
            arguments = ['0', '0', '-1', '0', '3.14159265', '0', 'camera_optical_frame', 'base']
        )
    ])