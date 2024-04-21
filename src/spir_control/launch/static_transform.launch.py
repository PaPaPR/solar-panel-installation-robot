from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_transform_publisher',
            arguments = ['0.00507', '0.715', '0.2174', '1.570796325', '0', '0', 'end_effector', 'camera_optical_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='install_calibrate_static_transform_publisher',
            arguments = ['0', '0', '-0.35', '0', '0', '0', 'installation', 'installation_calibration']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='install_static_transform_publisher',
            arguments = ['-0.7', '0.036', '0.268', '0.715', '0.699', '-0.016', '0.011', 'tag', 'installation']
        ),

        # Install position calculate tf
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='target_static_transform_publisher',
        #     arguments = ['2.194043', '-0.129242', '0.916813','0.000' , '-0.574', '-3.142', 'base_link', 'target']
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='tf2_echo',
        #     name='baselink_to_installation_tf_echo',
        #     arguments = ['base_link end_effector']
        # ),
    ])