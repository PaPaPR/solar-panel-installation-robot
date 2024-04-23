import threading

import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_ros
from sensor_msgs.msg import JointState

from spir_control import arm_protocol_handler

class StatePublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.declare_parameter('hostname', '192.168.4.4')
        self.declare_parameter('hostport', 9760)

        hostname = self.get_parameter('hostname').get_parameter_value().string_value
        hostport = self.get_parameter('hostport').get_parameter_value().integer_value

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.arm = arm_protocol_handler.ArmProtocolHandler(hostname, hostport)

        self.cmd_feedback_thread = threading.Thread(target = self.cmd_feedback)
        self.cmd_feedback_thread.start()

        self.state_query_timer = self.create_timer(0.1, self.state_query)

    def cmd_feedback(self):
        while True:
            # 接收到拍照指令
            if self.arm.command('photo'):
                self.arm.command('photo_reply')
                tf_trans = None
                retry_times = 0
                while retry_times < 3:
                    try:
                        tf_trans = self.tf_buffer.lookup_transform('end_effector', 'installation_calibration', rclpy.time.Time(), rclpy.duration.Duration(seconds=5))
                        retry_times = 3
                    except tf2_ros.TransformException as e:
                        retry_times += 1
                        self.get_logger().error(f'Could not lookup end_effector to installation_calibration transform : {e}')
                    if(tf_trans is not None and abs(tf_trans.transform.translation.x) < 0.1 and abs(tf_trans.transform.translation.y) < 0.1 and abs(tf_trans.transform.translation.z) < 0.1):
                        try:
                            tf_trans = self.tf_buffer.lookup_transform('end_effector', 'installation', rclpy.time.Time(), rclpy.duration.Duration(seconds=5))
                        except tf2_ros.TransformException as e:
                            retry_times += 1
                            self.get_logger().error(f'Could not lookup end_effector to installation transform : {e}')
                if tf_trans is not None:
                    self.arm.command('add_points', [[tf_trans.transform.translation.x, tf_trans.transform.translation.y, tf_trans.transform.translation.z],\
                                                    [tf_trans.transform.rotation.x, tf_trans.transform.rotation.y, tf_trans.transform.rotation.z, tf_trans.transform.rotation.w]])
                else:
                    self.get_logger().error('Tf translation is none when command is sending.')

    def state_query(self):
        while self.arm.isAvailable():
            state = self.arm.command('joint_state')
            if state is not None:
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = state['name']
                joint_state.position = state['position']
                joint_state.velocity = state['velocity']
                joint_state.effort = state['effort']
                self.joint_state_pub.publish(joint_state)
                t = tf2_ros.TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'base_link'
                t.child_frame_id = 'end_effector'
                t.transform.translation.x, t.transform.translation.y, t.transform.translation.z, \
                t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = \
                state['world']
                self.tf_broadcaster.sendTransform(t)
            else:
                self.get_logger().error(f'Timeout of joint state query.')
        self.arm_comm_retry_timer = self.create_timer(3, self.arm_comm_retry)

    def arm_comm_retry(self):
        while not self.arm.isAvailable():
            self.arm.init()

def main(args=None):
    rclpy.init(args=args)

    state_publisher = StatePublisher()

    rclpy.spin(state_publisher)

    state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()