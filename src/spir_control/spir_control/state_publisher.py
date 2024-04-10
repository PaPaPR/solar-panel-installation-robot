import socket
import json
import threading
import time
import math

import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_ros
from sensor_msgs.msg import JointState

from spir_control.euler_from_quaternion import *

class StatePublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.declare_parameter('hostname', '192.168.4.4')
        self.declare_parameter('hostport', 9760)
        self.declare_parameter('recvport', 9760)

        self.hostname = self.get_parameter('hostname').get_parameter_value().string_value
        self.hostport = self.get_parameter('hostport').get_parameter_value().integer_value
        self.recvhost = ''
        self.recvport = self.get_parameter('recvport').get_parameter_value().integer_value

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.sock_lock = threading.Lock()

        self.receiveThread = threading.Thread(target = self.state_receive)
        self.receiveThread.start()
        self.feedbackThread = threading.Thread(target = self.command_feedback)
        self.feedbackThread.start()
        

    def state_receive(self):
        initialized = False
        while not initialized:
            try:
                self.sendSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sendSock.connect((self.hostname, self.hostport))
                self.timer = self.create_timer(0.1, self.state_inquiry)
                initialized = True
                self.get_logger().debug(f'Connect to {self.hostname}:{self.hostport} succeed.')
            except Exception as e:
                self.get_logger().error(f'Send initialization failed with error: {e}, Connect to {self.hostname}:{self.hostport} failed! Retrying...')
                time.sleep(5)

        BUFF_SIZE = 4096
        data = b''
        while True:
            try:
                chunk = self.sendSock.recv(BUFF_SIZE)
                if not chunk:
                    continue
                if len(chunk) % BUFF_SIZE == 0:
                    data += chunk
                else:
                    data = chunk
                    try:
                        json_data = json.loads(data.decode('utf-8'))
                        data = b''
                        self.robot_state_publish(json_data)
                    except json.JSONDecodeError:
                        continue
            except:
                continue

    def state_inquiry(self):
        sendData = b'{"dsID":"www.hc-system.com.RemoteMonitor","reqType":"query","packID":"0","packID":"0",'+ \
                    b'"queryAddr":["axis-0","axis-1","axis-2","axis-3","axis-4","axis-5","curTorque-0",'+ \
                    b'"curTorque-1","curTorque-2","curTorque-3","curTorque-4","curTorque-5",'+ \
                    b'"curSpeed-0","curSpeed-1","curSpeed-2","curSpeed-3","curSpeed-4","curSpeed-5",'+ \
                    b'"world-0","world-1","world-2","world-3","world-4","world-5"]}'
        with self.sock_lock:
            try:
                self.sendSock.sendall(sendData)
            except Exception as e:
                self.get_logger().error(f'Send inquiry failed with error: {e}, Send to {self.hostname}:{self.hostport} failed! Retrying...')
                time.sleep(5)

    def command_feedback(self):
        initialized = False
        while not initialized:
            try:
                self.recvSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.recvSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.recvSock.bind((self.recvhost, self.recvport))
                self.recvSock.listen(10)
                self.recvConn, addr = self.recvSock.accept()
                initialized = True
                self.get_logger().debug(f'Bind to {self.recvhost}:{self.recvport} succeed.')
            except OSError as e:
                self.get_logger().error(f'Recive initialization failed with error: {e}, Bind to {self.recvhost}:{self.recvport} failed!')
                time.sleep(5)
        BUFF_SIZE = 4096
        data = b''
        while True:
            try:
                chunk = self.recvConn.recv(BUFF_SIZE)
                if not chunk:
                    continue
                if len(chunk) % BUFF_SIZE == 0:
                    data += chunk
                else:
                    data = chunk
                    json_data = json.loads(data.decode('utf-8'))
                    data = b''
                    # camera state feedback
                    if json_data['reqType'] == 'photo':
                        json_data['ret'] = 1
                        self.get_logger().debug('Received request of destination')
                        # lookup word coordinate of installing position
                        self.tf_buffer = tf2_ros.Buffer()
                        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
                        tag_frame = 'installation'
                        try:
                            tf_trans = self.tf_buffer.lookup_transform('base_link', tag_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=3))
                            with self.sock_lock:
                                time.sleep(0.01)
                                self.sendSock.sendall(json.dumps(json_data).encode())
                                self.get_logger().debug(f'Responded request of destination with: {json.dumps(json_data)}')
                                time.sleep(0.01)
                            translation = [tf_trans.transform.translation.x, tf_trans.transform.translation.y, tf_trans.transform.translation.z]
                            rotation = euler_from_quaternion(tf_trans.transform.rotation)
                            with self.sock_lock:
                                self.robot_target_send(translation, rotation)
                        except tf2_ros.TransformException as e:
                            self.get_logger().error(f'Could not transform : {e}')
                            continue
            except Exception as e:
                self.get_logger().error(f'Feedback failed with error: {e}')
                continue

    def robot_state_publish(self, json_data):
        query_addr = json_data['queryAddr']
        query_data = json_data['queryData']
        addr_data_mapping = dict(zip(query_addr, query_data))
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [k for k, v in addr_data_mapping.items() if k.startswith('axis')]
        joint_state.position = [(float(k) / 180. * math.pi) for k in [v for k, v in addr_data_mapping.items() if k.startswith('axis')]]
        joint_state.position[2] = -joint_state.position[2]
        joint_state.position[3] = -joint_state.position[3]
        joint_state.position[4] = -joint_state.position[4]
        joint_state.position[5] = -joint_state.position[5]
        joint_state.velocity = [float(k) for k in [v for k, v in addr_data_mapping.items() if k.startswith('curSpeed')]]
        joint_state.effort = [float(k) for k in [v for k, v in addr_data_mapping.items() if k.startswith('curTorque')]]
        self.publisher_.publish(joint_state)
        tf_baselink2endeffertor = [float(k) for k in [v for k, v in addr_data_mapping.items() if k.startswith('world')]]
        self.tf_publish_world(tf_baselink2endeffertor)

    def robot_target_send(self, translation, rotation):
        json_data = json.loads('{"dsID":"www.hc-system.com.cam", "reqTyTypype":"AddPoints", "dsData":[{"camID":"0", "data":[{"ModelID":"0","Similarity":"0","Color":"0","Rel":"0"}]}]}')
        json_data['dsData'][0]['data'][0]['X'] = translation[0]
        json_data['dsData'][0]['data'][0]['Y'] = translation[1]
        json_data['dsData'][0]['data'][0]['Z'] = translation[2]
        json_data['dsData'][0]['data'][0]['U'] = rotation[0]
        json_data['dsData'][0]['data'][0]['V'] = rotation[1]
        json_data['dsData'][0]['data'][0]['Angel'] = rotation[2]
        # todo: 对接数值大小、正负变化
        time.sleep(0.01)
        self.sendSock.sendall(json.dumps(json_data).encode())
        self.get_logger().debug(f'Feedback of destination: {json.dumps(json_data)}')
        time.sleep(0.01)

    def tf_publish_world(self, transform):
        t = tf2_ros.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'end_effector'
        t.transform.translation.x = transform[0] / 1000.
        t.transform.translation.y = transform[1] / 1000.
        t.transform.translation.z = transform[2] / 1000.
        q = quaternion_from_euler(transform[3]/ 180. * math.pi, transform[4]/ 180. * math.pi, transform[5]/ 180. * math.pi)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    state_publisher = StatePublisher()

    rclpy.spin(state_publisher)

    state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()