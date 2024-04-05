import socket
import json
import threading
import time

import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import JointState

class StatePublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.declare_parameter('hostname', '192.168.4.4')
        self.declare_parameter('hostport', 9760)

        self.hostname = self.get_parameter('hostname').get_parameter_value().string_value
        self.hostport = self.get_parameter('hostport').get_parameter_value().integer_value
        initialized = False
        while not initialized:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((self.hostname, self.hostport))
                self.receiveThread = threading.Thread(target = self.state_receive, args = (self.sock,))
                self.receiveThread.start()
                initialized = True
            except Exception as e:
                self.get_logger().error(f'Initialization failed with error: {e}, Connect to {self.hostname}:{self.hostport} failed! Retrying...')
                time.sleep(1)

        self.timer = self.create_timer(1, self.state_inquiry)

    def state_inquiry(self):
        sendData = b'["dsID":"www.hc-system.com.RemoteMonitor","reqType":"query"'+ \
                    b'"packID":"0","packID":"0","queryAddr":["axis-0","axis-1"'+ \
                    b'"axis-2","axis-3","axis-4","axis-5","curTorque-0"'+ \
                    b'"curTorque-1","curTorque-2","curTorque-3","curTorque-4"'+ \
                    b'"curTorque-5","curSpeed-0","curSpeed-1","curSpeed-2"'+ \
                    b'"curSpeed-3","curSpeed-4","curSpeed-5"]]'
        self.sock.sendall(sendData)

    def state_receive(self, socket):
        BUFF_SIZE = 4096
        data = b''
        while True:
            try:
                chunk = socket.recv(BUFF_SIZE)
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

    def robot_state_publish(self, json_data):
        query_addr = json_data['queryAddr']
        query_data = json_data['queryData']
        addr_data_mapping = dict(zip(query_addr, query_data))
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = {k for k, v in addr_data_mapping.items() if k.startswith('axis')}
        joint_state.position = [float(k) for k in {v for k, v in addr_data_mapping.items() if k.startswith('axis')}]
        joint_state.velocity = [float(k) for k in {v for k, v in addr_data_mapping.items() if k.startswith('curSpeed')}]
        joint_state.effort = [float(k) for k in {v for k, v in addr_data_mapping.items() if k.startswith('curTorque')}]
        self.publisher_.publish(joint_state)
    

def main(args=None):
    rclpy.init(args=args)

    state_publisher = StatePublisher()

    rclpy.spin(state_publisher)

    state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()