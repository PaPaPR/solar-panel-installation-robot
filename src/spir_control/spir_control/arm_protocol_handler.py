import socket
import queue
import json
import threading
import time
import math

from spir_control import quaternion_euler_conversions

class ArmProtocolHandler():
    def __init__(self, hostname, hostport, buff_size = 4096):
        self.hostname = hostname
        self.hostport = hostport
        self.buff_size = buff_size

        self.sock_lock = threading.Lock()

        self.send_queue = queue.Queue() # str need to send
        self.command_dict = {}
        self.command_dict['joint_state_query'] = '{"dsID":"www.hc-system.com.RemoteMonitor","reqType":"query","packID":"0","packID":"0",'+ \
                    '"queryAddr":["axis-0","axis-1","axis-2","axis-3","axis-4","axis-5","curTorque-0",'+ \
                    '"curTorque-1","curTorque-2","curTorque-3","curTorque-4","curTorque-5",'+ \
                    '"curSpeed-0","curSpeed-1","curSpeed-2","curSpeed-3","curSpeed-4","curSpeed-5",'+ \
                    '"world-0","world-1","world-2","world-3","world-4","world-5"]}'
        self.command_dict['joint_state_cmd'] = '{"dsID":"www.hc-system.com.cam", "reqType":"AddPoints", ' + \
                    '"dsData":[{"camID":"0", "data":[{"ModelID":"0","Similarity":"0","Color":"0","Rel":"0"}]}]}'
        self.command_dict['photo_reply']  = '{"dsID":"www.hc-system.com.cam", "reqType":"photo","camID":0,"ret":1}'
        self.msg_queue = {}
        self.msg_queue['joint_state'] = queue.Queue()
        self.msg_queue['photo'] = queue.Queue()
        self.msg_queue['add_points'] = queue.Queue()

        self.initialized = False

        self.init()

    def init(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.hostname, self.hostport))
        except Exception as e:
            return
        self.initialized = True
        self.recvThread = threading.Thread(target = self.recv)
        self.recvThread.start()
        self.sendThread = threading.Thread(target = self.send)
        self.sendThread.start()

    def isAvailable(self) -> bool:
        return self.initialized

    def recv(self):
        while self.initialized:
            try:
                chunk = self.sock.recv(self.buff_size)
                if not chunk:
                    continue
                if len(chunk) % self.buff_size == 0:
                    data += chunk
                else:
                    data = chunk
                    try:
                        json_data = json.loads(data.decode('ascii'))
                        data = b''
                        # Handle request of target position
                        if json_data['dsID'] == 'www.hc-system.com.cam':
                            if json_data['reqType'] == 'photo':
                                json_data['ret'] = 1
                                # self.command_dict['photo_reply'] = json.dumps(json_data, ensure_ascii=True)
                                self.msg_queue['photo'].put(True)
                        # Handle information of robot state
                        elif json_data['dsID'] == 'www.hc-system.com.RemoteMonitor':
                            joint_state = self.resolve_arm_state(json_data)
                            self.msg_queue['joint_state'].put(joint_state)
                    except json.JSONDecodeError:
                        continue
            except socket.error as e:
                self.initialized = False
                continue

    # 数据单项发送
    def send(self):
        while self.initialized:
            send_data = self.send_queue.get()
            self.send_queue.task_done()
            try:
                self.sock.sendall(bytes(send_data, 'ascii'))
            except socket.error as e:
                self.initialized = False
            time.sleep(0.02)

    # 命令
    # todo:超时以及出错清空，queue数量限制
    def command(self, type:str, value=None):
        if type == 'joint_state':
            self.send_queue.put(self.command_dict['joint_state_query'])
            try:
                joint_state = self.msg_queue['joint_state'].get(timeout=1)
                self.msg_queue['joint_state'].task_done()
            except queue.Empty as e:
                return
            return joint_state
        if type == 'photo':
            photo_cmd = self.msg_queue['photo'].get()
            self.msg_queue['photo'].task_done()
            if photo_cmd == True:
                return True
        if type == 'photo_reply':
            self.send_queue.put(self.command_dict['photo_reply'])
        if type == 'add_points':
            # todo:指令发送成功反馈确认以及失败重发送
            translation, rotation = value # rotation 为四元数组成的 list
            rpy = quaternion_euler_conversions.euler_from_quaternion(rotation)
            cmd = self.resolve_add_points(translation, rpy)
            self.send_queue.put(cmd)

    def resolve_add_points(self, translation:list, rotation:list):
        cmd_str = self.command_dict['joint_state_cmd']
        json_data = json.loads(cmd_str)
        json_data['dsData'][0]['data'][0]['X'] = str(translation[0] * 1000.)
        json_data['dsData'][0]['data'][0]['Y'] = str(translation[1] * 1000.)
        json_data['dsData'][0]['data'][0]['Z'] = str(translation[2] * 1000.)
        json_data['dsData'][0]['data'][0]['U'] = str(rotation[0] * 180. / math.pi)
        json_data['dsData'][0]['data'][0]['V'] = str(rotation[1] * 180. / math.pi)
        json_data['dsData'][0]['data'][0]['Angel'] = str(rotation[2] * 180. / math.pi)
        cmd_str = json.dumps(json_data, ensure_ascii=True)
        return cmd_str

    def resolve_arm_state(self, json_data):
        query_addr = json_data['queryAddr']
        query_data = json_data['queryData']
        addr_data_mapping = dict(zip(query_addr, query_data))
        state = {}
        state['name'] = [k for k, v in addr_data_mapping.items() if k.startswith('axis')]
        position = [(float(k) / 180. * math.pi) for k in [v for k, v in addr_data_mapping.items() if k.startswith('axis')]]
        state['position'] = position[0:2] + [-x for x in position[2:6]]
        state['velocity'] = [float(k) for k in [v for k, v in addr_data_mapping.items() if k.startswith('curSpeed')]]
        state['effort'] = [float(k) for k in [v for k, v in addr_data_mapping.items() if k.startswith('curTorque')]]
        world = [float(k) for k in [v for k, v in addr_data_mapping.items() if k.startswith('world')]]
        q = quaternion_euler_conversions.quaternion_from_euler([r / 180. * math.pi for r in world[3:6]])
        state['world'] = [t / 1000. for t in world[0:3]] + q
        return state