import time, os, sys, signal
import socket
import json

from threading import Thread

class KORAS_State(Thread):
    # IP_ADDRESS = "192.168.0.12"  # TCP 서버의 IP 주소
    IP_ADDRESS = "BOOK-E1I23S8BLS.local"  # TCP 서버의 IP 주소
    # IP_ADDRESS = "10.255.255.254"
    # IP_ADDRESS = "172.20.135.62"
    # IP_ADDRESS = "127.0.0.1"  # TCP 서버의 IP 주소
    PORT = 12123  # TCP 서버의 포트
    refreshRate = 0.1  # 데이터 수신 주기
    connection = False  # 서버 연결 상태
    finger_pos = 0
    motor_cur = 0
    motor_pos = 0
    motor_vel = 0

    def __init__(self, port=12123):
        super().__init__()
        self.PORT = port

    def run(self):
        try:
            self.tcp_connect(self.IP_ADDRESS, self.PORT)
        except Exception as e:
            print(e)
            self.connection = False
        else:
            print('>> KorasGripper socket is connected.')
            self.connection = True
        
        while True:
            if self.connection:
                self.status_recv()
                time.sleep(0.2)  # 주기적으로 서버 데이터 수신

    def start_client(self):
        self.start()

    def tcp_connect(self, ip, port):
        # TCP 서버에 연결
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((ip, port))

    def tcp_close(self):
        self.connection = False
        self.client_socket.close()
        print('>> KorasGripper socket is disconneced.')

    def status_recv(self):
        try:
            # 서버로부터 데이터 수신
            response = self.client_socket.recv(1024).decode("utf-8")
            
            # 수신한 데이터가 비어 있는지 확인
            if not response.strip():
                # print("Received empty response from server.")
                return

            # 디버깅: 원본 응답 출력
            # print("Raw response from server:", response)

            # JSON 응답을 파싱
            try:
                temp = json.loads(response)
            except json.JSONDecodeError as e:
                # print("Received data is not valid JSON:", e)
                return
            
            # 'recv_data'가 있는지 확인하고 처리
            recv_data = temp.get("recv_data")
            if recv_data is None:
                # print(f"No 'recv_data' found in server response: {temp}")
                return
            
            # print(f"Received Modbus data: {recv_data}")

            # 'recv_data'에 있는 값을 읽어서 클래스 속성 업데이트
            self.finger_pos = recv_data.get("finger_pos", 0)
            self.motor_cur = recv_data.get("motor_cur", 0)
            self.motor_vel = recv_data.get("motor_vel", 0)
            self.motor_pos = recv_data.get("motor_pos", 0)

            # print(f"Finger Position: {self.finger_pos}, Motor Current: {self.motor_cur}, Motor Velocity: {self.motor_vel}, Motor Position: {self.motor_pos}")

        except socket.error as e:
            print("Socket error:", e)
        except Exception as e:
            print(e)

    def initialize(self):
        self.grp_command_send({"command": 101}, "Initialize Send")

    def open(self):
        self.grp_command_send({"command": 102}, "Open Send")

    def close(self):
        self.grp_command_send({"command": 103}, "Close Send")

    def vacuum_on(self):
        self.grp_command_send({"command": 106, "value_1": 0}, "Vacuum On Send")

    def vacuum_off(self):
        self.grp_command_send({"command": 107, "value_1": 0}, "Vacuum Off Send")

    def finger_pos_set(self, pos):
        if pos > 1000 or pos < 0:
            print("[Gripper]: ***Finger Pos value error!***")
            return
        if type(pos) == float:
            pos = round(pos)
        self.grp_command_send({"command": 104, "value_1": pos}, "Finger pos set Send")
        
    def motor_pos_set(self, pos):
        if pos > 32768 or pos < -32767:
            print("[Gripper]: ***Motor Pos value error!***")
            return
        if type(pos) == float:
            pos = round(pos)
        self.grp_command_send({"command": 5, "value_1": pos, "value_2": 0}, "Motor pos set Send")
        
    def motor_pos_relative_set(self, pos):
        if pos > 32768 or pos < -32767:
            print("[Gripper]: ***Motor Pos value error!***")
            return
        if type(pos) == float:
            pos = round(pos)
        current_pos = self.motor_pos # 30000
        target_pos = current_pos + pos # 5000
        self.grp_command_send({"command": 5, "value_1": target_pos, "value_2": 100}, "Motor pos set Send")

    def grp_command_send(self, command, ref=''):
        if self.connection:
            if type(command) == dict:
                request = json.dumps(command)
                self.client_socket.send(request.encode("utf-8"))
                print("[Gripper]:", ref)
            else:
                print("Gripper command type Error")
        else:
            print("[Gripper] TCP Socket disconnected")

    def motor_stop(self):
        self.grp_command_send({"command": 2}, "Motor Stop Send")

def signal_handler(signal, frame):
    os._exit(-1)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    gripper = KORAS_State()
    gripper.start_client()
    
    while True:
        if gripper.connection:
            break

    print('Gripper init')
    gripper.initialize()
    gripper.tcp_close()
