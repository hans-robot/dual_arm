#!/usr/bin/env python3
import socket
import json
import time
import threading
import sys
import os
import math
from typing import List, Optional

sys.path.append(os.path.join(os.path.dirname(__file__), '../../robot_control/scripts'))
from CPS import CPSClient

class Arm:
    """机械臂类，封装单个手臂的连接和数据读取功能"""
    def __init__(self, arm_type: str, boxid: int, ip: str, port: int, robot_id: int, sdk: CPSClient):
        self.arm_type = arm_type
        self.boxid = boxid
        self.ip = ip
        self.port = port
        self.sdk = sdk
        self.robot_id = robot_id
        self.connected = False
        
    def connect(self) -> bool:
        """连接机械臂"""
        try:
            is_connected = self.sdk.HRIF_IsConnected(self.boxid)
            if is_connected:
                print(f"{self.arm_type}已处于连接状态 (boxid: {self.boxid})")
                self.connected = True
                return True
            
            print(f"开始连接{self.arm_type} (boxid: {self.boxid}, ip: {self.ip}, port: {self.port})")
            ret = self.sdk.HRIF_Connect(self.boxid, self.ip, self.port)
            
            if ret != 0:
                print(f"{self.arm_type}连接失败 (错误码: {ret})")
                self.connected = False
                return False
            
            if not self.sdk.HRIF_IsConnected(self.boxid):
                print(f"{self.arm_type}连接返回成功，但HRIF_IsConnected验证失败")
                self.connected = False
                return False
            
            print(f"{self.arm_type}连接成功 (boxid: {self.boxid}, ip: {self.ip})")
            self.connected = True
            return True
        
        except Exception as e:
            print(f"{self.arm_type}连接过程异常: {str(e)}")
            self.connected = False
            return False
    
    # def read_joint_positions(self) -> Optional[List[float]]:
    #     """读取6个关节位置并转换为弧度"""
    #     if not self.connected:
    #         print(f"{self.arm_type}未连接，无法读取关节位置")
    #         return None
            
    #     try:
    #         result = []
    #         ret = self.sdk.HRIF_ReadActJointPos(self.boxid, self.robot_id, result)
            
    #         if ret != 0:
    #             print(f"{self.arm_type}读取关节位置失败，返回码: {ret}")
    #             return None
                
    #         if len(result) != 6:
    #             print(f"{self.arm_type}关节位置数量不正确，期望6个，实际{len(result)}个")
    #             return None
            
    #         # 将字符串转换为浮点数，然后从度转换为弧度
    #         try:
    #             positions_degrees = [float(pos) for pos in result]
    #             positions_radians = [math.radians(pos) for pos in positions_degrees]
    #             return positions_radians
    #         except ValueError as e:
    #             print(f"{self.arm_type}位置数据转换错误: {str(e)}")
    #             return None
                
    #     except Exception as e:
    #         print(f"{self.arm_type}读取关节位置异常: {str(e)}")
    #         return None
    
    def read_joint_positions(self) -> Optional[List[float]]:
        """读取6个关节位置并转换为弧度"""
        if not self.connected:
            print(f"{self.arm_type}未连接，无法读取关节位置")
            return None
            
        try:
            result = []
            ret = self.sdk.HRIF_ReadActJointPos(self.boxid, self.robot_id, result)
            
            if ret != 0:
                print(f"{self.arm_type}读取关节位置失败，返回码: {ret}")
                return None
                
            if len(result) != 6:
                print(f"{self.arm_type}关节位置数量不正确，期望6个，实际{len(result)}个")
                return None
            
            # 将字符串转换为浮点数，然后从度转换为弧度
            try:
                positions_degrees = [float(pos) for pos in result]
                positions_radians = [math.radians(pos) for pos in positions_degrees]
                
                # 对第一个关节（joint1）加180度（π弧度）
                positions_radians[0] += math.pi
                
                return positions_radians
            except ValueError as e:
                print(f"{self.arm_type}位置数据转换错误: {str(e)}")
                return None
                
        except Exception as e:
            print(f"{self.arm_type}读取关节位置异常: {str(e)}")
            return None
    
    def read_joint_velocities(self) -> Optional[List[float]]:
        """读取6个关节速度（保持度/秒单位）"""
        if not self.connected:
            print(f"{self.arm_type}未连接，无法读取关节速度")
            return None
            
        try:
            result = []
            ret = self.sdk.HRIF_ReadActJointVel(self.boxid, self.robot_id, result)
            
            if ret != 0:
                print(f"{self.arm_type}读取关节速度失败，返回码: {ret}")
                return None
                
            if len(result) != 6:
                print(f"{self.arm_type}关节速度数量不正确，期望6个，实际{len(result)}个")
                return None
            
            # 将字符串转换为浮点数（速度单位保持度/秒）
            try:
                velocities = [float(vel) for vel in result]
                return velocities
            except ValueError as e:
                print(f"{self.arm_type}速度数据转换错误: {str(e)}")
                return None
                
        except Exception as e:
            print(f"{self.arm_type}读取关节速度异常: {str(e)}")
            return None
    
    def group_stop(self) -> bool:
        """急停机械臂"""
        try:
            nRet = self.sdk.HRIF_GrpStop(self.boxid, self.robot_id)
            if nRet != 0:
                print(f"{self.arm_type}急停失败 (错误码: {nRet})")
                return False
            print(f"{self.arm_type}急停成功")
            return True
        except Exception as e:
            print(f"{self.arm_type}急停过程异常: {str(e)}")
            return False

class RobotArmClient:
    def __init__(self, server_host='192.168.1.8', server_port=9092):
        self.server_host = server_host
        self.server_port = server_port
        self.client_socket = None
        self.running = False
        
        # 机械臂连接参数
        self.arm_params = {
            'left_arm_type': 'left_arm',
            'left_boxid': 0,
            'left_ip': "192.168.1.20",
            'left_port': 10003,
            'left_robot_id': 0,
            
            'right_arm_type': 'right_arm',
            'right_boxid': 1,
            'right_ip': "192.168.1.30",
            'right_port': 10003,
            'right_robot_id': 0
        }
        
        # 初始化CPS客户端
        self.sdk = CPSClient()
        print("CPS client initialized")
        
        # 创建左右臂实例
        self.left_arm = Arm(
            arm_type=self.arm_params['left_arm_type'],
            boxid=self.arm_params['left_boxid'],
            ip=self.arm_params['left_ip'],
            port=self.arm_params['left_port'],
            robot_id=self.arm_params['left_robot_id'],
            sdk=self.sdk
        )
        
        self.right_arm = Arm(
            arm_type=self.arm_params['right_arm_type'],
            boxid=self.arm_params['right_boxid'],
            ip=self.arm_params['right_ip'],
            port=self.arm_params['right_port'],
            robot_id=self.arm_params['right_robot_id'],
            sdk=self.sdk
        )
        
        # 连接机械臂
        self.connect_arms()
        
        # 碰撞检测标志
        self.collision_detected = False
        
    def connect_arms(self):
        """连接机械臂"""
        left_connected = self.left_arm.connect()
        right_connected = self.right_arm.connect()
        
        if not left_connected and not right_connected:
            print("左右臂均连接失败，无法继续")
            return False
        elif not left_connected:
            print(f"{self.left_arm.arm_type}连接失败，将尝试仅使用{self.right_arm.arm_type}")
        elif not right_connected:
            print(f"{self.right_arm.arm_type}连接失败，将尝试仅使用{self.left_arm.arm_type}")
        
        return True
    
    def connect_to_server(self):
        """连接到碰撞检测服务器"""
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.server_host, self.server_port))
            print(f"Connected to collision detection server at {self.server_host}:{self.server_port}")
            return True
        except Exception as e:
            print(f"Failed to connect to server: {str(e)}")
            return False
    
    def send_joint_data(self):
        """发送关节数据到服务器"""
        try:
            # 读取左右臂关节位置（已经是弧度）
            left_pos = self.left_arm.read_joint_positions() if self.left_arm.connected else None
            right_pos = self.right_arm.read_joint_positions() if self.right_arm.connected else None
            
            # 读取左右臂关节速度（度/秒）
            left_vel = self.left_arm.read_joint_velocities() if self.left_arm.connected else None
            right_vel = self.right_arm.read_joint_velocities() if self.right_arm.connected else None
            
            # 检查数据完整性
            if left_pos is None or right_pos is None:
                print("关节位置数据不完整，跳过本次发送")
                return False
                
            if left_vel is None or right_vel is None:
                print("关节速度数据不完整，跳过本次发送")
                return False
            
            # 拼接左右臂数据（左臂6个 + 右臂6个 = 12个）
            # 位置已经是弧度，速度保持度/秒
            all_pos = left_pos + right_pos
            all_vel = left_vel + right_vel
            
            # 准备发送的数据
            data = {
                'type': 'joint_data',
                'joint_positions': all_pos,  # 弧度
                'joint_velocities': all_vel,  # 度/秒
                'units': {
                    'positions': 'radians',
                    'velocities': 'degrees_per_second'
                }
            }
            
            # 发送数据
            self.client_socket.sendall(json.dumps(data).encode('utf-8'))
            
            # 接收响应
            response = self.client_socket.recv(4096)
            if response:
                response_data = json.loads(response.decode('utf-8'))
                if response_data.get('type') == 'collision_result':
                    result = response_data['result']
                    self.handle_collision_result(result)
            
            return True
            
        except Exception as e:
            print(f"Error sending joint data: {str(e)}")
            return False
    
    def handle_collision_result(self, result: dict):
        """处理碰撞检测结果"""
        if result.get('collision_detected', False):
            print(f"碰撞检测到! 碰撞对: {result.get('colliding_pairs', [])}, 最小距离: {result.get('min_distance', 0.0)}")
            self.collision_detected = True
            self.emergency_stop()
        else:
            self.collision_detected = False
    
    def emergency_stop(self):
        """执行急停操作"""
        print("开始执行双机械臂急停操作...")
        
        # 急停左臂
        if self.left_arm.connected:
            if self.left_arm.group_stop():
                print(f"{self.left_arm.arm_type}急停成功")
            else:
                print(f"{self.left_arm.arm_type}急停失败")
        
        # 急停右臂
        if self.right_arm.connected:
            if self.right_arm.group_stop():
                print(f"{self.right_arm.arm_type}急停成功")
            else:
                print(f"{self.right_arm.arm_type}急停失败")
        
        print("双机械臂急停操作完成")
    
    def send_heartbeat(self):
        """发送心跳包"""
        try:
            data = {'type': 'ping'}
            self.client_socket.sendall(json.dumps(data).encode('utf-8'))
            
            # 接收响应
            response = self.client_socket.recv(4096)
            if response:
                response_data = json.loads(response.decode('utf-8'))
                if response_data.get('type') == 'pong':
                    print("Heartbeat received from server")
            
        except Exception as e:
            print(f"Heartbeat error: {str(e)}")
            self.reconnect()
    
    def reconnect(self):
        """重新连接服务器"""
        self.disconnect()
        time.sleep(1)
        self.connect_to_server()
    
    def disconnect(self):
        """断开连接"""
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None
    
    def start(self, update_rate=10.0):
        """启动客户端"""
        if not self.connect_to_server():
            return
        
        self.running = True
        last_heartbeat = time.time()
        
        try:
            while self.running:
                # 发送关节数据
                self.send_joint_data()
                
                # 定期发送心跳
                if time.time() - last_heartbeat > 5.0:
                    self.send_heartbeat()
                    last_heartbeat = time.time()
                
                # 控制更新频率
                time.sleep(1.0 / update_rate)
                
        except KeyboardInterrupt:
            print("\nShutting down client...")
        except Exception as e:
            print(f"Client error: {str(e)}")
        finally:
            self.stop()
    
    def stop(self):
        """停止客户端"""
        self.running = False
        self.disconnect()
        print("Robot arm client stopped")

if __name__ == "__main__":
    client = RobotArmClient()
    try:
        client.start()
    except KeyboardInterrupt:
        print("\nShutting down...")
        client.stop()