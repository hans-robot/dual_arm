#!/usr/bin/env python3
import array
import traceback
import sys
import os
import numpy as np
import math
import json
import yaml
from typing import List, Dict, Any, Optional
import socket
import threading
import time

current_dir = os.path.dirname(os.path.abspath(__file__))  
sys.path.append(current_dir)
import dual_arm_collision as dac

from readDualArmSelfCollisionModelFromJson import (
    read_dual_arm_self_collision_model_from_json,
    DualArmColliderConfiguration,
    RobotType,
    LozengeModelCo,
    CapsuleModel,
    BallModel
)

class CollisionDetectionServer:
    def __init__(self, host='0.0.0.0', port=9092):
        self.host = host
        self.port = port
        self.server_socket = None
        self.running = False
        
        # 初始化配置和碰撞检测相关变量
        self.json_config_path = "/usr/local/RobotOS/home/RobotOS/dual_arm_server/dual_arm_collision.json"
        self.tool_config_path = "/usr/local/RobotOS/home/RobotOS/dual_arm_server/dualarm_tool_collision.json"
        
        self.config = None
        self.init_params = None
        self.joint_pos = dac.dualarm_tagAXISPOS_REF()  # 12个关节位置
        self.joint_vel = dac.dualarm_tagAXISPOS_REF()  # 12个关节速度
        
        # 碰撞检测结果
        self.collision_detected = False
        self.colliding_pairs = []
        self.min_distance = 0.0
        
        # 工具碰撞模型配置
        self.tool_config = self.load_tool_config()
        
        # 初始化流程
        self.initialize()
        
    def load_tool_config(self) -> Dict[str, Any]:
        """从JSON文件读取工具碰撞模型配置"""
        try:
            if not os.path.exists(self.tool_config_path):
                print(f"工具碰撞模型配置文件不存在: {self.tool_config_path}，将使用默认值")
                return {}
            
            with open(self.tool_config_path, 'r') as f:
                config = json.load(f)
            
            print(f"成功从{self.tool_config_path}加载工具碰撞模型配置")
            return config
            
        except Exception as e:
            print(f"读取工具碰撞模型配置文件失败: {str(e)}，将使用默认值")
            return {}
    
    def set_tool_collision_models(self) -> bool:
        """设置工具碰撞模型"""
        try:
            print("Setting tool collision models...")
            
            # 设置左臂工具碰撞模型
            if 'L_tool' in self.tool_config:
                l_tool = self.tool_config['L_tool']
                tool_index = l_tool.get('L_tool_index', 6)
                start_point = l_tool.get('start', [0.0, 0.0, 0.0])
                end_point = l_tool.get('end', [0.0, 0.0, 0.0])
                radius = l_tool.get('radius', 0.02)
                
                dac.set_toolcollisionmodel(tool_index, start_point, end_point, radius)
                print(f"Left arm tool collision model set (joint {tool_index}): "
                      f"start={start_point}, end={end_point}, radius={radius}")
            else:
                # 使用默认值
                dac.set_toolcollisionmodel(6, [0.0]*3, [0.0]*3, 0.02)
                print("Left arm tool collision model set with default values (joint 6)")
            
            # 设置右臂工具碰撞模型
            if 'R_tool' in self.tool_config:
                r_tool = self.tool_config['R_tool']
                tool_index = r_tool.get('R_tool_index', 16)
                start_point = r_tool.get('start', [0.0, 0.0, 0.0])
                end_point = r_tool.get('end', [0.0, 0.0, 0.0])
                radius = r_tool.get('radius', 0.02)
                
                dac.set_toolcollisionmodel(tool_index, start_point, end_point, radius)
                print(f"Right arm tool collision model set (joint {tool_index}): "
                      f"start={start_point}, end={end_point}, radius={radius}")
            else:
                # 使用默认值
                dac.set_toolcollisionmodel(16, [0.0]*3, [0.0]*3, 0.02)
                print("Right arm tool collision model set with default values (joint 16)")
            
            return True
            
        except Exception as e:
            print(f"Error setting tool collision models: {str(e)}")
            traceback.print_exc()
            return False
    
    def add_collision_pairs(self) -> bool:
        """添加碰撞对"""
        try:
            print("Adding collision pairs...")
            
            # 定义要添加的碰撞对列表
            collision_pairs = [
                (-1, 2),   # 平台与左臂关节2
                (-1, 3),   # 平台与左臂关节3
                (-1, 4),   # 平台与左臂关节4
                (-1, 5),   # 平台与左臂关节5
                (-1, 6),   # 平台与左臂关节6（工具）
                (-1, 12),  # 平台与右臂关节12
                (-1, 13),  # 平台与右臂关节13
                (-1, 14),  # 平台与右臂关节14
                (-1, 15),  # 平台与右臂关节15
                (-1, 16),   # 平台与右臂关节16（工具）
                (1, 16), 
                (6, 16), 
                (6, 11)
            ]
            
            # 添加所有碰撞对
            for id1, id2 in collision_pairs:
                dac.set_collision_Pair(id1, id2)
                print(f"Added collision pair: ({id1}, {id2})")
            
            return True
            
        except Exception as e:
            print(f"Error adding collision pairs: {str(e)}")
            traceback.print_exc()
            return False
    
    def check_collision_pairs(self) -> bool:
        """查询碰撞对"""
        try:
            print("Checking collision pairs...")
            dac.check_collision_Pair()
            print("Collision pairs checked successfully")
            return True
            
        except Exception as e:
            print(f"Error checking collision pairs: {str(e)}")
            traceback.print_exc()
            return False
    
    def load_config(self) -> bool:
        """从JSON文件加载配置"""
        try:
            print(f"Loading configuration from {self.json_config_path}")
            self.config = DualArmColliderConfiguration()
            read_dual_arm_self_collision_model_from_json(self.json_config_path, self.config)
            return True
        except Exception as e:
            print(f"Error loading config: {str(e)}")
            return False
    
    def convert_config(self) -> bool:
        """将配置对象转换为init_dual_arm函数所需的参数"""
        try:
            # 构建棱体模型参数（13个值：ref2local_frame(6) + geometry(4) + offset(3)）
            def build_lozenge_params(lozenge: LozengeModelCo) -> List[float]:
                params = []
                # 前6个值：参考系转换参数
                params.extend(lozenge.ref2local_frame[:6])
                params += [0.0] * (6 - len(lozenge.ref2local_frame[:6]))
                # 中间4个值：几何参数
                params.extend(lozenge.geometry[:4])
                params += [0.0] * (4 - len(lozenge.geometry[:4]))
                # 最后3个值：偏移量
                params.extend(lozenge.offset[:3])
                params += [0.0] * (3 - len(lozenge.offset[:3]))
                return params[:13]  # 确保总长度为13

            # 构建胶囊体模型参数（7个值：start(3) + end(3) + radius(1)）
            def build_capsule_params(capsule: CapsuleModel) -> List[float]:
                params = []
                # 前3个值：起点坐标
                params.extend(capsule.start[:3])
                params += [0.0] * (3 - len(capsule.start[:3]))
                # 中间3个值：终点坐标
                params.extend(capsule.end[:3])
                params += [0.0] * (3 - len(capsule.end[:3]))
                # 最后1个值：半径
                params.append(capsule.radius)
                return params[:7]  # 确保总长度为7

            # 构建腕部球体参数（4个值：offset(3) + radius(1)）
            def build_wrist_params(wrist: BallModel) -> List[float]:
                params = []
                # 前3个值：偏移量
                params.extend(wrist.offset[:3])
                params += [0.0] * (3 - len(wrist.offset[:3]))
                # 最后1个值：半径
                params.append(wrist.radius)
                return params[:4]  # 确保总长度为4

            # 生成各部分参数
            platform_geo = build_lozenge_params(self.config.platform)
            head_geo = build_lozenge_params(self.config.head)
            truck_geo = build_lozenge_params(self.config.truck)
            
            base_geo = build_capsule_params(self.config.L_base)
            lower_arm_geo = build_capsule_params(self.config.L_lowerArm)
            elbow_geo = build_capsule_params(self.config.L_elbow)
            upper_arm_geo = build_capsule_params(self.config.L_upperArm)
            
            wrist_geo = build_wrist_params(self.config.L_wrist)

            # 确保DH参数为4个值
            dh_params = self.config.dh[:4]
            dh_params += [0.0] * (4 - len(dh_params))

            self.init_params = (
                self.config.robType,            # robType
                dh_params,                      # dh参数
                platform_geo,                   # platformGeometry
                head_geo,                       # headGeometry
                truck_geo,                      # truckGeometry
                base_geo,                       # baseGeometry (左臂)
                lower_arm_geo,                  # lowerArmGeometry (左臂)
                elbow_geo,                      # elbowGeometry (左臂)
                upper_arm_geo,                  # upperArmGeometry (左臂)
                wrist_geo,                      # wristGeometry (左臂)
            )
            
            return True
            
        except Exception as e:
            print(f"Error converting config: {str(e)}")
            return False
    
    def init_dual_arm(self) -> bool:
        """初始化双机械臂模型"""
        try:
            # 使用默认关节位置
            left_pos = [math.radians(angle) for angle in [0, 0, 0, 0, 0, 0]]
            right_pos = [math.radians(angle) for angle in [-90, 50, 140, 0, 0, 0]]
            
            # 拼接左右臂关节位置（左臂6个 + 右臂6个 = 12个）
            joint_values = left_pos + right_pos
            
            # 设置初始关节位置
            for i, value in enumerate(joint_values):
                setattr(self.joint_pos, f"a{i}", value)
            
            # 初始化双机械臂
            dac.init_dual_arm(
                self.init_params[0],  # robType
                self.init_params[1],  # dh
                self.init_params[2],  # platformGeometry
                self.init_params[3],  # headGeometry
                self.init_params[4],  # truckGeometry
                self.init_params[5],  # baseGeometry (左臂)
                self.init_params[6],  # lowerArmGeometry (左臂)
                self.init_params[7],  # elbowGeometry (左臂)
                self.init_params[8],  # upperArmGeometry (左臂)
                self.init_params[9],  # wristGeometry (左臂)
                self.joint_pos        # jointPos
            )
            
            return True
            
        except Exception as e:
            print(f"初始化双机械臂模型错误: {str(e)}")
            traceback.print_exc()
            return False
    
    def initialize(self):
        """初始化配置和双机械臂模型"""
        try:
            print("开始初始化双机械臂碰撞检测系统...")
                        
            # 加载配置
            if not self.load_config():
                print("Failed to load configuration, exiting...")
                return False
                
            # 转换配置参数
            if not self.convert_config():
                print("Failed to convert configuration, exiting...")
                return False
                
            # 初始化双机械臂模型
            if not self.init_dual_arm():
                print("Failed to initialize dual arm model, exiting...")
                return False

            # 设置工具碰撞模型
            if not self.set_tool_collision_models():
                print("Failed to set tool collision models, exiting...")
                return False
                
            # 添加碰撞对
            if not self.add_collision_pairs():
                print("Failed to add collision pairs, exiting...")
                return False
                
            # 查询碰撞对
            if not self.check_collision_pairs():
                print("Collision pair check failed, but continuing...")
            
            print("双机械臂碰撞检测系统初始化成功")
            return True
            
        except Exception as e:
            print(f"初始化错误: {str(e)}")
            traceback.print_exc()
            return False
    
    def update_joints(self, joint_positions: List[float], joint_velocities: List[float]):
        """更新关节状态"""
        try:
            # 更新关节位置和速度
            for i in range(12):
                setattr(self.joint_pos, f"a{i}", joint_positions[i])
                setattr(self.joint_vel, f"a{i}", joint_velocities[i])
            
            dac.update_joints(self.joint_pos, self.joint_vel)
        except Exception as e:
            print(f"更新关节状态错误: {str(e)}")
    
    def check_collision(self) -> dict:
        """执行碰撞检测并返回结果"""
        try:
            result, collider_pair_list, distance = dac.check_collision()
            
            self.collision_detected = result
            self.colliding_pairs = collider_pair_list
            self.min_distance = distance
            
            print(f"碰撞检测结果: {result}, 碰撞对: {collider_pair_list}, 距离: {distance}")
            
            return {
                'collision_detected': result,
                'colliding_pairs': collider_pair_list,
                'min_distance': distance
            }
            
        except Exception as e:
            print(f"碰撞检测错误: {str(e)}")
            traceback.print_exc()
            return {
                'collision_detected': False,
                'colliding_pairs': [],
                'min_distance': 0.0,
                'error': str(e)
            }
    
    def handle_client(self, conn, addr):
        """处理客户端连接"""
        print(f"Connected by {addr}")
        try:
            while self.running:
                # 接收客户端数据
                data = conn.recv(4096)
                if not data:
                    break
                
                try:
                    # 解析JSON数据
                    received_data = json.loads(data.decode('utf-8'))
                    print(f"Received from {addr}: {received_data}")
                    
                    # 检查消息类型
                    if received_data.get('type') == 'joint_data':
                        # 更新关节状态并检测碰撞
                        joint_positions = received_data['joint_positions']
                        joint_velocities = received_data['joint_velocities']
                        
                        self.update_joints(joint_positions, joint_velocities)
                        collision_result = self.check_collision()
                        
                        # 发送碰撞检测结果
                        response = {
                            'type': 'collision_result',
                            'result': collision_result
                        }
                        conn.sendall(json.dumps(response).encode('utf-8'))
                        
                    elif received_data.get('type') == 'ping':
                        # 心跳检测
                        response = {'type': 'pong'}
                        conn.sendall(json.dumps(response).encode('utf-8'))
                        
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {str(e)}")
                    response = {'error': 'Invalid JSON format'}
                    conn.sendall(json.dumps(response).encode('utf-8'))
                
        except Exception as e:
            print(f"Error handling client {addr}: {str(e)}")
        finally:
            conn.close()
            print(f"Connection with {addr} closed")
    
    def start(self):
        """启动服务器"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            self.running = True
            
            print(f"Collision detection server started on {self.host}:{self.port}")
            
            while self.running:
                try:
                    conn, addr = self.server_socket.accept()
                    client_thread = threading.Thread(target=self.handle_client, args=(conn, addr))
                    client_thread.daemon = True
                    client_thread.start()
                except Exception as e:
                    if self.running:
                        print(f"Error accepting connection: {str(e)}")
            
        except Exception as e:
            print(f"Server error: {str(e)}")
            traceback.print_exc()
        finally:
            self.stop()
    
    def stop(self):
        """停止服务器"""
        self.running = False
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        print("Collision detection server stopped")

if __name__ == "__main__":
    server = CollisionDetectionServer()
    try:
        server.start()
    except KeyboardInterrupt:
        print("\nShutting down server...")
        server.stop()