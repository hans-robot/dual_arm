import json
from typing import List, Dict, Any

class RobotType:
    """机器人类型枚举"""
    Elfin = 0       # Elfin构型
    UR = 1          # UR构型
    DualArm = 2     # 双臂构型


class CollisionModelType:
    """碰撞模型类型枚举"""
    Ball = 1        # 球体模型
    Capsule = 2     # 胶囊体模型
    Lozenge = 3     # 棱体模型
    Plane = 4       # 平面模型


class CollisionModelIndex:
    """碰撞模型索引枚举"""
    # 工作台相关
    WorkBench2 = -20
    WorkBench1 = -21
    WorkBench0 = -19
    # 主体结构
    Truck = 0
    PlatForm = 22
    # 左臂部件
    L_Base = 1
    L_LowerArm = 2
    L_Elbow = 3
    L_UpperArm = 4
    L_Wrist = 5
    L_Tool1 = 6
    L_Tool2 = 7
    # 右臂部件
    R_Base = 11
    R_LowerArm = 12
    R_Elbow = 13
    R_UpperArm = 14
    R_Wrist = 15
    R_Tool1 = 16
    R_Tool2 = 17


# 基础碰撞模型结构体（对应C++的struct）
class BallModel:
    def __init__(self):
        self.offset: List[float] = []  # 三维偏移向量 [x, y, z]
        self.radius: float = 0.0       # 球体半径
        self.type: str = ""            # 模型类型标识


class CapsuleModel:
    def __init__(self):
        self.start: List[float] = []   # 起点坐标 [x1, y1, z1]
        self.end: List[float] = []     # 终点坐标 [x2, y2, z2]
        self.radius: float = 0.0       # 胶囊体半径
        self.type: str = ""            # 模型类型标识


class LozengeModelCo:
    def __init__(self):
        self.ref2local_frame: List[float] = []  # 参考系到局部坐标系的转换参数
        self.offset: List[float] = []          # 三维偏移向量 [x, y, z]
        self.geometry: List[float] = []        # 几何参数 [长, 宽, 高, 半径]
        self.type: str = ""                    # 模型类型标识


class DualArmColliderConfiguration:
    """双臂机器人碰撞配置结构体"""
    def __init__(self):
        self.dh: List[float] = []                  # DH参数（运动学建模）
        self.robType: int = RobotType.DualArm      # 机器人类型（默认为双臂）
        # 主体结构（棱体模型）
        self.platform: LozengeModelCo = LozengeModelCo()  # 平台
        self.truck: LozengeModelCo = LozengeModelCo()      # 躯干
        self.head: LozengeModelCo = LozengeModelCo()       # 头部
        # 左臂部件（胶囊体/球体模型）
        self.L_base: CapsuleModel = CapsuleModel()         # 左臂底座
        self.L_lowerArm: CapsuleModel = CapsuleModel()     # 左臂下臂
        self.L_elbow: CapsuleModel = CapsuleModel()        # 左臂肘部
        self.L_upperArm: CapsuleModel = CapsuleModel()     # 左臂上臂
        self.L_wrist: BallModel = BallModel()              # 左臂腕部（球体）
        # 右臂部件（胶囊体/球体模型）
        self.R_base: CapsuleModel = CapsuleModel()         # 右臂底座
        self.R_lowerArm: CapsuleModel = CapsuleModel()     # 右臂下臂
        self.R_elbow: CapsuleModel = CapsuleModel()        # 右臂肘部
        self.R_upperArm: CapsuleModel = CapsuleModel()     # 右臂上臂
        self.R_wrist: BallModel = BallModel()              # 右臂腕部（球体）


def read_dual_arm_self_collision_model_from_json(file_path: str, config: DualArmColliderConfiguration) -> None:
    """读取JSON文件并填充双臂机器人碰撞配置"""
    try:
        with open(file_path, 'r') as f:
            json_data = json.load(f)

        # 解析机器人类型和DH参数
        rob_type_str = json_data.get("RobType", "")
        if "dual_arm" in rob_type_str:
            print("检测到双臂机器人类型")
            config.robType = RobotType.DualArm
            # 解析DH参数
            dh_data = json_data.get("DH", {})
            config.dh = [
                dh_data.get("d1", 0.0),
                dh_data.get("d4", 0.0),
                dh_data.get("d6", 0.0),
                dh_data.get("a2", 0.0)
            ]

        # 解析主体棱体模型（platform/truck/head）
        # 平台（LozengeModel_co）
        platform_data = json_data.get("platform", {})
        config.platform.ref2local_frame = platform_data.get("ref2local_frame", [])
        config.platform.geometry = platform_data.get("geometry", [])
        config.platform.offset = platform_data.get("offset", [])
        config.platform.type = platform_data.get("type", "")

        # 躯干（LozengeModel_co）
        truck_data = json_data.get("truck", {})
        config.truck.ref2local_frame = truck_data.get("ref2local_frame", [])
        config.truck.geometry = truck_data.get("geometry", [])
        config.truck.offset = truck_data.get("offset", [])
        config.truck.type = truck_data.get("type", "")

        # 头部（LozengeModel_co）
        head_data = json_data.get("head", {})
        config.head.ref2local_frame = head_data.get("ref2local_frame", [])
        config.head.geometry = head_data.get("geometry", [])
        config.head.offset = head_data.get("offset", [])
        config.head.type = head_data.get("type", "")

        # 解析左臂胶囊体/球体模型
        # 左臂底座（CapsuleModel）
        l_base_data = json_data.get("base", {})  # 对应JSON中的"l_base"字段
        config.L_base.start = l_base_data.get("start", [])
        config.L_base.end = l_base_data.get("end", [])
        config.L_base.radius = l_base_data.get("radius", 0.0)
        config.L_base.type = l_base_data.get("type", "")

        # 左臂下臂（CapsuleModel）
        l_lower_arm_data = json_data.get("l_lowerArm", {})
        config.L_lowerArm.start = l_lower_arm_data.get("start", [])
        config.L_lowerArm.end = l_lower_arm_data.get("end", [])
        config.L_lowerArm.radius = l_lower_arm_data.get("radius", 0.0)
        config.L_lowerArm.type = l_lower_arm_data.get("type", "")

        # 左臂肘部（CapsuleModel）
        l_elbow_data = json_data.get("l_elbow", {})
        config.L_elbow.start = l_elbow_data.get("start", [])
        config.L_elbow.end = l_elbow_data.get("end", [])
        config.L_elbow.radius = l_elbow_data.get("radius", 0.0)
        config.L_elbow.type = l_elbow_data.get("type", "")

        # 左臂上臂（CapsuleModel）
        l_upper_arm_data = json_data.get("l_upperArm", {})
        config.L_upperArm.start = l_upper_arm_data.get("start", [])
        config.L_upperArm.end = l_upper_arm_data.get("end", [])
        config.L_upperArm.radius = l_upper_arm_data.get("radius", 0.0)
        config.L_upperArm.type = l_upper_arm_data.get("type", "")

        # 左臂腕部（BallModel）
        l_wrist_data = json_data.get("l_wrist", {})
        config.L_wrist.offset = l_wrist_data.get("offset", [])
        config.L_wrist.radius = l_wrist_data.get("radius", 0.0)
        config.L_wrist.type = l_wrist_data.get("type", "")

        # 解析右臂胶囊体/球体模型
        # 右臂底座（CapsuleModel）
        r_base_data = json_data.get("base", {})
        config.R_base.start = r_base_data.get("start", [])
        config.R_base.end = r_base_data.get("end", [])
        config.R_base.radius = r_base_data.get("radius", 0.0)
        config.R_base.type = r_base_data.get("type", "")

        # 右臂下臂（CapsuleModel）
        r_lower_arm_data = json_data.get("r_lowerArm", {})
        config.R_lowerArm.start = r_lower_arm_data.get("start", [])
        config.R_lowerArm.end = r_lower_arm_data.get("end", [])
        config.R_lowerArm.radius = r_lower_arm_data.get("radius", 0.0)
        config.R_lowerArm.type = r_lower_arm_data.get("type", "")

        # 右臂肘部（CapsuleModel）
        r_elbow_data = json_data.get("r_elbow", {})
        config.R_elbow.start = r_elbow_data.get("start", [])
        config.R_elbow.end = r_elbow_data.get("end", [])
        config.R_elbow.radius = r_elbow_data.get("radius", 0.0)
        config.R_elbow.type = r_elbow_data.get("type", "")

        # 右臂上臂（CapsuleModel）
        r_upper_arm_data = json_data.get("r_upperArm", {})
        config.R_upperArm.start = r_upper_arm_data.get("start", [])
        config.R_upperArm.end = r_upper_arm_data.get("end", [])
        config.R_upperArm.radius = r_upper_arm_data.get("radius", 0.0)
        config.R_upperArm.type = r_upper_arm_data.get("type", "")

        # 右臂腕部（BallModel）
        r_wrist_data = json_data.get("r_wrist", {})
        config.R_wrist.offset = r_wrist_data.get("offset", [])
        config.R_wrist.radius = r_wrist_data.get("radius", 0.0)
        config.R_wrist.type = r_wrist_data.get("type", "")

    except FileNotFoundError:
        raise RuntimeError(f"无法打开文件: {file_path}")
    except Exception as e:
        raise RuntimeError(f"JSON解析错误: {str(e)}")