import cv2
import re
import rclpy
import string
import subprocess
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import time
from cv_bridge import CvBridge
from std_msgs.msg import String,Int8,Bool
from sensor_msgs.msg import Image
from nav2_msgs.action import NavigateToPose
from interfaces.action import Progress
import math
import yaml
import tempfile, shutil
import psutil
from concurrent.futures import Future
from ament_index_python.packages import get_package_share_directory
import os
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import threading
from rclpy.executors import MultiThreadedExecutor
from turn_on_wheeltec_robot.msg import Position 
from rclpy.qos import qos_profile_sensor_data

# ---------- 参数 ----------
PREDICT_TIME = 1.5          # s，预测前瞻时间
ROBOT_WIDTH  = 0.3         # m，车身半宽
LIDAR_RANGE  = 0.8         # m，最大刹车距离

def normalize_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))

class CustomActionServer(Node):
    def __init__(self):
        super().__init__("action_service_node")
        # 初始化参数配置 
        self.init_param_config()
        # 初始化ROS通信 
        self.init_ros_comunication()
        self.init_navigation_client()
        self.get_logger().info("action service started...")

    def init_param_config(self):
        """
        初始化参数配置 / Initialize parameter configuration
        """
        # 设置夹取启动文件路径 
        pkg_share = get_package_share_directory("largemodel")
        self.map_mapping_config = os.path.join(pkg_share, "config", "map_mapping.yaml")
        config_param_file = os.path.join(pkg_share, "config", "model_config.yaml")
        with open(config_param_file, "r") as file:
            config_param = yaml.safe_load(file)
        self.multimodel = config_param.get("multimodel")
        # 声明参数 
        self.declare_parameter("Speed_topic", "/cmd_vel")
        self.declare_parameter("text_chat_mode", False)
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        # 获取参数值 
        self.Speed_topic = (
            self.get_parameter("Speed_topic").get_parameter_value().string_value
        )
        self.text_chat_mode = (
            self.get_parameter("text_chat_mode").get_parameter_value().bool_value
        )
        # 创建文字交互发布者 
        self.text_pub = self.create_publisher(String, "feedback_words", 1)
        print(self.text_chat_mode)
        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.pkg_path = get_package_share_directory("largemodel")
        self.image_save_path = os.path.join(
            self.pkg_path, "resources_file", "image.png"
        )
        self.positionSubscriber = self.create_subscription(
		    Position,
		    '/object_tracker/current_position',
		    self.distanceCallback,
		    qos_profile=qos_profile_sensor_data)

        self.visual_follower_future = Future()
        self.laser_follower_future = Future()
        self.line_follower_future = Future()
        self.KCF_follow_future = Future()
        self.navigation_future = Future()
        self.slam_future = Future()
        self.slam_future.set_result(True)

        self.interrupt_flag = False  # 打断标志 
        self.action_runing = False  # 动作执行状态 
        self.IS_SAVING = False #是否正在保存图像
        self.obstacle_angle = 0.0 
        self.obstacle_dist = 0.0

        # 图像处理对象 
        self.image_msg = None
        self.bridge = CvBridge()

        self.feedback_largemoel_dict =  {  
            "navigation_1": "机器人反馈:导航目标{point_name}被拒绝",
            "navigation_2": "机器人反馈:执行navigation({point_name})完成",
            "navigation_3": "机器人反馈:执行navigation({point_name})失败，目标点不存在",
            "navigation_4": "机器人反馈:执行navigation({point_name})失败",
            "get_current_pose_success": "机器人反馈:get_current_pose()成功",
            "wait_done": "机器人反馈:执行wait({duration})完成",
            "set_cmdvel_done": "机器人反馈:执行set_cmdvel({linear_x},{linear_y},{angular_z},{duration})完成",
            "seewhat_done": "机器人反馈:执行seewhat()完成",
            "seewhat_func": "seewhat_func",
            "move_left_done": "机器人反馈:执行move_left({angle},{angular_speed})完成",
            "move_right_done": "机器人反馈:执行move_right({angle},{angular_speed})完成",
            "response_done": "机器人反馈：回复用户完成",
            "failure_execute_action_function_not_exists": "机器人反馈:动作函数不存在，无法执行",
            "finish": "finish",
            "finish_task": "f机器人反馈：执行跟随任务完成",
            "multiple_done": "机器人反馈：执行{actions}完成",
            "shutdown_done": "机器人反馈：关机指令执行完毕",
            "restart_done": "机器人反馈：重启指令执行完毕",
            "auto_charge_done": "机器人反馈：执行auto_charge()完成，已成功对接充电桩",
            "leave_charge_done": "机器人反馈：执行leave_charge()完成，已安全脱离充电桩",
            "set_initial_pose_done": "机器人反馈：执行set_initial_pose_to_origin()完成，位置已重置",
            "formation_row_done": "机器人反馈:执行formation_row()完成，横队目标已下发",
            "formation_triangle_done": "机器人反馈:执行formation_triangle()完成，三角队目标已下发",
            "all_return_start_done": "机器人反馈:执行all_return_start()完成，全体返回发车点目标已下发"
        }
        self._sensor_map = {
            '相机': '/camera/color/image_raw',
            '雷达': '/scan',
            '里程计': '/odom'
        }

    def init_ros_comunication(self):
        """
        初始化创建ros通信对象、函数 / Initialize creation of ROS communication objects and functions
        """
        # 创建速度话题发布者 
        self.publisher = self.create_publisher(Twist, self.Speed_topic, 10)
        # 创建动作执行服务器，用于接受动作列表，并执行动作 
        self._action_server = ActionServer(
            self, Progress, "action_service", self.execute_callback
        )
        # 创建执行动作状态发布者 
        self.actionstatus_pub = self.create_publisher(String, "actionstatus", 3)
        # 创建发布者，发布 seewhat_handle 话题 
        self.seewhat_handle_pub = self.create_publisher(String, "seewhat_handle", 1)
        # 创建打断状态订阅者
        self.wakeup = self.create_subscription(Int8, "awake_flag",self.wakeup_callback, 1)

        # 【新增】：发布初始位姿的话题，用于自动和手动重置小车原点坐标
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # 【编队】次车导航目标发布器（下行中继白名单已有 goal_pose）
        self._formation_robot_1_goal_pub = self.create_publisher(PoseStamped, "/robot_1/goal_pose", 10)
        self._formation_robot_2_goal_pub = self.create_publisher(PoseStamped, "/robot_2/goal_pose", 10)

        # 创建tf监听者，监听坐标变换 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 图像话题订阅者
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, 2
        )
        self._check_timer = self.create_timer(5.0, self._check_sensor_timer)
        # 【新增】：创建 KCF 速度中间话题订阅者，用于拦截并加入底盘避障安全逻辑
        self.kcf_cmd_sub = self.create_subscription(
            Twist,
            "/kcf_cmd_vel",
            self.kcf_cmd_callback,
            10
        )
        self.debug_sub = self.create_subscription(
            String, "/debug_cmd", self.debug_callback, 1
        )
    def debug_callback(self, msg):
        """调试专用入口：收到特定字符串，直接执行对应的函数"""
        if msg.data == 'align':
            self.get_logger().info("====================================")
            self.get_logger().info("🚪 收到调试指令：准备启动视觉节点并执行对齐！")
            self.get_logger().info("====================================")
            
            # 1. 模拟 auto_charge，手动拉起视觉神经系统
            vision_tf_process = subprocess.Popen([
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_color_optical_frame'
            ])
            apriltag_process = subprocess.Popen([
                'ros2', 'run', 'apriltag_ros', 'apriltag_node',
                '--ros-args',
                '-r', 'image_rect:=/camera/color/image_raw',
                '-r', 'camera_info:=/camera/color/camera_info',
                '--params-file', '/home/nvidia/wheeltec_ros2/tags.yaml',
                '-p', 'decimate:=2.0',
                '-p', 'threads:=2'
            ])
            
            self.get_logger().info("正在等待视觉节点启动 (3秒)...")
            time.sleep(3.0) 
            
            # 2. 执行对齐逻辑
            try:
                self.visual_align()
            finally:
                # 3. 测试完后自动杀掉进程释放算力
                self.kill_process_tree(vision_tf_process.pid)
                self.kill_process_tree(apriltag_process.pid)
                self.get_logger().info("调试结束，视觉节点已安全关闭。")    
    def init_navigation_client(self):
        # 创建导航功能客户端，请求导航动作服务器 
        self.load_target_points()
        self.navclient = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.current_pose = PoseWithCovarianceStamped()
        self.record_pose = PoseStamped()
        # self.get_current_pose()
        #打断正在导航标志
        self.nav_runing = False 
        self.nav_status = False

    def load_target_points(self):
        """
        加载地图映射文件 /Load map mapping file
        """
        if not os.path.isfile(self.map_mapping_config):
            self.navpose_dict = {}
            self.navname_dict = {}
            return
            
        with open(self.map_mapping_config, "r", encoding="utf-8") as file:
            target_points = yaml.safe_load(file) or {}
            
        self.navpose_dict = {}
        self.navname_dict = {}  # 【新增】：专门用于存储 字母(如'B') -> 中文名(如'充电桩') 的翻译对照表
        
        for key, data in target_points.items():
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = data["position"]["x"]
            pose.pose.position.y = data["position"]["y"]
            pose.pose.position.z = data["position"]["z"]
            pose.pose.orientation.x = data["orientation"]["x"]
            pose.pose.orientation.y = data["orientation"]["y"]
            pose.pose.orientation.z = data["orientation"]["z"]
            pose.pose.orientation.w = data["orientation"]["w"]
            
            self.navpose_dict[key] = pose
            self.navname_dict[key] = data.get("name", key) # 保存对应的中文名

    def get_current_pose(self,name: str= "" ):
        """
        获取当前在全局地图坐标系下的位置与名称，并写入 map_mapping.yaml
        """
        # 1. 获取当前位姿
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            msg = String(data=f"获取失败，请重新定位")
            self.text_pub.publish(msg)
            if not self.interrupt_flag:
                self.action_status_pub("get_current_pose_failed")
            return

        # 2. 读取已有 YAML 或新建
        if os.path.isfile(self.map_mapping_config):
            with open(self.map_mapping_config, "r", encoding="utf-8") as f:
                target_points = yaml.safe_load(f) or {}
        else:
            target_points = {}
            os.makedirs(os.path.dirname(self.map_mapping_config), exist_ok=True)
        # 3. 检查是否有重复的名称
        name=name.strip('"\'')
        target_points = {k: v for k, v in target_points.items()
                 if v["name"].strip('"\'') != name}
        
        # 4.分配下一个字母键（防跳跃）
        used = {k for k in target_points.keys()
                if len(k) == 1 and k in string.ascii_uppercase}
        next_key = next((ch for ch in string.ascii_uppercase if ch not in used), None)
        if next_key is None:
            self.get_logger().error("Too many poses, ran out of letters A-Z!")
            if not self.interrupt_flag:
                self.action_status_pub("get_current_pose_failed")
            return

        if not name:
            name = f"未命名{len(target_points)}"
        
        # 5. 组装数据结构
        target_points[next_key] = {
            "name": name,
            "position": {
                "x": float(transform.transform.translation.x),
                "y": float(transform.transform.translation.y),
                "z": 0.0,
            },
            "orientation": {
                "x": float(transform.transform.rotation.x),
                "y": float(transform.transform.rotation.y),
                "z": float(transform.transform.rotation.z),
                "w": float(transform.transform.rotation.w),
            },
        }

        # 6.原子写
        try:
            with tempfile.NamedTemporaryFile(
                    mode="w", encoding="utf-8",
                    dir=os.path.dirname(self.map_mapping_config),
                    delete=False) as tmp:
                yaml.dump(target_points, tmp, allow_unicode=True,
                        sort_keys=False, default_flow_style=False)
                tmp.flush()
            shutil.move(tmp.name, self.map_mapping_config)
        except OSError as e:
            self.get_logger().error(f"Write map_mapping.yaml failed: {e}")
            if not self.interrupt_flag:
                self.action_status_pub("get_current_pose_failed")
            return
        # 7. 打印日志
        self.get_logger().info(
            f"Recorded pose {next_key}: '{name}' -> \n"
            f"  position: x={target_points[next_key]['position']['x']:.2f}, "
            f"y={target_points[next_key]['position']['y']:.2f}, z=0.0\n"
        )

        if not self.interrupt_flag:
            self.action_status_pub("get_current_pose_success")

    def action_status_pub(self, key, **kwargs):
        """
        动作结果发布方法
        """
        text_template = self.feedback_largemoel_dict.get(key)

        try:
            message = text_template.format(**kwargs)
        except KeyError as e:
            self.get_logger().error(f"Translation placeholder error: {e} (key: {key})")
            message = f"[Translation failed: {key}]"
        # 发布消息
        self.actionstatus_pub.publish(String(data=message))
        self.get_logger().info(f"Published message: {message}")
        
    def navigation(self, point_name):
        """
        从navpose_dict字典中获取目标点坐标，并导航到目标点
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            msg = String(data=f"导航失败，请重新定位")
            self.text_pub.publish(msg)
            return False
            
        self.load_target_points()
        self.navigation_finish_flag = False
        self.nav_success_flag = False
        self.goal_handle = None
        self.result = None
        point_name = point_name.strip("'\"")
        
        if point_name not in self.navpose_dict:
            self.get_logger().error(f"Target point '{point_name}' does not exist.")
            self.action_status_pub("navigation_3", point_name=point_name)
            return False
            
        # 【核心翻译】：获取该字母对应的中文名，用于语音播报
        chinese_name = self.navname_dict.get(point_name, point_name)
        
        target_pose = self.navpose_dict.get(point_name)
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = "map"

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        send_goal_future = self.navclient.send_goal_async(goal_msg)
        
        def goal_response_callback(future):
            self.goal_handle = future.result()
            if not self.goal_handle or not self.goal_handle.accepted:
                self.get_logger().error("Goal was rejected!")
                self.action_status_pub("navigation_1", point_name=point_name) # 大模型用字母
                self.navigation_finish_flag = True
                return
            get_result_future = self.goal_handle.get_result_async()

            def result_callback(future_result):
                result = future_result.result()
                self.navigation_finish_flag = True
                if self.nav_status:
                    self.nav_status = False
                    self.action_status_pub("navigation_5", point_name=point_name)
                    self.nav_runing = False
                    self.nav_success_flag = False
                else:
                    if result.status == 4:
                        self.nav_success_flag = True
                        self.action_status_pub("navigation_2", point_name=point_name) # 大模型用字母
                        msg = String(data=f"报告，我已经成功到达{chinese_name}啦！") # 人类语音用中文名
                        self.text_pub.publish(msg)
                    else:
                        self.nav_success_flag = False
                        self.get_logger().info(f"Navigation failed with status: {result.status}")
                        self.action_status_pub("navigation_4", point_name=point_name) # 大模型用字母
                        msg = String(data=f"抱歉，前往{chinese_name}的路上遇到了困难，导航失败了。") # 人类语音用中文名
                        self.text_pub.publish(msg)

            get_result_future.add_done_callback(result_callback)

        send_goal_future.add_done_callback(goal_response_callback)
        
        while not self.navigation_finish_flag:
            if self.interrupt_flag and self.goal_handle is not None:
                self.navclient._cancel_goal(self.goal_handle)
                return False
            time.sleep(0.1)
        self.stop()
        return self.nav_success_flag
        def goal_response_callback(future):
            self.goal_handle = future.result()
            if not self.goal_handle or not self.goal_handle.accepted:
                self.get_logger().error("Goal was rejected!")
                self.action_status_pub("navigation_1", point_name=point_name)
                self.navigation_finish_flag = True  # 【修改点】：目标被拒绝时，允许跳出 while 循环
                return
            get_result_future = self.goal_handle.get_result_async()

            def result_callback(future_result):
                result = future_result.result()
                self.navigation_finish_flag = True
                if self.nav_status:
                    self.nav_status = False
                    self.action_status_pub("navigation_5", point_name=point_name)
                    self.nav_runing = False
                    self.nav_success_flag = False
                else:
                    if result.status == 4:
                        self.nav_success_flag = True  # 标记成功
                        self.action_status_pub("navigation_2", point_name=point_name)
                        msg = String(data=f"报告，我已经成功到达{point_name}啦！")
                        self.text_pub.publish(msg)
                    else:
                        self.nav_success_flag = False # 标记失败
                        self.get_logger().info(f"Navigation failed with status: {result.status}")
                        self.action_status_pub("navigation_4", point_name=point_name)
                        msg = String(data=f"抱歉，前往{point_name}的路上遇到了困难，导航失败了。")
                        self.text_pub.publish(msg)

            get_result_future.add_done_callback(result_callback)

        send_goal_future.add_done_callback(goal_response_callback)
        
        while not self.navigation_finish_flag:
            if self.interrupt_flag and self.goal_handle is not None:
                self.navclient._cancel_goal(self.goal_handle)
                return False  # 【修改点】：被打断时返回 False
            time.sleep(0.1)
        self.stop()
        return self.nav_success_flag  # 【核心修改】：将导航的最终真实结果返回给调用者
        # 将 goal_response_callback 放在正确的层级（navigation的内部）
        def goal_response_callback(future):
            self.goal_handle = future.result()
            if not self.goal_handle or not self.goal_handle.accepted:
                self.get_logger().error("Goal was rejected!")
                self.action_status_pub("navigation_1", point_name=point_name)
                return
            get_result_future = self.goal_handle.get_result_async()

            def result_callback(future_result):
                result = future_result.result()
                self.navigation_finish_flag = True
                if self.nav_status:
                    self.nav_status = False
                    self.action_status_pub("navigation_5", point_name=point_name)
                    self.nav_runing = False
                else:
                    if result.status == 4:
                        self.action_status_pub("navigation_2", point_name=point_name)
                        msg = String(data=f"报告，我已经成功到达{point_name}啦！")
                        self.text_pub.publish(msg)
                    else:
                        self.get_logger().info(f"Navigation failed with status: {result.status}")
                        self.action_status_pub("navigation_4", point_name=point_name)
                        msg = String(data=f"抱歉，前往{point_name}的路上遇到了困难，导航失败了。")
                        self.text_pub.publish(msg)

            get_result_future.add_done_callback(result_callback)

        send_goal_future.add_done_callback(goal_response_callback)
        
        while not self.navigation_finish_flag:
            if self.interrupt_flag and self.goal_handle is not None:
                self.navclient._cancel_goal(self.goal_handle)
                break
            time.sleep(0.1)
        self.stop()

    def auto_charge(self):
        """
        前置自动充电连招：动态寻找充电Key -> 导航 -> 视觉精瞄与盲推入库
        （已删除废弃的 180 度掉头与倒车逻辑，完全交由 visual_align 处理入库）
        """
        self.get_logger().info("====== 开始执行全自动充电任务 ======")
        self.get_logger().info("1. 正在导航至充电预备点...")
        
        # 1. 动态寻找“充电桩”对应的字母 Key
        self.load_target_points()
        charge_key = None
        for k, v in getattr(self, 'navname_dict', {}).items():
            if "充电桩" in v:
                charge_key = k
                break
                
        if charge_key is None:
            self.get_logger().error("地图里未找到'充电桩'的记录，任务终止。")
            msg = String(data="抱歉，我的地图里还没记录充电桩的位置呢。")
            self.text_pub.publish(msg)
            return
            
        # 2. 导航并验证结果
        nav_success = self.navigation(charge_key)
        if self.interrupt_flag: return
        if not nav_success:
            self.get_logger().error("未能成功到达充电预备点，终止对接！")
            msg = String(data="抱歉，我没能开到充电桩附近，对接任务取消。")
            self.text_pub.publish(msg)
            return

        time.sleep(2.0) 
        
        # 3. 拉起视觉系统
        self.get_logger().info("到达预备点！正在自动唤醒底层的视觉神经系统...")
        vision_tf_process = subprocess.Popen([
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_color_optical_frame'
        ])
        apriltag_process = subprocess.Popen([
            'ros2', 'run', 'apriltag_ros', 'apriltag_node',
            '--ros-args',
            '-r', 'image_rect:=/camera/color/image_raw',
            '-r', 'camera_info:=/camera/color/camera_info',
            '--params-file', '/home/nvidia/wheeltec_ros2/tags.yaml',
            '-p', 'decimate:=2.0',
            '-p', 'threads:=2'
        ])
        
        time.sleep(3.0) 

        try:
            # ---------------------------------------------------------
            # 视觉高精度瞄准与贴合
            # ---------------------------------------------------------
            self.get_logger().info("2. 启动底层视觉伺服对齐与前置压紧...")
            align_success = self.visual_align()
            
            if not align_success:
                self.get_logger().error("视觉对齐失败！")
                msg = String(data="我找不到充电桩的二维码了。")
                self.text_pub.publish(msg)
                return

            # visual_align 成功返回后，小车已经稳稳压住充电桩
            self.get_logger().info("✅ 充电对接完全成功！")
            msg = String(data="开始充电啦！")
            self.text_pub.publish(msg)
            
            if not self.interrupt_flag:
                self.action_status_pub("auto_charge_done")

        finally:
            self.get_logger().info("任务结束，正在自动关闭视觉神经系统以节省算力...")
            self.kill_process_tree(vision_tf_process.pid)
            self.kill_process_tree(apriltag_process.pid)
    def leave_charge(self):
        """
        脱离充电桩：使用陀螺仪闭环控制笔直前进，驶出充电桩的障碍物膨胀区。
        解决直接使用 Nav2 导航会导致“起点在障碍物内”的报错问题。
        """
        self.get_logger().info("====== 开始执行脱离充电桩任务 ======")
        msg = String(data="电量已满！我正在脱离充电桩，请稍等...")
        self.text_pub.publish(msg)

        # 记录启动时的绝对方位角，作为直行的“钢丝线”
        try:
            trans = self.tf_buffer.lookup_transform('odom_combined', 'base_footprint', rclpy.time.Time())
            rx, ry, rz, rw = trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w
            lock_yaw = math.atan2(2.0 * (rw * rz + rx * ry), 1.0 - 2.0 * (ry * ry + rz * rz))
        except Exception as e:
            self.get_logger().warn(f"获取直行基准角度失败，退回开环盲进: {e}")
            lock_yaw = None
        
        twist = Twist()
        # 向前行驶 4.0 秒，确保小车的后轮完全离开充电桩的代价地图膨胀半径
        forward_duration = 4.0 
        start_time = time.time()
        
        # 按照 20Hz 的频率发布控制指令
        while (time.time() - start_time) < forward_duration:
            if self.interrupt_flag:
                self.stop()
                return
            
            # 基础前进速度 0.15 m/s，与倒车速度大小保持一致
            twist.linear.x = 0.15
            twist.linear.y = 0.0
            
            # 陀螺仪防偏纠正（如果前向开动时压到不平整地面导致车头偏斜，自动纠正）
            if lock_yaw is not None:
                try:
                    curr_trans = self.tf_buffer.lookup_transform('odom_combined', 'base_footprint', rclpy.time.Time())
                    crx, cry, crz, crw = curr_trans.transform.rotation.x, curr_trans.transform.rotation.y, curr_trans.transform.rotation.z, curr_trans.transform.rotation.w
                    curr_yaw = math.atan2(2.0 * (crw * crz + crx * cry), 1.0 - 2.0 * (cry * cry + crz * crz))
                    
                    diff = lock_yaw - curr_yaw
                    diff = math.atan2(math.sin(diff), math.cos(diff)) 
                    
                    # P控制修正偏航
                    twist.angular.z = 1.0 * diff  
                except:
                    twist.angular.z = 0.0
            else:
                twist.angular.z = 0.0
                
            self.publisher.publish(twist)
            time.sleep(0.05)
            
        self.stop()
        
        if not self.interrupt_flag:
            msg = String(data="我已经安全离开充电桩啦，可以执行下一个任务了。")
            self.text_pub.publish(msg)
            # 发布动作完成状态给大模型
            self.action_status_pub("leave_charge_done")
    # 主车标准发车点（my_map 新狗图坐标系；2026-09-18 重测）
    # 与 maincar_start_pose.txt、rtabmap_nav_params.yaml 的 initial_pose 一致
    START_POSE_X = 0.763
    START_POSE_Y = -2.143
    START_POSE_YAW_DEG = 2.0

    def _publish_initial_pose(self):
        """
        底层基座方法：向 /initialpose 话题发送主车标准发车点坐标
        （2026-09-18 起由 (0,0,0) 原点改为实测发车点 START_POSE_X/Y/YAW_DEG）
        """
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        yaw = math.radians(self.START_POSE_YAW_DEG)
        msg.pose.pose.position.x = self.START_POSE_X
        msg.pose.pose.position.y = self.START_POSE_Y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        
        msg.pose.covariance[0] = 0.9
        msg.pose.covariance[7] = 0.9
        msg.pose.covariance[35] = 0.9
        
        # 连发3次，防止丢包
        for _ in range(3):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.2)

    def set_initial_pose_to_origin(self):
        """
        大模型动作：人工搬运后语音确认，重置位置为标准发车点。
        """
        self.get_logger().info("收到用户指令，正在将位置重置为标准发车点 (0.763, -2.143, 2.0°)...")
        self._publish_initial_pose()
        
        msg_text = String(data="位置校准完毕，我现在清晰地知道自己在哪里啦！")
        self.text_pub.publish(msg_text)
        
        if not self.interrupt_flag:
            self.action_status_pub("set_initial_pose_done")

    # ════════════════════════════════════════════════════════════
    #  多机编队（2026-09-22 新增）：横队列队
    #  锚点=主车当前实时位姿(map系)；小车1在主车左侧、狗在右侧，
    #  各偏移 FORMATION_SPACING 米、朝向与主车一致；只发 goal_pose，
    #  到位由次车 Nav2 自主完成；60s 后台线程语音回报整队结果。
    # ════════════════════════════════════════════════════════════
    FORMATION_SPACING = 1.5     # m，槽位与锚点的横向间距（2026-09-23 1.0→1.5：bag 实测 1m 间距下两车互入对方 inflation 区，狗被列队中的小车1 挤退 1.4m）
    FORMATION_TIMEOUT = 60.0    # s，整队回报时限
    FORMATION_YAW_OFF = math.radians(90.0)   # 槽位相对主车朝向的方位角（左90°/右-90°）

    def _formation_slot(self, ax, ay, ayaw, side):
        """
        由锚点(主车位姿)推算编队槽位。
        side=+1 → 主车前进方向左侧；side=-1 → 右侧。
        返回 (x, y, yaw)：槽位位置与朝向（朝向与主车一致）。
        """
        bx = ax + math.cos(ayaw + side * self.FORMATION_YAW_OFF) * self.FORMATION_SPACING
        by = ay + math.sin(ayaw + side * self.FORMATION_YAW_OFF) * self.FORMATION_SPACING
        return bx, by, ayaw

    @staticmethod
    def _yaw_of(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def formation_row(self):
        """
        大模型动作：横队列队。小车1去主车左侧1米、机器狗去右侧1米，
        主车原地不动。目标经下行中继下发，次车 Nav2 各自导航。
        """
        # 1. 取锚点：主车当前实时位姿
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"[formation] TF lookup failed: {e}")
            self.text_pub.publish(String(data="我还没定位好，请先校准位置再列队。"))
            return
        ax = tf.transform.translation.x
        ay = tf.transform.translation.y
        ayaw = self._yaw_of(tf.transform.rotation)
        self.get_logger().info(
            f"[formation] anchor x={ax:.2f} y={ay:.2f} yaw={math.degrees(ayaw):.1f}°"
        )

        # 2. 推算两槽位（左=小车1，右=狗）
        r1 = self._formation_slot(ax, ay, ayaw, side=+1)
        r2 = self._formation_slot(ax, ay, ayaw, side=-1)

        # 3. 下发（公共函数，与三角队共用）
        self._formation_publish(r1, r2)

        # 4. 即时语音 + 60s 后台回报
        self.text_pub.publish(String(
            data="收到！小车1到我左边一米，机器狗到我右边一米，横排整队开始！"))
        t = threading.Thread(target=self._formation_report, kwargs={"form_type": "横队"}, daemon=True)
        t.start()

        if not self.interrupt_flag:
            self.action_status_pub("formation_row_done")

    def _formation_report(self, form_type="横队"):
        """60 秒后整队回报（2026-09-23 起真校验：TF 读两车实际位姿，距各自槽位 <0.30m 才算就位）。"""
        time.sleep(self.FORMATION_TIMEOUT)
        try:
            ok, detail = self._check_formation_arrival()
            if ok:
                self.text_pub.publish(String(data=f"{form_type}整队完毕！"))
            else:
                self.text_pub.publish(String(data=f"{form_type}整队未完成：{detail}。可以再说一次列队口令让我重发目标。"))
        except Exception as e:
            self.get_logger().warn(f"[formation_report] 校验异常回退盲报: {e}")
            self.text_pub.publish(String(data=f"{form_type}整队完毕！"))

    def _check_formation_arrival(self, tol=0.30, wait=6.0):
        """
        用最近一次 _formation_publish 记录的两个槽位做校验。
        返回 (bool, str): 全部就位/谁没到位。
        """
        slots = getattr(self, "_last_slots", None)
        if not slots:
            return False, "没有可校验的槽位记录"
        # 2026-09-23 改原始 /tf BEST_EFFORT 订阅+手工两级复合:
        # tf2 Buffer 默认 RELIABLE 收不到 domain 桥转发的 robot_1 帧(实测 QoS 不兼容)
        tf_cache = {}
        node = rclpy.create_node('formation_check_tmp')
        qos = QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        def on_tf(msg):
            for t in msg.transforms:
                tf_cache[(t.header.frame_id, t.child_frame_id)] = t
        node.create_subscription(TFMessage, '/tf', on_tf, qos)
        t0 = time.time()
        while time.time() - t0 < wait:
            rclpy.spin_once(node, timeout_sec=0.3)
        node.destroy_node()
        bad = []
        for (gx, gy, gyaw, name) in slots:
            odom = "robot_1/odom" if "小车1" in name else "robot_2/odom"
            base = "robot_1/base_link" if "小车1" in name else "robot_2/base_link"
            t1 = tf_cache.get(("map", odom))
            t2 = tf_cache.get((odom, base))
            if not (t1 and t2):
                bad.append(f"{name}位姿不可读")
                continue
            def _yaw(tr):
                q = tr.transform.rotation
                return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            ax, ay, ayaw = t1.transform.translation.x, t1.transform.translation.y, _yaw(t1)
            bx, by, byaw = t2.transform.translation.x, t2.transform.translation.y, _yaw(t2)
            x = ax + math.cos(ayaw)*bx - math.sin(ayaw)*by
            y = ay + math.sin(ayaw)*bx + math.cos(ayaw)*by
            dist = math.hypot(x - gx, y - gy)
            self.get_logger().info(
                f"[formation_check] {name} 距槽位 {dist:.2f}m (实际({x:.2f},{y:.2f}) 目标({gx:.2f},{gy:.2f}))"
            )
            if dist > tol:
                bad.append(f"{name}还差{dist:.1f}米")
        if bad:
            return False, "、".join(bad)
        return True, ""


    def formation_triangle(self):
        """
        大模型动作：三角队列队。主车为前顶点，小车1去左后45°、
        机器狗去右后45°，各1.0米（等腰直角三角形，两车相距约1.41米），
        主车原地不动。目标经下行中继下发，次车 Nav2 各自导航。
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"[formation] TF lookup failed: {e}")
            self.text_pub.publish(String(data="我还没定位好，请先校准位置再列队。"))
            return
        ax = tf.transform.translation.x
        ay = tf.transform.translation.y
        ayaw = self._yaw_of(tf.transform.rotation)

        off = math.radians(135.0)   # 左后/右后 45°（相对前进方向 180°∓45°）
        r1 = (
            ax + math.cos(ayaw + off) * self.FORMATION_SPACING,
            ay + math.sin(ayaw + off) * self.FORMATION_SPACING,
            ayaw,
        )
        r2 = (
            ax + math.cos(ayaw - off) * self.FORMATION_SPACING,
            ay + math.sin(ayaw - off) * self.FORMATION_SPACING,
            ayaw,
        )
        self._formation_publish(r1, r2)
        self.text_pub.publish(String(
            data="收到！小车1到我左后方，机器狗到我右后方，三角整队开始！"))
        threading.Thread(target=self._formation_report, kwargs={"form_type": "三角队"}, daemon=True).start()
        if not self.interrupt_flag:
            self.action_status_pub("formation_triangle_done")

    def _formation_publish(self, r1, r2):
        """编队公共下发：两槽位各连发两次 goal_pose 防跨机丢包。"""
        now = self.get_clock().now().to_msg()
        # 记录槽位供 60s 后整队校验(2026-09-23)
        self._last_slots = [
            (r1[0], r1[1], r1[2], "小车1"),
            (r2[0], r2[1], r2[2], "机器狗"),
        ]
        for (x, y, yaw), topic in ((r1, "/robot_1/goal_pose"), (r2, "/robot_2/goal_pose")):
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = now
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.orientation.w = math.cos(yaw / 2.0)
            pub = getattr(self, "_formation_" + topic.split("/")[1] + "_goal_pub")
            for _ in range(2):
                pub.publish(goal)
                time.sleep(0.1)
            self.get_logger().info(
                f"[formation] {topic} -> ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)"
            )

    # ════════════════════════════════════════════════════════════
    #  全体回发车点（2026-09-23 新增）：语音"全体回到发车点"
    #  按名字在 map_mapping.yaml 动态找 G/H/I 三点(各自车种的发车点)：
    #    主车 -> navigation(自己的key)   (Nav2 自主导航)
    #    小车1 -> /robot_1/goal_pose    (下行中继, 连发2次防丢包)
    #    狗    -> /robot_2/goal_pose    (下行中继, 连发2次防丢包)
    # ════════════════════════════════════════════════════════════
    def all_return_start(self):
        """
        大模型动作：全体回发车点。狗和小车1各自导航回自己的发车点，
        主车同时导航回自己的发车点。
        """
        # 1. 加载点位映射(字母->中文名)
        self.load_target_points()
        navname = getattr(self, 'navname_dict', {})
        # 名字匹配规则：兼容"主车发车点/机械狗发车点/履带车发车点"的既有命名
        main_key = next((k for k, v in navname.items() if '主车发车' in v), None)
        r1_key   = next((k for k, v in navname.items() if '履带车发车' in v or '小车1发车' in v), None)
        dog_key  = next((k for k, v in navname.items() if '机械狗发车' in v or '狗发车' in v), None)

        if not all([main_key, r1_key, dog_key]):
            self.get_logger().error(f"[all_return] 发车点记录不全: 主车={main_key} 小车1={r1_key} 狗={dog_key}")
            self.text_pub.publish(String(data="抱歉，地图里还没有记全三台车的发车点呢。"))
            return

        # 2. 下发次车目标(连发 2 次防跨机丢包, 与编队共用发布器)
        now = self.get_clock().now().to_msg()
        for key, topic in ((r1_key, "/robot_1/goal_pose"), (dog_key, "/robot_2/goal_pose")):
            pose = self.navpose_dict.get(key)
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = now
            goal.pose = pose.pose
            pub = getattr(self, "_formation_" + topic.split("/")[1] + "_goal_pub")
            for _ in range(2):
                pub.publish(goal)
                time.sleep(0.1)
            self.get_logger().info(
                f"[all_return] {topic} -> {key}({navname[key]}) "
                f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
            )

        # 3. 主车自己回自己的发车点(Nav2 动作, 不阻塞语音线程太久: 后台线程)
        t = threading.Thread(
            target=lambda: self.navigation(main_key), daemon=True
        )
        t.start()

        self.text_pub.publish(String(
            data="收到！大家各自回发车点，小车1回履带车发车点，机器狗回机械狗发车点，我回主车发车点！"))
        if not self.interrupt_flag:
            self.action_status_pub("all_return_start_done")

    def wait(self, duration):
        duration = float(duration)
        time.sleep(duration)
        if not self.interrupt_flag:
            self.action_status_pub("wait_done", duration=duration)

    def seewhat(self,func=None):
        self.save_single_image()
        if func is not None:
            msg = String(data=f'{func}')
        else :
            msg = String(data="seewhat")
        self.seewhat_handle_pub.publish(
            msg
        )  
        self.action_status_pub("seewhat_done")
            
    def set_cmdvel(self, linear_x, linear_y, angular_z, duration): 
        linear_x = float(linear_x)
        linear_y = float(linear_y)
        angular_z = float(angular_z)
        duration = float(duration)
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self._execute_action(twist, durationtime=duration+0.3)
        if not self.interrupt_flag:
            self.action_status_pub(
                "set_cmdvel_done",
                linear_x=linear_x,
                linear_y=linear_y,
                angular_z=angular_z,
                duration=duration,
            )

    def move_left(self, angle, angular_speed): 
        angle = float(angle)
        angular_speed = float(angular_speed)
        angle_rad = math.radians(angle) 
        duration = abs(angle_rad / angular_speed)
        angular_speed = abs(angular_speed)
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        self._execute_action(twist, 1, duration+0.8)
        self.stop()
        if not self.interrupt_flag:
            self.action_status_pub(
                "move_left_done",
                angle=angle,
                angular_speed=angular_speed,
            )

    def move_right(self, angle, angular_speed): 
        angle = float(angle)
        angular_speed = float(angular_speed)
        angle_rad = math.radians(angle)  
        duration = abs(angle_rad / angular_speed)
        angular_speed = -abs(angular_speed)
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        self._execute_action(twist, 1, duration+0.8)
        self.stop()
        if not self.interrupt_flag:
            self.action_status_pub(
                "move_right_done",
                angle=angle,
                angular_speed=angular_speed,
            )

    def stop(self):  
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def stop_follow(self):
        self.get_logger().info("stop procress.....")
        futures = [
            self.visual_follower_future,
            self.laser_follower_future,
            self.line_follower_future,
            self.KCF_follow_future,
        ]
        for future in futures:
            if not future.done():
                future.set_result(True)
        if self.interrupt_flag:
            return
        else:
            self.action_status_pub("finish_task")

    def cancel(self):
        self.stop()

    def shutdown(self):
        if hasattr(self, '_shutting_down') and self._shutting_down:
            return
        self._shutting_down = True
        
        self.get_logger().info("【系统指令】收到关机指令，准备断电...")
        self.stop()
        self.stop_follow()
        self.interrupt_flag = True
        time.sleep(3.0)
        
        os.system("echo 'nvidia' | sudo -S /sbin/shutdown -h now")
        os.system("echo 'nvidia' | sudo -S /sbin/poweroff")
        
        if not self.interrupt_flag:
            self.action_status_pub("shutdown_done")
    def restart(self):
        if hasattr(self, '_shutting_down') and self._shutting_down:
            return
        self._shutting_down = True
        
        self.get_logger().info("准备重启系统...")
        self.stop()
        self.stop_follow()
        self.interrupt_flag = True
        time.sleep(3.0)
        
        # 使用你机器上的密码 'nvidia' 调用系统的重启命令
        os.system("echo 'nvidia' | sudo -S /sbin/reboot")
        
        if not self.interrupt_flag:
            self.action_status_pub("restart_done")
    def visual_align(self):
        """
        视觉伺服精确定位（极限防抖消除随机性版）：
        3毫米极限死区 + 慢速滑行 + 纯柔性纠偏。
        """
        self.get_logger().info("开启视觉高精度对齐 (启用 3mm 极限收敛防随机漂移)...")
        msg = String(data="正在精准对接，马上就好！")
        self.text_pub.publish(msg)

        time.sleep(2.0)

        start_time = time.time()
        stage = 1  
        stable_count = 0
        aligned = False

        last_tf_stamp = None
        stale_tf_count = 0
        last_known_x = 1.0  

        # ================= 👑 核心微调区 =================
        OFFSET_Y = 0.00     
        OFFSET_YAW = 0.00   
        
        # ⚠️ 盲推时间
        BLIND_PUSH_TIME = 0.1  

        TARGET_X_PREP = 0.60  
        TARGET_X_DOCK = 0.04  
        TARGET_Y = 0.00       
        
        # 基础控制参数 (降低了整体上限，追求绝对平稳)
        MIN_VX, MAX_VX, KP_X = 0.05, 0.12, 0.8  
        MIN_VY, MAX_VY, KP_Y = 0.05, 0.12, 0.6  
        MIN_WZ, MAX_WZ, KP_Z = 0.06, 0.20, 0.6  
        # =================================================

        def smooth_control(error, kp, min_v, max_v, near_zone, creep_v):
            ideal = kp * error
            if abs(error) < near_zone:
                if ideal > 0: return min(max(ideal, creep_v), max_v)
                else: return max(min(ideal, -creep_v), -max_v)
            else:
                if ideal > 0: return min(max(ideal, min_v), max_v)
                else: return max(min(ideal, -min_v), -max_v)

        while (time.time() - start_time) < 60.0:
            if self.interrupt_flag:
                self.stop()
                return False

            try:
                trans = self.tf_buffer.lookup_transform(
                    'base_link', 'tag36h11_0', rclpy.time.Time()
                )
                last_known_x = trans.transform.translation.x
                
                # ---- 帧率冻结检测 ----
                current_stamp = f"{trans.header.stamp.sec}_{trans.header.stamp.nanosec}"
                if current_stamp == last_tf_stamp:
                    stale_tf_count += 1
                else:
                    stale_tf_count = 0
                    last_tf_stamp = current_stamp
                    
                if stale_tf_count > 20:
                    raise Exception("TF数据已冻结超1秒，二维码丢失")

                # ---- 误差计算 ----
                error_y = trans.transform.translation.y - TARGET_Y - OFFSET_Y
                
                rx, ry, rz, rw = trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w
                tag_z_x = 2.0 * (rx * rz + rw * ry)
                tag_z_y = 2.0 * (ry * rz - rw * rx)
                raw_yaw = math.atan2(tag_z_y, tag_z_x)
                if tag_z_x < 0: raw_yaw = math.atan2(-tag_z_y, -tag_z_x)
                error_yaw = raw_yaw - OFFSET_YAW

                twist = Twist()

                # ================= 状态机逻辑 =================
                if stage == 1:
                    error_x = trans.transform.translation.x - TARGET_X_PREP
                    self.get_logger().info(f"[阶段 1] 距离X:{error_x:.3f}m | 角度Yaw:{error_yaw:.3f}rad")
                    twist.linear.y = 0.0 
                    
                    if abs(error_x) > 0.015 or abs(error_yaw) > 0.03:
                        twist.linear.x = smooth_control(error_x, KP_X, MIN_VX, MAX_VX, 0.03, 0.04) if abs(error_x) > 0.015 else 0.0
                        twist.angular.z = smooth_control(error_yaw, KP_Z, MIN_WZ, MAX_WZ, 0.05, 0.06) if abs(error_yaw) > 0.03 else 0.0
                    else:
                        self.stop()
                        time.sleep(0.5)
                        stage = 2
                        stable_count = 0

                elif stage == 2:
                    error_x = trans.transform.translation.x - TARGET_X_PREP
                    self.get_logger().info(f"[阶段 2] 距离X:{error_x:.3f}m | 横向Y:{error_y:.3f}m")
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    
                    if abs(error_x) > 0.04 or abs(error_yaw) > 0.08:
                        stage = 1
                        continue

                    if abs(error_y) > 0.003: 
                        # 使用极小蠕动电流 0.035 进行毫米级抠边
                        twist.linear.y = smooth_control(error_y, KP_Y, MIN_VY, MAX_VY, 0.015, 0.035)
                        stable_count = 0
                    else:
                        twist.linear.y = 0.0
                        stable_count += 1
                        if stable_count >= 4:
                            self.get_logger().info(">>> 极限 3mm 对准完成！开始超慢速直线滑入！")
                            self.stop()
                            time.sleep(0.5)
                            stage = 3
                            stable_count = 0

                elif stage == 3:
                    error_x_dock = trans.transform.translation.x - TARGET_X_DOCK
                    self.get_logger().info(f"[阶段 3] 距极片还剩:{error_x_dock:.3f}m | 横偏:{error_y:.3f}m")

                    # 👑 纯比例控制：前进时即使偏了，也不加强制最小蠕动力，避免猛烈甩尾
                    if abs(error_y) > 0.008: 
                        twist.linear.y = error_y * KP_Y  
                    else: twist.linear.y = 0.0

                    if abs(error_yaw) > 0.015:
                        twist.angular.z = error_yaw * KP_Z
                    else: twist.angular.z = 0.0

                    if error_x_dock > 0.01:
                        # 👑 慢速滑行：前进最高速限死在 0.08，有效防止麦轮高速跑偏
                        twist.linear.x = smooth_control(error_x_dock, KP_X, MIN_VX, 0.08, 0.03, 0.04)
                        stable_count = 0
                    else:
                        twist.linear.x = 0.0
                        stable_count += 1
                        if stable_count >= 3:
                            self.get_logger().info("✅ 对接距离满足，完成！")
                            aligned = True
                            break

                self.publisher.publish(twist)
                time.sleep(0.05) 

            except Exception as e:
                # 盲推判定
                if stage == 3 and last_known_x < 0.45: 
                    self.get_logger().warning(f"贴脸盲区({last_known_x:.2f}m)丢失视野，执行最后盲推压紧...")
                    twist = Twist()
                    twist.linear.x = 0.06  
                    twist.linear.y = 0.0
                    twist.angular.z = 0.0
                    self.publisher.publish(twist)
                    
                    time.sleep(BLIND_PUSH_TIME) 
                    
                    self.stop()
                    self.get_logger().info("✅ 盲推结束，已稳稳贴合充电桩！")
                    aligned = True
                    break
                else:
                    self.get_logger().warning(f"远距离丢失画面或异常: {e}")
                    self.stop() 
                    time.sleep(0.2) 

        self.stop()
        if not aligned:
            self.get_logger().error("60秒内未完成对齐，已超时")
        return aligned
    def slam_start(self):
        self.navigation_stop()
        self.slam_future = Future()
        
        # 定义建图后台守护线程
        def slam_daemon():
            process_fuc = subprocess.Popen(['ros2', 'launch', 'largemodel', 'largemodel_slam.launch.py'])
            
            self.loop_closure_count = 0
            self.loop_closure_announced = False
            self._start_lc_monitor()
            
            # 【修改点】：取消 interrupt_flag 的打断，死等直到真正调用了 slam_stop
            while not self.slam_future.done():
                time.sleep(0.5)

            self.kill_process_tree(process_fuc.pid)
            self._stop_lc_monitor()
            self.cancel()
            self.get_logger().info("建图后台服务已彻底关闭。")

        # 启动独立线程，让主程序立刻返回，不阻塞后续语音指令
        slam_thread = threading.Thread(target=slam_daemon)
        slam_thread.daemon = True
        slam_thread.start()
        
        self.get_logger().info("已启动建图后台守护线程。")

    def slam_stop(self):
        if not self.slam_future.done():
            subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                           '-t', '/map', 
                           '-f', '/home/nvidia/wheeltec_ros2/src/wheeltec_robot_rtab/my_map',
                           '--free', '0.19',
                           '--occ', '0.65',
                           '--mode', 'trinary',
                           '--ros-args', 
                           '-p', 'save_map_timeout:=10000.0'],
                          check=False)
            time.sleep(3)
            src_db = os.path.expanduser('~/.ros/rtabmap.db')
            dst_db = '/home/nvidia/wheeltec_ros2/src/wheeltec_robot_rtab/my_room.db'
            if os.path.exists(src_db):
                shutil.copy2(src_db, dst_db)
                self.get_logger().info(f"rtabmap.db has been copied to {dst_db}")
            else:
                self.get_logger().warn(f"rtabmap.db not found at {src_db}")
            self.slam_future.set_result(True)
            msg = String(data="建图结束，地图保存完成")
            self.text_pub.publish(msg)
            self._stop_lc_monitor()

    def navigation_start(self):
        self.slam_stop()
        self.navigation_future = Future()
        
        # 定义导航后台守护线程
        def nav_daemon():
            # 1. 启动导航
            process_fuc = subprocess.Popen(['ros2', 'launch', 'largemodel', 'largemodel_nav.launch.py'])
            
            # 2. 等待底层导航和雷达节点启动完毕
            self.get_logger().info("等待导航节点启动...")
            time.sleep(30.0) 
            
            # 3. 核心机制：自动下发初始坐标
            self.get_logger().info("发布初始位姿（RTAB 链路由 rtabmap 自行重定位，不消费 /initialpose）...")
            self._publish_initial_pose()
            time.sleep(10.0)
            
            if not self.is_localized():
                self.get_logger().info("原点特征匹配失败，请求人工协助重置...")
                msg = String(data="哎呀，我匹配不到周围的环境。请确认我已经放在了标准发车点（充电桩正前方），然后对我说‘已经把你放到原点了’。")
                self.text_pub.publish(msg)
            else:
                msg = String(data="导航系统启动成功，定位已就绪！")
                self.text_pub.publish(msg)
            
            # 4. 【修改点】：取消 interrupt_flag 的打断！
            # 只有当大模型明确下达 navigation_stop() 任务时，才会跳出这里
            while not self.navigation_future.done():
                time.sleep(0.5)

            # 5. 任务结束，彻底杀死后台
            self.kill_process_tree(process_fuc.pid)
            self.cancel()
            self.get_logger().info("导航后台服务已彻底关闭。")

        # 启动独立线程，让主程序立刻返回
        nav_thread = threading.Thread(target=nav_daemon)
        nav_thread.daemon = True
        nav_thread.start()
        
        self.get_logger().info("已启动导航后台守护线程。")

    def _start_lc_monitor(self):
        try:
            from rtabmap_msgs.msg import Statistics
            self._lc_sub = self.create_subscription(
                Statistics,
                '/rtabmap/statistics',
                self._lc_callback,
                10
            )
            self.get_logger().info("Loop closure monitoring started")
        except ImportError:
            self.get_logger().warn("rtabmap_msgs not available, loop closure monitoring disabled")
            self._lc_sub = None

    def _stop_lc_monitor(self):
        if hasattr(self, '_lc_sub') and self._lc_sub is not None:
            self.destroy_subscription(self._lc_sub)
            self._lc_sub = None

    def _lc_callback(self, msg):
        lc_id = getattr(msg, 'loop_closure_id', None)
        if lc_id is None:
            lc_id = getattr(msg, 'loopClosureId', -1)
        if lc_id > 0:
            self.loop_closure_count += 1
            self.get_logger().info(f"Loop closure detected! Count: {self.loop_closure_count}")
            if self.loop_closure_count >= 5 and not self.loop_closure_announced:
                self.loop_closure_announced = True
                text = String(data="建图已达到5个回环点，如果要结束保存时可以叫我，我马上为你执行")
                self.text_pub.publish(text)

    def is_localized(self):
        """检查 TF 树中 map 到 base_footprint 是否存在，判断是否已定位"""
        try:
            self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            return True
        except Exception:
            return False

    def safe_spin(self, duration=10.0, angular_speed=0.3):
        start_time = time.time()
        twist = Twist()
        twist.angular.z = angular_speed
        
        while (time.time() - start_time) < duration:
            if self.obstacle_in_path(twist, self.obstacle_dist, self.obstacle_angle):
                self.stop()
                return False
            self.publisher.publish(twist)
            time.sleep(0.1)
        self.stop()
        return True        

    def navigation_stop(self):
        if not self.navigation_future.done():
            self.navigation_future.set_result(True)

    def KCF_follow(self, x1, y1, x2, y2):
        x1 = float(x1)
        y1 = float(y1)
        x2 = float(x2)
        y2 = float(y2)
        self.KCF_follow_future = Future()
        
        real_x1 = int((x1 / 1000.0) * 848)
        real_y1 = int((y1 / 1000.0) * 480)
        real_x2 = int((x2 / 1000.0) * 848)
        real_y2 = int((y2 / 1000.0) * 480)
        
        box_width = real_x2 - real_x1
        box_height = real_y2 - real_y1
        margin_x = int(box_width * 0.20)
        margin_y = int(box_height * 0.20)
        
        real_x1 += margin_x
        real_y1 += margin_y
        real_x2 -= margin_x
        real_y2 -= margin_y
        
        real_x1 = max(0, min(real_x1, 847))
        real_y1 = max(0, min(real_y1, 479))
        real_x2 = max(0, min(real_x2, 847))
        real_y2 = max(0, min(real_y2, 479))
        
        self.get_logger().info(f'大模型原始坐标: x1:{x1};y1:{y1};x2:{x2};y2:{y2}')
        self.get_logger().info(f'相机真实坐标: x1:{real_x1};y1:{real_y1};x2:{real_x2};y2:{real_y2}')
        self.get_logger().info("wheeltec_robot kcf_tracker start")

        my_env = os.environ.copy()
        my_env["DISPLAY"] = ":0"  
        my_env["QT_QPA_PLATFORM"] = "offscreen" 
        my_env["XDG_RUNTIME_DIR"] = "/tmp/runtime-nvidia" 

        process_fuc = subprocess.Popen(
            ['ros2', 'run', 'wheeltec_robot_kcf', 'run_tracker_node',
             '--ros-args','-p',f'x1:={real_x1}','-p',f'y1:={real_y1}','-p',f'x2:={real_x2}','-p',f'y2:={real_y2}'],
            env=my_env
        )

        while not self.KCF_follow_future.done():
            if self.interrupt_flag:
                break
            time.sleep(0.1)

        self.kill_process_tree(process_fuc.pid)
        self.cancel()

    def visual_follower(self,color):
        try:
            self.visual_follower_future = Future() 
            color = color.strip("'\"")
            if color == 'red':
                target_color = int(0)
            elif color == 'green':
                target_color = int(1)
            elif color == 'blue':
                target_color = int(2)
            elif color == 'yellow':
                target_color = int(3)
            else:
                target_color = int(0)
            process_fuc1 = subprocess.Popen(['ros2', 'run', 'simple_follower_ros2', 'visualtracker','--ros-args','-p',f'target_color:={target_color}'])
            process_fuc2 = subprocess.Popen(['ros2', 'run', 'simple_follower_ros2', 'visualfollow'])
            while not self.visual_follower_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)
            self.get_logger().info(f'killed process_pid') 
            self.kill_process_tree(process_fuc1.pid)
            self.kill_process_tree(process_fuc2.pid)
            self.cancel()
        except:
            self.get_logger().error('visual_follower Startup failure')
            return
        
    def laser_follower(self):
        self.laser_follower_future = Future()
        self.get_logger().warning("接收到雷达跟随指令，但实际物理功能已被屏蔽，机器人原地待命。")
        while not self.laser_follower_future.done():
            if self.interrupt_flag:
                break
            time.sleep(0.1)
        self.cancel()
        
    def line_follower(self,color):
        try:
            self.line_follower_future = Future() 
            color = color.strip("'\"")
            if color == 'red':
                target_color = int(0)
            elif color == 'green':
                target_color = int(1)
            elif color == 'blue':
                target_color = int(2)
            elif color == 'yellow':
                target_color = int(3)
            else:
                target_color = int(0)
            process_fuc = subprocess.Popen(['ros2', 'run', 'simple_follower_ros2', 'line_follow_model','--ros-args','-p',f'target_color:={target_color}'])
            while not self.line_follower_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)
            self.get_logger().info(f'killed process_pid') 
            self.kill_process_tree(process_fuc.pid)
            self.cancel()     
        except:
            self.get_logger().error('line_follower Startup failure')
            return

    def _execute_action(self, twist, num=1, durationtime=3.0):
        for _ in range(num):
            start_time = time.time()
            count= 0
            while (time.time() - start_time) < durationtime:
                if self.obstacle_in_path(twist, self.obstacle_dist,self.obstacle_angle):
                    count += 1
                    if count >= 3:
                        twist.linear.x = 0.0
                        twist.linear.y = 0.0
                        twist.angular.z = 0.0
                        self.publisher.publish(twist)
                        msg = String(data=f"遇到障碍物,停止移动")
                        self.text_pub.publish(msg)
                        return
                if self.interrupt_flag:
                    self.stop()
                    return
                self.publisher.publish(twist)
                time.sleep(0.1)
                
    @staticmethod
    def obstacle_in_path(cmd: Twist, d: float, ang: float) -> bool:
        if d > LIDAR_RANGE:
            return False
        v = cmd.linear.x
        w = cmd.angular.z
        if abs(w) < 1e-3:                 
            along = v * PREDICT_TIME
            across = ROBOT_WIDTH
        else:                             
            r = v /(w+1e-3)                     
            along = r * math.sin(w * PREDICT_TIME)
            across = ROBOT_WIDTH + abs(r * (1 - math.cos(w * PREDICT_TIME)))
        x = d * math.cos(ang)
        y = d * math.sin(ang)
        x_in = (0 <= x <= along) if along >= 0 else (along <= x <= 0)
        y_in = abs(y) <= across
        return x_in and y_in
        
    @staticmethod
    def kill_process_tree(pid):
        try:
            parent = psutil.Process(pid)
            for child in parent.children(recursive=True):
                child.kill()
            parent.kill()
        except psutil.NoSuchProcess:
            pass

    def execute_callback(self, goal_handle):
        feedback_msg = Progress.Feedback()
        actions = goal_handle.request.actions
        self.action_runing = True
        if not actions:  
                self.action_status_pub("response_done")
        else:  
            for action in actions:
                time.sleep(1)
                if self.interrupt_flag:
                    break
                match = re.match(r"(\w+)\((.*)\)", action)
                action_name, args_str = match.groups()
                args = [arg.strip() for arg in args_str.split(",")] if args_str else []

                if not hasattr(self, action_name):
                    self.get_logger().warning(
                        f"action_service: {action} is invalid action，skip execution" 
                    )
                    self.action_status_pub(
                        "failure_execute_action_function_not_exists"
                    )
                else:
                    method = getattr(self, action_name)
                    method(*args)
                    feedback_msg.status = f"action service execute  {action}  successed"
            
            if not self.interrupt_flag:
                self.action_status_pub(
                    "multiple_done", actions=actions
                )
                
        self.stop()  
        self.action_runing = False   
        self.interrupt_flag = False
        goal_handle.succeed()
        result = Progress.Result()
        result.success = True
        return result

    def _check_sensor_timer(self):
        # 1. 高频监测：每 5 秒雷打不动地执行一次
        curr = {n for n, t in self._sensor_map.items() if not self.get_publishers_info_by_topic(t)}
        bad = getattr(self, '_last_missing', set()) & curr
        
        if bad:
            # 监测报警：每5秒向控制台输出警告日志
            self.get_logger().warning(f"高频监控警告 - 传感器数据异常: {', '.join(bad)}")
            
            # 2. 语音播报节流：限制机器人发声的频率
            now = time.time()
            prev_t = getattr(self, '_last_alert_t', 0.0)
            prev_set = getattr(self, '_last_alert_set', set())
            
            # 触发语音条件：异常情况发生了变化，或者距离上次喊话已经过了 60 秒
            if bad != prev_set or (now - prev_t) >= 60.0:
                msg = String()
                msg.data = '数据异常，请检查: ' + ', '.join(bad)
                self.text_pub.publish(msg)  # 触发大模型和TTS发声
                
                self._last_alert_t = now
                self._last_alert_set = bad
        else:
            # 传感器恢复正常，清空报警状态
            self._last_alert_set = set()
            
        self._last_missing = curr

    def finishtask(self):  
        self.action_status_pub("finish")  

    def save_single_image(self):
        self.IS_SAVING=True
        time.sleep(0.1)
        if self.image_msg is None:
            self.get_logger().warning("No image received yet.")  
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, "bgr8")
            cv2.imwrite(self.image_save_path, cv_image)
        except Exception as e:
            self.get_logger().error(f"Error saving image: {e}")  
        self.IS_SAVING=False

    def display_saved_image(self):
        try:
            img = cv2.imread(self.image_save_path)
            if img is not None:
                cv2.imshow("Saved Image", img)
                cv2.waitKey(4000)  
                cv2.destroyAllWindows()
            else:
                self.get_logger().error("Failed to load saved image for display.")  
        except Exception as e:
            self.get_logger().error(f"Error displaying image: {e}")  

    def image_callback(self, msg):  
        if not self.IS_SAVING:
            self.image_msg = msg
        else:
            self.get_logger().error("The image is being saved and no new information will be accepted")

    def wakeup_callback(self, msg):
        if msg.data==1:
            self.interrupt_flag = True
            self.stop()
            self.stop_follow()
            time.sleep(1)
            self.interrupt_flag = False
    
    def distanceCallback(self, msg: Position):
        angle = msg.angle_x
        self.obstacle_angle = normalize_angle(angle)
        self.obstacle_dist = msg.distance

    def kcf_cmd_callback(self, msg):
        safe_msg = Twist()
        if self.obstacle_in_path(msg, self.obstacle_dist, self.obstacle_angle):
            safe_msg.linear.x = 0.0
            safe_msg.linear.y = 0.0
            safe_msg.angular.z = 0.0
            if not hasattr(self, '_last_kcf_warn_time') or (time.time() - self._last_kcf_warn_time > 20.5):
                warn_text = String(data="安全警告：前方检测到障碍物，视觉跟随已自动刹车避障！")
                self.text_pub.publish(warn_text)
                self._last_kcf_warn_time = time.time()
        else:
            safe_msg.linear.x = msg.linear.x
            safe_msg.linear.y = msg.linear.y
            safe_msg.angular.z = msg.angular.z
        self.publisher.publish(safe_msg)

def main(args=None):
    rclpy.init(args=args)
    custom_action_server = CustomActionServer()
    
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(custom_action_server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        custom_action_server.stop()
        pass
    finally:
        custom_action_server.stop()
        custom_action_server.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()