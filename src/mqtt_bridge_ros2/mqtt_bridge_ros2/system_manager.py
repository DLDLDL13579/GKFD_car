#!/usr/bin/env python3
"""
system_manager — 机器人"大管家"节点

将原始的 mqtt_bridge_node 重构为完整的功能管理系统：
 - 统一管理所有机器人功能（导航、建图、KCF、跟随、摄像头、大模型等）的生命周期
 - 通过 MQTT 与远端前端双向通信
 - 语音交互桥接：MQTT → ROS2 voice_words → action_service → LLM
 - 定时状态上报，支持 RViz2 级别的远程控制
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool, Int8
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np
import cv2
import base64
import paho.mqtt.client as mqtt
import json
import math
import subprocess
import os
import time
import signal
import threading
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# ════════════════════════════════════════════
#  功能注册表 — 所有可管理的功能单元
# ════════════════════════════════════════════
FEATURE_REGISTRY = {
    # ── 基础功能(自动启动,不可启停) ──
    "base": {
        "name": "底盘驱动",
        "voice_start": None,  # 开机自启,不可启停
        "voice_stop":  None,
        "description": "STM32 串口通信与底盘驱动",
        "category": "base",
        "auto_start": True,
        "startable": False,
    },
    "camera": {
        "name": "摄像头",
        "voice_start": None, "voice_stop": None,
        "description": "RGB 相机驱动",
        "category": "sensor",
        "auto_start": True,
        "startable": False,
    },
    "mic": {
        "name": "麦克风阵列",
        "voice_start": None, "voice_stop": None,
        "description": "语音采集",
        "category": "sensor",
        "auto_start": True,
        "startable": False,
    },
    "webrtc": {
        "name": "视频推流",
        "voice_start": None, "voice_stop": None,
        "description": "WebRTC 摄像头视频流",
        "category": "sensor",
        "auto_start": True,  # mqtt_bridge_ros2 自启
        "startable": False,
    },
    "web_console": {
        "name": "Web 控制台",
        "voice_start": None, "voice_stop": None,
        "description": "rosbridge + Web 可视化",
        "category": "utility",
        "auto_start": True,
        "startable": False,
    },
    "largemodel": {
        "name": "大模型 AI",
        "voice_start": None, "voice_stop": None,
        "description": "语音交互+大模型任务执行",
        "category": "ai",
        "auto_start": True,
        "startable": False,
    },
    # ── 语音可启停的功能(统一走 LLM → action_service) ──
    "slam": {
        "name": "SLAM 建图",
        "voice_start": "开启建图",
        "voice_stop":  "结束建图",
        "description": "同步定位与地图构建(RTAB-Map)",
        "category": "perception",
        "auto_start": False,
        "startable": True,
    },
    "nav2": {
        "name": "自主导航",
        "voice_start": "开启导航",
        "voice_stop":  "结束导航",
        "description": "Nav2 路径规划与导航",
        "category": "navigation",
        "auto_start": False,
        "startable": True,
        "depends": ["slam"],
    },
    "kcf": {
        "name": "KCF 视觉跟随",
        "voice_start": "开始 KCF 视觉跟随",
        "voice_stop":  "结束 KCF 视觉跟随",
        "description": "KCF 目标跟踪与跟随",
        "category": "follow",
        "auto_start": False,
        "startable": True,
    },
    "path_follow": {
        "name": "路径跟随",
        "voice_start": "开始路径跟随",
        "voice_stop":  "结束路径跟随",
        "description": "沿预设路径行驶",
        "category": "navigation",
        "auto_start": False,
        "startable": True,
    },
    "align": {
        "name": "回充对接",
        "voice_start": "去充电",
        "voice_stop":  "结束充电",
        "description": "视觉引导充电桩自动对接(由 LLM/auto_charge 触发)",
        "category": "utility",
        "auto_start": False,
        "startable": True,
    },
}


ROS_NODE_TO_FEATURE = {
    # ── 底盘(注意:真节点名是 /wheeltec_robot,不是 /base_serial) ──
    "/wheeltec_robot":        "base",
    "/ekf_filter_node":       "base",   # robot_localization
    "/base_to_camera_tf":     "base",
    "/base_to_laser_tf":      "base",
    "/base_to_gyro":          "base",
    "/base_to_link":          "base",
    "/lslidar_driver_node":   "base",   # 雷达也算"基础"类
    # ── 摄像头 ──
    "/camera/camera":         "camera",
    "/realsense2_camera_node":"camera",
    # ── 麦克风 ──
    "/wheeltec_mic":          "mic",
    "/wheeltec_mic_aiui":     "mic",
    "/aiui_node":             "mic",
    # ── SLAM 建图 ──
    # /rtabmap 在 SLAM 和 Nav2 两种模式都跑,需要在 _poll_ros_nodes 里消歧
    "/rtabmap":               "slam",
    "/slam_toolbox":          "slam",
    "/rgbd_sync":             "slam",
    # ── 自主导航(nav2_bringup 一堆节点) ──
    "/nav2_container":        "nav2",
    "/planner_server":        "nav2",
    "/controller_server":     "nav2",
    "/bt_navigator":          "nav2",
    "/smoother_server":       "nav2",
    "/velocity_smoother":     "nav2",
    "/behavior_server":       "nav2",
    "/global_costmap/global_costmap":   "nav2",
    "/local_costmap/local_costmap":     "nav2",
    "/lifecycle_manager_navigation":   "nav2",
    # ── KCF 视觉跟随 ──
    "/kcf_tracker_node":      "kcf",
    # ── 大模型 ──
    "/model_service":         "largemodel",
    "/action_service":        "largemodel",
    # ── Web 控制台 ──
    "/web_video_server":      "web_console",
    "/rosbridge_websocket":   "web_console",
    "/rosapi":                "web_console",
    # ── WebRTC 推流 ──
    "/webrtc_camera_node":    "webrtc",
    "/system_manager":        "mqtt_bridge",  # 自己也算
}


class SystemManager(Node):
    """机器人功能总管家 — 功能管理 + MQTT 桥接 + RViz 远程操作 + 语音桥接"""

    def __init__(self):
        super().__init__("system_manager")

        # ═══ 参数 ═══
        self.declare_parameter("mqtt_broker", "192.168.1.67")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("mqtt_client_id", "jetson_robot")
        self.declare_parameter("topic_prefix", "robot_0")
        self.declare_parameter("status_interval", 0.2)  # 修改为 0.2 秒 (5Hz)

        self.broker = self.get_parameter("mqtt_broker").value
        self.port = self.get_parameter("mqtt_port").value
        self.client_id = self.get_parameter("mqtt_client_id").value
        self.prefix = self.get_parameter("topic_prefix").value
        self.status_interval = self.get_parameter("status_interval").value

        # ═══ 进程管理 ═══
        self.processes: dict[str, subprocess.Popen] = {}
        self.process_status: dict[str, str] = {}  # idle | running | crashed
        # 重要:不预先标任何功能为 running
        # 一切以 ROS 节点列表的实际探测为准 — 前端标绿只能来自 _poll_ros_nodes
        self._init_feature_status()

        # ═══ MQTT ═══
        self.mqtt = mqtt.Client(client_id=self.client_id)
        self.mqtt.on_connect = self._on_mqtt_connect
        self.mqtt.on_disconnect = self._on_mqtt_disconnect
        self.mqtt.on_message = self._on_mqtt_message
        self._mqtt_connected = False

        try:
            self.mqtt.connect_async(self.broker, self.port, 60)
            self.mqtt.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT connect failed: {e}")

        # ═══ ROS2 发布者 ═══
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self._initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._voice_pub = self.create_publisher(String, "voice_words", 1)
        self._debug_pub = self.create_publisher(String, "/debug_cmd", 1)
        self._map_pgm_pub = self.create_publisher(Image, "/map_pgm", 1)

        # ═══ ROS2 订阅者 ═══
        self._odom_sub = self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self._odom_combined_sub = self.create_subscription(Odometry, "/odom_combined", self._odom_combined_cb, 10)
        # ROS 2 地图专用 QoS（TRANSIENT_LOCAL 让后入网节点也能拿到旧地图）
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._map_sub = self.create_subscription(OccupancyGrid, "/map", self._map_cb, map_qos)
        self._volt_sub = self.create_subscription(
            Float32, "/PowerVoltage", self._voltage_cb, 10
        )
        self._charge_sub = self.create_subscription(
            Bool, "/robot_charging_flag", self._charge_cb, 10
        )
        self._feedback_sub = self.create_subscription(
            String, "feedback_words", self._feedback_cb, 10
        )

        # ═══ TF 监听器 ═══
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # ═══ 状态缓存 ═══
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_vx = 0.0
        self._odom_vz = 0.0
        self._odom_yaw = 0.0
        self._battery = 0.0
        self._charging = False
        self._last_llm_feedback = ""

        # ═══ 状态定时器 ═══
        self._status_timer = self.create_timer(self.status_interval, self._report_status)
        self._proc_watchdog = self.create_timer(5.0, self._watchdog_check)
        # ROS 节点轮询(5s)把实际在跑的节点映射到 features
        self._ros_poll_timer = self.create_timer(5.0, self._poll_ros_nodes)
        # 启动后立即跑一次(不等 5s),让前端一连上 MQTT 就能看到准确状态
        self._initial_poll_done = False

        self.get_logger().info(
            f"SystemManager started — prefix={self.prefix}, "
            f"broker={self.broker}:{self.port}"
        )

    # ────────────────────────────────────────
    #  功能初始化
    # ────────────────────────────────────────
    def _init_feature_status(self):
        for key in FEATURE_REGISTRY:
            self.process_status[key] = "idle"

    # ────────────────────────────────────────
    #  MQTT 回调
    # ────────────────────────────────────────
    def _on_mqtt_disconnect(self, client, ud, rc):
        self._mqtt_connected = False
        self.get_logger().warn("MQTT disconnected, rc=" + str(rc))

    def _on_mqtt_connect(self, client, ud, flags, rc):
        if rc == 0:
            self._mqtt_connected = True
            self.get_logger().info("MQTT connected")
            # 订阅主题列表
            topics = [
                (f"{self.prefix}/cmd_vel", 0),
                (f"{self.prefix}/goal_pose", 0),
                (f"{self.prefix}/sys_cmd", 0),
                (f"{self.prefix}/initialpose", 0),
                (f"{self.prefix}/voice_input", 0),
                (f"{self.prefix}/debug_cmd", 0),
                (f"{self.prefix}/webrtc_answer", 0),
                
                (f"{self.prefix}/cmd/map_update", 0),
                ("robot/all/cmd/broadcast", 0),
                (f"{self.prefix}/cmd/voice", 0),          # 前端按钮 = 本地语音,统一进 voice pipeline
            ]
            self.mqtt.subscribe(topics)
            self.get_logger().info(f"Subscribed to {len(topics)} topics under {self.prefix}")
        else:
            self._mqtt_connected = False
            self.get_logger().error(f"MQTT rc={rc}")

    def _on_mqtt_message(self, client, ud, msg):
        topic = msg.topic
        try:
            payload = msg.payload.decode("utf-8")
            data = json.loads(payload) if payload.startswith("{") else {"_raw": payload}

            # ── 运动控制 ──
            if topic == f"{self.prefix}/cmd_vel":
                twist = Twist()
                twist.linear.x = float(data.get("linear_x", 0.0))
                twist.angular.z = float(data.get("angular_z", 0.0))
                self._cmd_pub.publish(twist)

            # ── 导航目标 ──
            elif topic == f"{self.prefix}/goal_pose":
                goal = PoseStamped()
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.header.frame_id = "map"
                goal.pose.position.x = float(data.get("x", 0.0))
                goal.pose.position.y = float(data.get("y", 0.0))
                goal.pose.orientation.z = float(data.get("oz", 0.0))
                goal.pose.orientation.w = float(data.get("ow", 1.0))
                self._goal_pub.publish(goal)

            # ── 初始位姿 ──
            elif topic == f"{self.prefix}/initialpose":
                pose = PoseWithCovarianceStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.pose.position.x = float(data.get("x", 0.0))
                pose.pose.pose.position.y = float(data.get("y", 0.0))
                pose.pose.pose.orientation.z = float(data.get("oz", 0.0))
                pose.pose.pose.orientation.w = float(data.get("ow", 1.0))
                # 默认协方差
                pose.pose.covariance[0] = 0.25
                pose.pose.covariance[7] = 0.25
                pose.pose.covariance[35] = 0.0685
                self._initpose_pub.publish(pose)
                self.get_logger().info(f"Set initial pose: ({data.get('x')}, {data.get('y')})")

            # ── 功能启停（增强版 sys_cmd）──
            elif topic == f"{self.prefix}/sys_cmd":
                action = data.get("action")  # start | stop | restart
                target = data.get("target")
                params = data.get("params", {})
                self._handle_feature_cmd(action, target, params)

            # ── 语音输入 → 桥接到大模型 ──
            elif topic == f"{self.prefix}/voice_input":
                text = data.get("text", data.get("_raw", ""))
                if text:
                    msg_out = String(data=text)
                    self._voice_pub.publish(msg_out)
                    self.get_logger().info(f"Voice input → LLM: {text[:60]}")

            # 👇 前端按钮 = 本地语音:统一塞进 voice_words,LLM 解析后调 action_service
            elif topic == f"{self.prefix}/cmd/voice":
                text = data.get("text", data.get("_raw", ""))
                if text:
                    self._voice_pub.publish(String(data=text))
                    self.get_logger().info(f"cmd/voice → LLM: {text[:60]}")
                    self._mqtt_pub("/status/info", {"msg": f"语音指令已转发: {text[:60]}"})

            elif topic == f"{self.prefix}/debug_cmd":
                cmd = data.get("cmd", data.get("_raw", ""))
                if cmd:
                    self._debug_pub.publish(String(data=cmd))

            # 👇 匹配测试路径或正式路径，去执行地图更新👇
            elif topic in [f"{self.prefix}/cmd/map_update", "robot/all/cmd/broadcast"]:
                self._handle_map_update(data)

        except json.JSONDecodeError:
            self.get_logger().warn(f"Non-JSON on {topic}: {msg.payload[:80]}")
        except Exception as e:
            self.get_logger().error(f"Handle {topic}: {e}")

    # ────────────────────────────────────────
    #  功能启停核心逻辑 — 全部走 voice pipeline
    # ────────────────────────────────────────
    def _handle_feature_cmd(self, action: str, target: str, params: dict):
        if target not in FEATURE_REGISTRY:
            self._mqtt_pub("/status/error", {"error": f"Unknown feature: {target}"})
            return

        feat = FEATURE_REGISTRY[target]
        if not feat.get("startable", False):
            self.get_logger().warning(f"{target} 不能由 MQTT 启停(开机自启或由 LLM 触发)")
            self._mqtt_pub("/status/error", {"error": f"{target} 不可启停,开机即在跑或由 LLM 触发"})
            return

        if action == "start":
            text = feat.get("voice_start")
        elif action == "stop":
            text = feat.get("voice_stop")
        elif action == "restart":
            for t in (feat.get("voice_stop"), feat.get("voice_start")):
                if t:
                    self._voice_pub.publish(String(data=t))
                    time.sleep(0.5)
            self.get_logger().info(f"Feature {target} restart → voice pipeline")
            return
        else:
            self.get_logger().warning(f"Unknown action: {action}")
            return

        if not text:
            self.get_logger().warning(f"{target} 没有定义 {action} 的语音指令")
            return

        self._voice_pub.publish(String(data=text))
        self.get_logger().info(f"Feature {target} {action} → voice: '{text}'")
        self._mqtt_pub("/status/info", {"msg": f"已请求 {action} {target}(语音管线: '{text}')"})

    # ────────────────────────────────────────
    #  进程看门狗
    # ────────────────────────────────────────
    def _watchdog_check(self):
        pass

    def _poll_ros_nodes(self):
        slam_proc_running = False
        nav_proc_running = False
        try:
            ps_out = subprocess.check_output(
                ["bash", "-c",
                 "ps -eo pid,cmd | grep -E 'largemodel_(slam|nav)\\.launch\\.py'"
                 " | grep -v 'grep' | grep -v 'bash -c' || true"],
                stderr=subprocess.DEVNULL, timeout=2,
            ).decode(errors="ignore")
            for line in ps_out.splitlines():
                low = line.lower()
                if "largemodel_slam.launch.py" in low:
                    slam_proc_running = True
                if "largemodel_nav.launch.py" in low:
                    nav_proc_running = True
        except Exception:
            pass

        try:
            out = subprocess.check_output(
                ["ros2", "node", "list"],
                stderr=subprocess.DEVNULL, timeout=3,
            ).decode().strip().splitlines()
        except Exception:
            return
        raw_running = set()
        for line in out:
            node = line.strip()
            if not node: continue
            feat = ROS_NODE_TO_FEATURE.get(node)
            if feat:
                raw_running.add(feat); continue
            for pat, f in ROS_NODE_TO_FEATURE.items():
                if node == pat or node.endswith("/" + pat.lstrip("/")):
                    raw_running.add(f); break

        running = set(raw_running)
        if nav_proc_running and slam_proc_running:
            running.discard("slam")
        elif nav_proc_running:
            running.discard("slam")
            running.add("nav2")
        elif slam_proc_running:
            running.discard("nav2")
            running.add("slam")
        else:
            if "nav2" in raw_running:
                running.discard("slam")

        changed = False
        for feat_id, status in list(self.process_status.items()):
            present = feat_id in running
            if present and status in ("idle", "crashed"):
                self.process_status[feat_id] = "running"
                changed = True
            elif not present and status == "running":
                if feat_id not in self.processes:
                    self.process_status[feat_id] = "idle"
                    changed = True
        if changed:
            self.get_logger().debug(f"ROS-poll running features: {running}")

    # ────────────────────────────────────────
    #  状态收集 & 上报
    # ────────────────────────────────────────
    def _report_status(self):
        if not self._mqtt_connected:
            return

        if not self._initial_poll_done:
            self._poll_ros_nodes()
            self._initial_poll_done = True

        features = {}
        for key, feat in FEATURE_REGISTRY.items():
            status = self.process_status.get(key, "idle")
            features[key] = {
                "status": status,
                "name": feat["name"],
                "category": feat.get("category", ""),
                "auto_start": feat.get("auto_start", False),
                "startable":  feat.get("startable", False),
                "voice_start": feat.get("voice_start"),
                "voice_stop":  feat.get("voice_stop"),
            }

        try:
            trans = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            self._odom_x = trans.transform.translation.x
            self._odom_y = trans.transform.translation.y
            q = trans.transform.rotation
            self._odom_yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
        except Exception:
            pass

        robot = {
            "position": {"x": round(self._odom_x, 3), "y": round(self._odom_y, 3),
                         "yaw": round(self._odom_yaw, 4),
                         "yaw_deg": round(math.degrees(self._odom_yaw), 1)},
            "velocity": {"linear": round(self._odom_vx, 3), "angular": round(self._odom_vz, 3)},
            "battery": round(self._battery, 2),
            "charging": self._charging,
            "online": True,
        }

        report = {
            "features": features,
            "robot": robot,
            "llm_feedback": self._last_llm_feedback,
            "timestamp": time.time(),
        }

        self._mqtt_pub("/status/all", report)

    # ────────────────────────────────────────
    #  ROS2 订阅回调
    # ────────────────────────────────────────
    def _map_cb(self, msg: OccupancyGrid):
        try:
            w = msg.info.width
            h = msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((h, w))
            img = np.zeros((h, w), dtype=np.uint8)
            img[data == -1] = 128
            img[data == 0]  = 254
            img[data >= 1]  = 0
            
            _, buf = cv2.imencode(".png", img)
            b64_str = base64.b64encode(buf).decode("utf-8")
            if self._mqtt_connected:
                self._mqtt_pub("/map_data", {
                    "image": b64_str,
                    "resolution": msg.info.resolution,
                    "width": w, "height": h,
                    "origin_x": msg.info.origin.position.x,
                    "origin_y": msg.info.origin.position.y,
                }, retain=True)

            img_msg = Image()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "map"
            img_msg.height = h
            img_msg.width = w
            img_msg.encoding = "mono8"
            img_msg.is_bigendian = 0
            img_msg.step = w
            img_msg.data = img.tobytes()
            self._map_pgm_pub.publish(img_msg)

            # 直接转发 /map 原始 OccupancyGrid.data(int8[]) 到 MQTT
            if self._mqtt_connected:
                raw_b64 = base64.b64encode(bytes(msg.data)).decode("utf-8")
                self._mqtt_pub("/map_pgm", {
                    "header": {
                        "stamp": {
                            "sec": msg.header.stamp.sec,
                            "nanosec": msg.header.stamp.nanosec
                        },
                        "frame_id": msg.header.frame_id
                    },
                    "info": {
                        "map_load_time": {
                            "sec": msg.info.map_load_time.sec,
                            "nanosec": msg.info.map_load_time.nanosec
                        },
                        "resolution": msg.info.resolution,
                        "width": msg.info.width,
                        "height": msg.info.height,
                        "origin": {
                            "position": {
                                "x": msg.info.origin.position.x,
                                "y": msg.info.origin.position.y,
                                "z": msg.info.origin.position.z
                            },
                            "orientation": {
                                "x": msg.info.origin.orientation.x,
                                "y": msg.info.origin.orientation.y,
                                "z": msg.info.origin.orientation.z,
                                "w": msg.info.origin.orientation.w
                            }
                        }
                    },
                    "image": raw_b64  
                }, retain=True)
        except Exception as e:
            self.get_logger().error(f"处理地图数据时出错: {e}")

    def _odom_cb(self, msg: Odometry):
        # 彻底删除位置更新,防止覆盖 TF 坐标
        self._odom_vx = msg.twist.twist.linear.x
        self._odom_vz = msg.twist.twist.angular.z

    def _odom_combined_cb(self, msg: Odometry):
        # 彻底删除位置更新,防止覆盖 TF 坐标
        self._odom_vx = msg.twist.twist.linear.x
        self._odom_vz = msg.twist.twist.angular.z

    @staticmethod
    def _quat_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def _voltage_cb(self, msg: Float32):
        self._battery = msg.data

    def _charge_cb(self, msg: Bool):
        self._charging = msg.data

    def _feedback_cb(self, msg: String):
        self._last_llm_feedback = msg.data
        if self._mqtt_connected:
            self._mqtt_pub("/voice/response", {"text": msg.data})

    def _mqtt_pub(self, topic_suffix: str, data: dict, qos=0, retain=False):
        full_topic = f"{self.prefix}{topic_suffix}"
        try:
            self.mqtt.publish(full_topic, json.dumps(data), qos=qos, retain=retain)
        except Exception:
            pass

    # 👇 【彻底修复：双模解析、修正颜色、保留方向的完整地图重载模块】👇
    def _handle_map_update(self, payload: dict):
        self.get_logger().info("收到远端地图更新数据，开始解析与重载...")
        try:
            # ==== 1. 兼容并统一数据格式 ====
            if "image" in payload and "info" in payload:
                # 新标准格式 (优先处理：Base64 + info 结构)
                info = payload["info"]
                width = info["width"]
                height = info["height"]
                resolution = info["resolution"]
                
                # 【修改点】：使用 .get() 设定默认值，兼容前端缺少字段的情况
                origin_pos = info["origin"].get("position", {"x": 0.0, "y": 0.0, "z": 0.0})
                origin_ori = info["origin"].get("orientation", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})
                
                origin_x = origin_pos["x"]
                origin_y = origin_pos["y"]
                
                # 提取原始地图的旋转角度 (Yaw)
                yaw = self._quat_to_yaw(origin_ori["x"], origin_ori["y"], origin_ori["z"], origin_ori["w"])
                
                # 解码 Base64 数组
                raw_bytes = base64.b64decode(payload["image"])
                grid = np.frombuffer(raw_bytes, dtype=np.int8).reshape((height, width))
                
            elif "map" in payload and "data" in payload["map"]:
                map_info = payload["map"]
                width = map_info["width"]
                height = map_info["height"]
                resolution = map_info.get("resolution", 0.05)
                origin_x = map_info.get("origin_x", 0.0)
                origin_y = map_info.get("origin_y", 0.0)
                yaw = 0.0 # 旧格式丢失了方向，只能默认 0° 兜底
                
                grid = np.array(map_info["data"], dtype=np.int8).reshape((height, width))
            else:
                self.get_logger().error("数据格式错误：无法识别地图 Payload")
                return

            # ==== 2. 颜色阈值映射 (修复颜色丢失问题) ====
            img = np.zeros((height, width), dtype=np.uint8)
            img[grid == -1] = 128  # 核心：将未知区域改为128(标准灰)，完美落在未知阈值之间
            img[grid == 0]  = 254  # 空白安全区域 (白色)
            img[grid >= 1]  = 0    # 障碍物区域 (黑色)

            # ==== 3. 翻转 Y 轴，对齐 ROS 坐标系 ====
            img = cv2.flip(img, 0)

            # ==== 4. 覆盖真实物理目录 ====
            map_dir = "/home/nvidia/wheeltec_ros2/src/wheeltec_robot_rtab"
            os.makedirs(map_dir, exist_ok=True)
            pgm_path = os.path.join(map_dir, "my_map.pgm")
            yaml_path = os.path.join(map_dir, "my_map.yaml")

            cv2.imwrite(pgm_path, img)

            # ==== 5. 保留方向：将 yaw 写入 origin ====
            yaml_content = f"""image: my_map.pgm
mode: trinary
resolution: {resolution}
origin: [{origin_x}, {origin_y}, {yaw}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.19
"""
            with open(yaml_path, 'w') as f:
                f.write(yaml_content)

            self.get_logger().info(f"地图已覆盖: {pgm_path} (Yaw: {yaw})")

            # ==== 6. 触发 Nav2 热更新 ====
            srv_cmd = [
                "ros2", "service", "call", 
                "/map_server/load_map", 
                "nav2_msgs/srv/LoadMap", 
                f"{{map_url: '{yaml_path}'}}"
            ]
            subprocess.Popen(srv_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self._mqtt_pub("/status/info", {"msg": "地图重载成功"})

        except Exception as e:
            self.get_logger().error(f"地图更新失败: {e}")
            
    def shutdown(self):
        self.get_logger().info("Shutting down SystemManager, stopping all features...")
        for target in list(self.processes.keys()):
            self._handle_feature_cmd("stop", target, {})
        self.mqtt.loop_stop()
        self.mqtt.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = SystemManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()