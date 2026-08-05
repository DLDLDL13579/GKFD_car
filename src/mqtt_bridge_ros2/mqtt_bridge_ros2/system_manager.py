#!/usr/bin/env python3
"""
system_manager — 机器人"大管家"节点（统一 JSON 结构版）

将 /status/all 的 JSON 结构调整为：
  - features: array（每项带 id，方便前端遍历）
  - robots:  array（统一 schema，每车一致）
  - 主车 robot_0 即 self（不依赖远端 topic）
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool, Int8
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, BatteryState, LaserScan
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

# ──────────────────────────────────────────────
#  功能注册表
# ──────────────────────────────────────────────
FEATURE_REGISTRY = {
    "base": {
        "name": "底盘驱动", "voice_start": None, "voice_stop": None,
        "description": "STM32 串口通信与底盘驱动",
        "category": "base", "auto_start": True, "startable": False,
    },
    "camera": {
        "name": "摄像头", "voice_start": None, "voice_stop": None,
        "description": "RGB 相机驱动",
        "category": "sensor", "auto_start": True, "startable": False,
    },
    "mic": {
        "name": "麦克风阵列", "voice_start": None, "voice_stop": None,
        "description": "语音采集",
        "category": "sensor", "auto_start": True, "startable": False,
    },
    "webrtc": {
        "name": "视频推流", "voice_start": None, "voice_stop": None,
        "description": "WebRTC 摄像头视频流",
        "category": "sensor", "auto_start": True, "startable": False,
    },
    "web_console": {
        "name": "Web 控制台", "voice_start": None, "voice_stop": None,
        "description": "rosbridge + Web 可视化",
        "category": "utility", "auto_start": True, "startable": False,
    },
    "largemodel": {
        "name": "大模型 AI", "voice_start": None, "voice_stop": None,
        "description": "语音交互+大模型任务执行",
        "category": "ai", "auto_start": True, "startable": False,
    },
    "slam": {
        "name": "SLAM 建图", "voice_start": "开启建图", "voice_stop": "结束建图",
        "description": "同步定位与地图构建(RTAB-Map)",
        "category": "perception", "auto_start": False, "startable": True,
    },
    "nav2": {
        "name": "自主导航", "voice_start": "开启导航", "voice_stop": "结束导航",
        "description": "Nav2 路径规划与导航",
        "category": "navigation", "auto_start": False, "startable": True,
        "depends": ["slam"],
    },
    "kcf": {
        "name": "KCF 视觉跟随", "voice_start": "开始 KCF 视觉跟随", "voice_stop": "结束 KCF 视觉跟随",
        "description": "KCF 目标跟踪与跟随",
        "category": "follow", "auto_start": False, "startable": True,
    },
    "path_follow": {
        "name": "路径跟随", "voice_start": "开始路径跟随", "voice_stop": "结束路径跟随",
        "description": "沿预设路径行驶",
        "category": "navigation", "auto_start": False, "startable": True,
    },
    "align": {
        "name": "回充对接", "voice_start": "去充电", "voice_stop": "结束充电",
        "description": "视觉引导充电桩自动对接",
        "category": "utility", "auto_start": False, "startable": True,
    },
}

ROS_NODE_TO_FEATURE = {
    "/wheeltec_robot": "base", "/ekf_filter_node": "base",
    "/base_to_camera_tf": "base", "/base_to_laser_tf": "base",
    "/base_to_gyro": "base", "/base_to_link": "base",
    "/lslidar_driver_node": "base",
    "/camera/camera": "camera", "/realsense2_camera_node": "camera",
    "/wheeltec_mic": "mic", "/wheeltec_mic_aiui": "mic", "/aiui_node": "mic",
    "/rtabmap": "slam", "/slam_toolbox": "slam", "/rgbd_sync": "slam",
    "/nav2_container": "nav2", "/planner_server": "nav2",
    "/controller_server": "nav2", "/bt_navigator": "nav2",
    "/smoother_server": "nav2", "/velocity_smoother": "nav2",
    "/behavior_server": "nav2",
    "/global_costmap/global_costmap": "nav2",
    "/local_costmap/local_costmap": "nav2",
    "/lifecycle_manager_navigation": "nav2",
    "/kcf_tracker_node": "kcf",
    "/model_service": "largemodel", "/action_service": "largemodel",
    "/web_video_server": "web_console", "/rosbridge_websocket": "web_console",
    "/rosapi": "web_console",
    "/webrtc_camera_node": "webrtc",
    "/system_manager": "mqtt_bridge",
}

# ──────────────────────────────────────────────
#  电池状态码 → 字符串
# ──────────────────────────────────────────────
_POWER_STATUS_MAP = {
    0: "unknown", 1: "charging", 2: "discharging",
    3: "full", 4: "not_charging", 5: "fault",
}
_POWER_HEALTH_MAP = {
    0: "unknown", 1: "good", 2: "overheat",
    3: "dead", 4: "over_voltage", 5: "unspecified_failure",
    6: "cold", 7: "watchdog_timer_expire", 8: "safety_input_timeout",
}


def _bat_status(v: int) -> str:
    return _POWER_STATUS_MAP.get(v, "unknown")

def _bat_health(v: int) -> str:
    return _POWER_HEALTH_MAP.get(v, "unknown")


class SystemManager(Node):

    def __init__(self):
        super().__init__("system_manager")

        self.declare_parameter("mqtt_broker", "192.168.31.175")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("mqtt_client_id", "jetson_robot")
        self.declare_parameter("topic_prefix", "robot_0")
        self.declare_parameter("status_interval", 0.2)

        self.broker = self.get_parameter("mqtt_broker").value
        self.port = self.get_parameter("mqtt_port").value
        self.client_id = self.get_parameter("mqtt_client_id").value
        self.prefix = self.get_parameter("topic_prefix").value
        self.status_interval = self.get_parameter("status_interval").value

        # 次车1 主车代规划参数
        self.declare_parameter("subcar1_robot_radius", 0.20)
        self.declare_parameter("subcar1_safety_margin", 0.10)
        self.declare_parameter("subcar1_max_linear", 0.35)
        self.declare_parameter("subcar1_min_linear", 0.08)
        self.declare_parameter("subcar1_max_angular", 1.2)
        self.declare_parameter("subcar1_lookahead", 0.35)
        self.declare_parameter("subcar1_obstacle_stop", 0.35)
        self.declare_parameter("subcar1_obstacle_slow", 0.8)
        self.declare_parameter("subcar1_arrival_tol", 0.15)

        self.subcar1_radius  = self.get_parameter("subcar1_robot_radius").value
        self.subcar1_margin  = self.get_parameter("subcar1_safety_margin").value
        self.subcar1_max_v   = self.get_parameter("subcar1_max_linear").value
        self.subcar1_min_v   = self.get_parameter("subcar1_min_linear").value
        self.subcar1_max_w   = self.get_parameter("subcar1_max_angular").value
        self.subcar1_look    = self.get_parameter("subcar1_lookahead").value
        self.subcar1_stop_d  = self.get_parameter("subcar1_obstacle_stop").value
        self.subcar1_slow_d  = self.get_parameter("subcar1_obstacle_slow").value
        self.subcar1_arrive  = self.get_parameter("subcar1_arrival_tol").value

        # ── 进程管理 ──
        self.processes: dict[str, subprocess.Popen] = {}
        self.process_status: dict[str, str] = {}
        self._init_feature_status()

        # ── MQTT ──
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

        # ── 发布者 ──
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self._initpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self._voice_pub = self.create_publisher(String, "voice_words", 1)
        self._debug_pub = self.create_publisher(String, "/debug_cmd", 1)
        self._map_pgm_pub = self.create_publisher(Image, "/map_pgm", 1)

        self._r1_cmd_pub = self.create_publisher(Twist, "/robot_1/cmd_vel", 10)
        self._r1_goal_pub = self.create_publisher(PoseStamped, "/robot_1/goal_pose", 10)
        self._r1_initpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/robot_1/initialpose", 10)
        self._r2_cmd_pub = self.create_publisher(Twist, "/robot_2/cmd_vel", 10)
        self._r2_goal_pub = self.create_publisher(PoseStamped, "/robot_2/goal_pose", 10)
        self._r2_initpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/robot_2/initialpose", 10)

        # ── 订阅者 ──
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(Odometry, "/odom_combined", self._odom_combined_cb, 10)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._map_sub = self.create_subscription(OccupancyGrid, "/map", self._map_cb, map_qos)
        self.create_subscription(Float32, "/PowerVoltage", self._voltage_cb, 10)
        self.create_subscription(Bool, "/robot_charging_flag", self._charge_cb, 10)
        self.create_subscription(String, "feedback_words", self._feedback_cb, 10)

        # 次车1 状态订阅
        self.create_subscription(Odometry, "/robot_1/odom", self._r1_odom_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/robot_1/soldier_pose", self._r1_soldier_pose_cb, 10)
        self.create_subscription(BatteryState, "/robot_1/battery_state", self._r1_battery_state_cb, 10)
        self.create_subscription(Odometry, "/robot_2/odom", self._r2_odom_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/robot_2/soldier_pose", self._r2_soldier_pose_cb, 10)
        self.create_subscription(BatteryState, "/robot_2/battery_state", self._r2_battery_state_cb, 10)

        # 次车1 地图+激光
        sub_car_map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self._r1_map_cb, sub_car_map_qos)
        self.create_subscription(LaserScan, "/robot_1/scan", self._r1_scan_cb, 5)

        # ── TF ──
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── 主车状态缓存 ──
        self._odom_x = 0.0; self._odom_y = 0.0
        self._odom_vx = 0.0; self._odom_vz = 0.0
        self._odom_yaw = 0.0
        self._battery = 0.0; self._charging = False
        self._last_llm_feedback = ""

        # ── 次车状态缓存 ──
        self._r1_state = self._new_robot_state()
        self._r2_state = self._new_robot_state()

        # ── 次车1 规划状态 ──
        self._r1_map_msg: OccupancyGrid | None = None
        self._r1_path: list[tuple[float, float]] = []
        self._r1_wp_idx: int = 0
        self._r1_front_obstacle: bool = False
        self._r1_pp_timer = None

        # ── 定时器 ──
        self._status_timer = self.create_timer(self.status_interval, self._report_status)
        self._proc_watchdog = self.create_timer(5.0, self._watchdog_check)
        self._ros_poll_timer = self.create_timer(5.0, self._poll_ros_nodes)
        self._initial_poll_done = False

        self.get_logger().info(
            f"SystemManager started — prefix={self.prefix}, broker={self.broker}:{self.port}"
        )

    # ──────────────────────────────────────────
    #  工具方法
    # ──────────────────────────────────────────
    @staticmethod
    def _new_robot_state() -> dict:
        return {
            "x": 0.0, "y": 0.0, "yaw": 0.0,
            "pose_source": None, "pose_ts": 0.0,
            "vx": 0.0, "vz": 0.0,
            "battery_voltage": None, "battery_current": None,
            "battery_soc": None, "battery_charging": None,
            "battery_status": None, "battery_health": None,
            "battery_ts": 0.0,
        }

    @staticmethod
    def _quat_to_yaw(x, y, z, w) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _init_feature_status(self):
        for key in FEATURE_REGISTRY:
            self.process_status[key] = "idle"

    def _mqtt_pub(self, topic_suffix: str, data: dict, qos=0, retain=False):
        full_topic = f"{self.prefix}{topic_suffix}"
        try:
            self.mqtt.publish(full_topic, json.dumps(data, ensure_ascii=False), qos=qos, retain=retain)
        except Exception:
            pass

    def _iso_now(self) -> str:
        return time.strftime("%Y-%m-%dT%H:%M:%S.000+08:00")

    # ──────────────────────────────────────────
    #  MQTT
    # ──────────────────────────────────────────
    def _on_mqtt_disconnect(self, client, ud, rc):
        self._mqtt_connected = False
        self.get_logger().warn("MQTT disconnected, rc=" + str(rc))

    def _on_mqtt_connect(self, client, ud, flags, rc):
        if rc == 0:
            self._mqtt_connected = True
            self.get_logger().info("MQTT connected")
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
                (f"{self.prefix}/cmd/voice", 0),
                ("robot_1/cmd_vel", 0),
                ("robot_1/goal_pose", 0),
                ("robot_1/initialpose", 0),
                ("robot_1/cancel_goal", 0),
                ("robot_2/cmd_vel", 0),
                ("robot_2/goal_pose", 0),
                ("robot_2/initialpose", 0),
            ]
            self.mqtt.subscribe(topics)
            self.get_logger().info(f"Subscribed to {len(topics)} topics")
        else:
            self._mqtt_connected = False

    def _on_mqtt_message(self, client, ud, msg):
        topic = msg.topic
        try:
            payload = msg.payload.decode("utf-8")
            data = json.loads(payload) if payload.startswith("{") else {"_raw": payload}

            # ── 次车1 指令 ──
            if topic.startswith("robot_1/"):
                if topic == "robot_1/cancel_goal":
                    self._stop_pp()
                    self._r1_cmd_pub.publish(Twist())
                    self._mqtt_pub("/status/info", {"msg": "subcar1 已取消路径并停车"})
                    return
                if "cmd_vel" in topic:
                    t = Twist()
                    t.linear.x = float(data.get("linear_x", 0.0))
                    t.angular.z = float(data.get("angular_z", 0.0))
                    self._r1_cmd_pub.publish(t)
                elif "goal_pose" in topic:
                    self._plan_for_subcar_1(
                        float(data.get("x", 0.0)),
                        float(data.get("y", 0.0))
                    )
                elif "initialpose" in topic:
                    p = PoseWithCovarianceStamped()
                    p.header.stamp = self.get_clock().now().to_msg()
                    p.header.frame_id = "map"
                    p.pose.pose.position.x = float(data.get("x", 0.0))
                    p.pose.pose.position.y = float(data.get("y", 0.0))
                    p.pose.pose.orientation.z = float(data.get("oz", 0.0))
                    p.pose.pose.orientation.w = float(data.get("ow", 1.0))
                    p.pose.covariance[0] = 0.25; p.pose.covariance[7] = 0.25; p.pose.covariance[35] = 0.0685
                    self._r1_initpose_pub.publish(p)
                return

            # ── 次车2 指令 ──
            if topic.startswith("robot_2/"):
                if "cmd_vel" in topic:
                    t = Twist()
                    t.linear.x = float(data.get("linear_x", 0.0))
                    t.angular.z = float(data.get("angular_z", 0.0))
                    self._r2_cmd_pub.publish(t)
                elif "goal_pose" in topic:
                    g = PoseStamped()
                    g.header.stamp = self.get_clock().now().to_msg()
                    g.header.frame_id = "map"
                    g.pose.position.x = float(data.get("x", 0.0))
                    g.pose.position.y = float(data.get("y", 0.0))
                    g.pose.orientation.z = float(data.get("oz", 0.0))
                    g.pose.orientation.w = float(data.get("ow", 1.0))
                    self._r2_goal_pub.publish(g)
                elif "initialpose" in topic:
                    p = PoseWithCovarianceStamped()
                    p.header.stamp = self.get_clock().now().to_msg()
                    p.header.frame_id = "map"
                    p.pose.pose.position.x = float(data.get("x", 0.0))
                    p.pose.pose.position.y = float(data.get("y", 0.0))
                    p.pose.pose.orientation.z = float(data.get("oz", 0.0))
                    p.pose.pose.orientation.w = float(data.get("ow", 1.0))
                    p.pose.covariance[0] = 0.25; p.pose.covariance[7] = 0.25; p.pose.covariance[35] = 0.0685
                    self._r2_initpose_pub.publish(p)
                return

            # ── 主车指令 ──
            if topic == f"{self.prefix}/cmd_vel":
                t = Twist()
                t.linear.x = float(data.get("linear_x", 0.0))
                t.angular.z = float(data.get("angular_z", 0.0))
                self._cmd_pub.publish(t)
            elif topic == f"{self.prefix}/goal_pose":
                g = PoseStamped()
                g.header.stamp = self.get_clock().now().to_msg()
                g.header.frame_id = "map"
                g.pose.position.x = float(data.get("x", 0.0))
                g.pose.position.y = float(data.get("y", 0.0))
                g.pose.orientation.z = float(data.get("oz", 0.0))
                g.pose.orientation.w = float(data.get("ow", 1.0))
                self._goal_pub.publish(g)
            elif topic == f"{self.prefix}/initialpose":
                p = PoseWithCovarianceStamped()
                p.header.stamp = self.get_clock().now().to_msg()
                p.header.frame_id = "map"
                p.pose.pose.position.x = float(data.get("x", 0.0))
                p.pose.pose.position.y = float(data.get("y", 0.0))
                p.pose.pose.orientation.z = float(data.get("oz", 0.0))
                p.pose.pose.orientation.w = float(data.get("ow", 1.0))
                p.pose.covariance[0] = 0.25; p.pose.covariance[7] = 0.25; p.pose.covariance[35] = 0.0685
                self._initpose_pub.publish(p)
            elif topic == f"{self.prefix}/sys_cmd":
                self._handle_feature_cmd(data.get("action"), data.get("target"), data.get("params", {}))
            elif topic == f"{self.prefix}/voice_input":
                text = data.get("text", data.get("_raw", ""))
                if text:
                    self._voice_pub.publish(String(data=text))
            elif topic == f"{self.prefix}/cmd/voice":
                text = data.get("text", data.get("_raw", ""))
                if text:
                    self._voice_pub.publish(String(data=text))
                    self._mqtt_pub("/status/info", {"msg": f"语音指令已转发: {text[:60]}"})
            elif topic == f"{self.prefix}/debug_cmd":
                cmd = data.get("cmd", data.get("_raw", ""))
                if cmd:
                    self._debug_pub.publish(String(data=cmd))
            elif topic in [f"{self.prefix}/cmd/map_update", "robot/all/cmd/broadcast"]:
                self._handle_map_update(data)

        except json.JSONDecodeError:
            self.get_logger().warn(f"Non-JSON on {topic}: {msg.payload[:80]}")
        except Exception as e:
            self.get_logger().error(f"Handle {topic}: {e}")

    # ──────────────────────────────────────────
    #  功能启停
    # ──────────────────────────────────────────
    def _handle_feature_cmd(self, action: str, target: str, params: dict):
        if target not in FEATURE_REGISTRY:
            self._mqtt_pub("/status/error", {"error": f"Unknown feature: {target}"})
            return
        feat = FEATURE_REGISTRY[target]
        if not feat.get("startable", False):
            self._mqtt_pub("/status/error", {"error": f"{target} 不可启停"})
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
            return
        else:
            return
        if text:
            self._voice_pub.publish(String(data=text))
            self._mqtt_pub("/status/info", {"msg": f"已请求 {action} {target}"})

    # ──────────────────────────────────────────
    #  进程看门狗
    # ──────────────────────────────────────────
    def _watchdog_check(self):
        pass

    def _poll_ros_nodes(self):
        slam_proc_running = nav_proc_running = False
        try:
            ps_out = subprocess.check_output(
                ["bash", "-c",
                 "ps -eo pid,cmd | grep -E 'largemodel_(slam|nav)\\.launch\\.py'"
                 " | grep -v 'grep' | grep -v 'bash -c' || true"],
                stderr=subprocess.DEVNULL, timeout=2,
            ).decode(errors="ignore")
            for line in ps_out.splitlines():
                if "largemodel_slam.launch.py" in line.lower():
                    slam_proc_running = True
                if "largemodel_nav.launch.py" in line.lower():
                    nav_proc_running = True
        except Exception:
            pass
        try:
            out = subprocess.check_output(
                ["ros2", "node", "list"], stderr=subprocess.DEVNULL, timeout=3,
            ).decode().strip().splitlines()
        except Exception:
            return
        raw_running = set()
        for line in out:
            node = line.strip()
            if not node:
                continue
            feat = ROS_NODE_TO_FEATURE.get(node)
            if feat:
                raw_running.add(feat)
                continue
            for pat, f in ROS_NODE_TO_FEATURE.items():
                if node == pat or node.endswith("/" + pat.lstrip("/")):
                    raw_running.add(f)
                    break
        running = set(raw_running)
        if nav_proc_running and slam_proc_running:
            running.discard("slam")
        elif nav_proc_running:
            running.discard("slam"); running.add("nav2")
        elif slam_proc_running:
            running.discard("nav2"); running.add("slam")
        else:
            if "nav2" in raw_running:
                running.discard("slam")
        for feat_id, status in list(self.process_status.items()):
            present = feat_id in running
            if present and status in ("idle", "crashed"):
                self.process_status[feat_id] = "running"
            elif not present and status == "running":
                if feat_id not in self.processes:
                    self.process_status[feat_id] = "idle"

    # ──────────────────────────────────────────
    #  ★ 核心：统一格式的状态上报 ★
    # ──────────────────────────────────────────
    def _report_status(self):
        if not self._mqtt_connected:
            return
        if not self._initial_poll_done:
            self._poll_ros_nodes()
            self._initial_poll_done = True

        # ── features: array ──
        features = []
        for key, feat in FEATURE_REGISTRY.items():
            status = self.process_status.get(key, "idle")
            entry = {
                "id": key,
                "name": feat["name"],
                "category": feat.get("category", ""),
                "behavior": "startable" if feat.get("startable") else "auto",
                "status": status,
            }
            if feat.get("startable"):
                entry["voice"] = {
                    "start": feat.get("voice_start"),
                    "stop": feat.get("voice_stop"),
                }
            features.append(entry)

        # ── 主车 pose（来自 TF）──
        try:
            trans = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            self._odom_x = trans.transform.translation.x
            self._odom_y = trans.transform.translation.y
            q = trans.transform.rotation
            self._odom_yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
        except Exception:
            pass

        # ── 构建统一的 robots 数组 ──
        # robot_0 = 主车（自己）
        main_robot = self._build_robot_entry(
            robot_id="robot_0",
            x=self._odom_x, y=self._odom_y, yaw=self._odom_yaw,
            vx=self._odom_vx, vz=self._odom_vz,
            bat_voltage=self._battery, bat_soc=None, bat_current=None,
            bat_charging=self._charging, bat_status=1 if self._charging else 2, bat_health=1,
            pose_source="tf",
        )
        # robot_1 = 次车1
        r1 = self._build_robot_entry(
            robot_id="robot_1",
            x=self._r1_state["x"], y=self._r1_state["y"], yaw=self._r1_state["yaw"],
            vx=self._r1_state["vx"], vz=self._r1_state["vz"],
            bat_voltage=self._r1_state["battery_voltage"],
            bat_soc=self._r1_state["battery_soc"],
            bat_current=self._r1_state["battery_current"],
            bat_charging=self._r1_state["battery_charging"],
            bat_status=self._r1_state["battery_status"],
            bat_health=self._r1_state["battery_health"],
            pose_source=self._r1_state["pose_source"],
        )
        # robot_2 = 次车2
        r2 = self._build_robot_entry(
            robot_id="robot_2",
            x=self._r2_state["x"], y=self._r2_state["y"], yaw=self._r2_state["yaw"],
            vx=self._r2_state["vx"], vz=self._r2_state["vz"],
            bat_voltage=self._r2_state["battery_voltage"],
            bat_soc=self._r2_state["battery_soc"],
            bat_current=self._r2_state["battery_current"],
            bat_charging=self._r2_state["battery_charging"],
            bat_status=self._r2_state["battery_status"],
            bat_health=self._r2_state["battery_health"],
            pose_source=self._r2_state["pose_source"],
        )

        robots = [main_robot, r1, r2]

        # ── llm ──
        llm_entry = {
            "feedback": self._last_llm_feedback,
            "timestamp": self._iso_now(),
        }

        # ── 完整 report ──
        report = {
            "version": "2.0",
            "timestamp": self._iso_now(),
            "features": features,
            "robots": robots,
            "llm": llm_entry,
        }

        self._mqtt_pub("/status/all", report)

    def _build_robot_entry(
        self, robot_id: str,
        x: float, y: float, yaw: float,
        vx: float, vz: float,
        bat_voltage, bat_soc, bat_current, bat_charging,
        bat_status, bat_health, pose_source,
    ) -> dict:
        has_pose = pose_source is not None and not (x == 0.0 and y == 0.0)
        has_bat  = bat_voltage is not None
        online   = has_bat  # 有电量数据就算在线

        entry: dict = {"id": robot_id, "online": online}

        # pose / velocity：始终输出，缺数据填 null
        if has_pose:
            entry["pose"] = {
                "x": round(x, 3),
                "y": round(y, 3),
                "yaw": round(yaw, 4),
                "source": pose_source,
            }
            entry["velocity"] = {
                "linear": round(vx, 3),
                "angular": round(vz, 3),
            }
        else:
            entry["pose"]     = None
            entry["velocity"] = None

        # battery：精简字段，按车型计算 SOC
        if has_bat:
            v = float(bat_voltage)
            # 主车 robot_0: 21V~25.55V 对应 0~100%
            # 次车 1/2: 10V~12.6V 对应 0~100%
            if robot_id == "robot_0":
                soc_calc = (v - 21.0) / (25.55 - 21.0) * 100.0
            else:
                soc_calc = (v - 10.0) / (12.6 - 10.0) * 100.0
            soc_calc = max(0.0, min(100.0, soc_calc))

            entry["battery"] = {
                "voltage": round(v, 2),
                "soc": round(bat_soc, 1) if bat_soc is not None else round(soc_calc, 1),
                "charging": bool(bat_charging) if bat_charging is not None else False,
            }
        else:
            entry["battery"] = None

        return entry

    # ──────────────────────────────────────────
    #  ROS 回调
    # ──────────────────────────────────────────
    def _map_cb(self, msg: OccupancyGrid):
        try:
            w, h = msg.info.width, msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((h, w))
            img = np.zeros((h, w), dtype=np.uint8)
            img[data == -1] = 128
            img[data == 0] = 254
            img[data >= 1] = 0
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
            img_msg.height = h; img_msg.width = w
            img_msg.encoding = "mono8"; img_msg.is_bigendian = 0
            img_msg.step = w; img_msg.data = img.tobytes()
            self._map_pgm_pub.publish(img_msg)
            # /map_pgm 原始数据
            if self._mqtt_connected:
                raw_b64 = base64.b64encode(bytes(msg.data)).decode("utf-8")
                self._mqtt_pub("/map_pgm", {
                    "header": {
                        "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                        "frame_id": msg.header.frame_id,
                    },
                    "info": {
                        "map_load_time": {"sec": msg.info.map_load_time.sec, "nanosec": msg.info.map_load_time.nanosec},
                        "resolution": msg.info.resolution,
                        "width": w, "height": h,
                        "origin": {
                            "position": {
                                "x": msg.info.origin.position.x,
                                "y": msg.info.origin.position.y,
                                "z": msg.info.origin.position.z,
                            },
                            "orientation": {
                                "x": msg.info.origin.orientation.x,
                                "y": msg.info.origin.orientation.y,
                                "z": msg.info.origin.orientation.z,
                                "w": msg.info.origin.orientation.w,
                            },
                        },
                    },
                    "image": raw_b64,
                }, retain=True)
        except Exception as e:
            self.get_logger().error(f"处理地图数据时出错: {e}")

    def _odom_cb(self, msg: Odometry):
        self._odom_vx = msg.twist.twist.linear.x
        self._odom_vz = msg.twist.twist.angular.z

    def _odom_combined_cb(self, msg: Odometry):
        self._odom_vx = msg.twist.twist.linear.x
        self._odom_vz = msg.twist.twist.angular.z

    def _voltage_cb(self, msg: Float32):
        self._battery = msg.data

    def _charge_cb(self, msg: Bool):
        self._charging = msg.data

    def _feedback_cb(self, msg: String):
        self._last_llm_feedback = msg.data
        if self._mqtt_connected:
            self._mqtt_pub("/voice/response", {"text": msg.data})

    # ── 次车回调 ──
    def _r1_soldier_pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        self._r1_state["x"] = p.position.x
        self._r1_state["y"] = p.position.y
        self._r1_state["yaw"] = self._quat_to_yaw(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        self._r1_state["pose_source"] = "soldier_pose"
        self._r1_state["pose_ts"] = time.time()

    def _r2_soldier_pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        self._r2_state["x"] = p.position.x
        self._r2_state["y"] = p.position.y
        self._r2_state["yaw"] = self._quat_to_yaw(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        self._r2_state["pose_source"] = "soldier_pose"
        self._r2_state["pose_ts"] = time.time()

    def _r1_odom_cb(self, msg: Odometry):
        self._r1_state["vx"] = msg.twist.twist.linear.x
        self._r1_state["vz"] = msg.twist.twist.angular.z

    def _r2_odom_cb(self, msg: Odometry):
        self._r2_state["vx"] = msg.twist.twist.linear.x
        self._r2_state["vz"] = msg.twist.twist.angular.z

    def _r1_battery_state_cb(self, msg: BatteryState):
        soc_pct = (msg.percentage * 100.0) if msg.percentage is not None else None
        self._r1_state["battery_voltage"] = msg.voltage
        self._r1_state["battery_current"] = msg.current
        self._r1_state["battery_soc"] = soc_pct
        self._r1_state["battery_status"] = int(msg.power_supply_status)
        self._r1_state["battery_health"] = int(msg.power_supply_health)
        self._r1_state["battery_charging"] = bool(msg.power_supply_status == msg.POWER_SUPPLY_STATUS_CHARGING)
        self._r1_state["battery_ts"] = time.time()

    def _r2_battery_state_cb(self, msg: BatteryState):
        soc_pct = (msg.percentage * 100.0) if msg.percentage is not None else None
        self._r2_state["battery_voltage"] = msg.voltage
        self._r2_state["battery_current"] = msg.current
        self._r2_state["battery_soc"] = soc_pct
        self._r2_state["battery_status"] = int(msg.power_supply_status)
        self._r2_state["battery_health"] = int(msg.power_supply_health)
        self._r2_state["battery_charging"] = bool(msg.power_supply_status == msg.POWER_SUPPLY_STATUS_CHARGING)
        self._r2_state["battery_ts"] = time.time()

    def _r1_map_cb(self, msg: OccupancyGrid):
        self._r1_map_msg = msg

    def _r1_scan_cb(self, msg: LaserScan):
        if not msg.ranges:
            self._r1_front_min_dist = float("inf")
            self._r1_front_obstacle = False
            return
        n = len(msg.ranges)
        center = n // 2
        span = max(1, int(0.17 * n))
        front_min = float("inf")
        for i in range(max(0, center - span), min(n, center + span)):
            r = msg.ranges[i]
            if math.isfinite(r) and r < front_min:
                front_min = r
        self._r1_front_min_dist = front_min
        self._r1_front_obstacle = front_min < self.subcar1_stop_d

    # ──────────────────────────────────────────
    #  次车1 主车代规划：A* + Pure Pursuit
    # ──────────────────────────────────────────
    def _plan_for_subcar_1(self, goal_x: float, goal_y: float):
        if self._r1_map_msg is None:
            self._mqtt_pub("/status/error", {"error": "subcar1 map not available"})
            return
        sx, sy = self._r1_state["x"], self._r1_state["y"]
        if sx == 0.0 and sy == 0.0 and self._r1_state["pose_source"] is None:
            self._mqtt_pub("/status/error", {"error": "subcar1 pose unknown"})
            return
        path = self._astar(self._r1_map_msg, (sx, sy), (goal_x, goal_y))
        if not path:
            self._mqtt_pub("/status/error", {"error": "subcar1 A* 找不到路径"})
            return
        simplified = self._simplify_path(path, min_dist=0.2)
        self._r1_path = simplified
        self._r1_wp_idx = 0
        self._mqtt_pub("/status/info", {"msg": f"subcar1 路径规划成功 ({len(simplified)} waypoints)"})
        if self._r1_pp_timer is not None:
            self.destroy_timer(self._r1_pp_timer)
        self._r1_pp_timer = self.create_timer(0.2, self._r1_pp_tick)

    @staticmethod
    def _simplify_path(path, min_dist=0.2):
        if len(path) < 3:
            return path
        out = [path[0]]
        for p in path[1:-1]:
            if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= min_dist:
                out.append(p)
        out.append(path[-1])
        return out

    def _astar(self, grid_msg: OccupancyGrid, start_xy, goal_xy):
        import heapq
        info = grid_msg.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y

        def to_gx(x, y): return int((x - ox) / res), int((y - oy) / res)
        def to_xy(gx, gy): return (gx * res + ox, gy * res + oy)
        sx, sy = to_gx(*start_xy); gx, gy = to_gx(*goal_xy)
        if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
            return None
        data = np.array(grid_msg.data, dtype=np.int8).reshape((h, w))
        OBSTACLE = 50
        if data[sy, sx] >= OBSTACLE or data[gy, gx] >= OBSTACLE:
            return None

        inflate_px = int(math.ceil((self.subcar1_radius + self.subcar1_margin) / res))
        if inflate_px > 0:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * inflate_px + 1, 2 * inflate_px + 1))
            obst_mask = (data >= OBSTACLE).astype(np.uint8) * 255
            dilated = cv2.dilate(obst_mask, kernel)
            data = np.where(dilated > 0, np.int8(OBSTACLE), data).astype(np.int8)

        neigh = [(-1,-1,1.414),(0,-1,1.0),(1,-1,1.414),
                 (-1,0,1.0),(1,0,1.0),
                 (-1,1,1.414),(0,1,1.0),(1,1,1.414)]

        def h(ix, iy):
            dx = abs(ix - gx); dy = abs(iy - gy)
            return (max(dx, dy) - min(dx, dy)) * 1.0 + min(dx, dy) * 1.414

        g_score = {(sx, sy): 0.0}; came = {}; closed = set()
        open_heap = [(h(sx, sy), 0.0, (sx, sy))]
        counter = 0
        GOAL_TOL = 2

        while open_heap:
            _, _, cur = heapq.heappop(open_heap)
            if cur in closed:
                continue
            closed.add(cur)
            cx, cy = cur
            if abs(cx - gx) <= GOAL_TOL and abs(cy - gy) <= GOAL_TOL:
                rev = [cur]
                while cur in came:
                    cur = came[cur]; rev.append(cur)
                rev.reverse()
                return [to_xy(*p) for p in rev]
            for dx, dy, cost in neigh:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < w and 0 <= ny < h): continue
                if (nx, ny) in closed: continue
                if data[ny, nx] >= OBSTACLE: continue
                if dx != 0 and dy != 0:
                    if data[cy, cx + dx] >= OBSTACLE or data[cy + dy, cx] >= OBSTACLE:
                        continue
                ng = g_score[cur] + cost
                if ng < g_score.get((nx, ny), float("inf")):
                    g_score[(nx, ny)] = ng
                    came[(nx, ny)] = cur
                    counter += 1
                    heapq.heappush(open_heap, (ng + h(nx, ny), counter, (nx, ny)))
        return None

    def _r1_pp_tick(self):
        if not self._r1_path or self._r1_wp_idx >= len(self._r1_path):
            self._r1_cmd_pub.publish(Twist())
            self._stop_pp()
            self._mqtt_pub("/status/info", {"msg": "subcar1 已到达目标点"})
            return
        cx, cy = self._r1_state["x"], self._r1_state["y"]
        yaw = self._r1_state["yaw"]
        if cx == 0.0 and cy == 0.0 and self._r1_state["pose_source"] is None:
            return

        while self._r1_wp_idx < len(self._r1_path):
            wx, wy = self._r1_path[self._r1_wp_idx]
            if math.hypot(wx - cx, wy - cy) < self.subcar1_arrive:
                self._r1_wp_idx += 1
            else:
                break

        if self._r1_wp_idx >= len(self._r1_path):
            self._r1_cmd_pub.publish(Twist())
            self._stop_pp()
            self._mqtt_pub("/status/info", {"msg": "subcar1 已到达目标点"})
            return

        L = self.subcar1_look
        target = self._r1_path[-1]
        for i in range(self._r1_wp_idx, len(self._r1_path)):
            wx, wy = self._r1_path[i]
            if math.hypot(wx - cx, wy - cy) >= L:
                target = (wx, wy); break

        dx = target[0] - cx; dy = target[1] - cy
        alpha = math.atan2(math.sin(math.atan2(dy, dx) - yaw),
                           math.cos(math.atan2(dy, dx) - yaw))

        v = self.subcar1_max_v
        if abs(alpha) > math.radians(45):
            v = self.subcar1_min_v
        elif abs(alpha) > math.radians(20):
            ratio = (math.radians(45) - abs(alpha)) / math.radians(25)
            v = self.subcar1_min_v + (self.subcar1_max_v - self.subcar1_min_v) * ratio

        front_d = getattr(self, "_r1_front_min_dist", float("inf"))
        if front_d < self.subcar1_stop_d:
            v = 0.0
        elif front_d < self.subcar1_slow_d:
            ratio = (front_d - self.subcar1_stop_d) / (self.subcar1_slow_d - self.subcar1_stop_d)
            v = min(v, self.subcar1_min_v + (self.subcar1_max_v - self.subcar1_min_v) * ratio)

        w = 2.0 * v * math.sin(alpha) / max(L, 0.05)
        w = max(-self.subcar1_max_w, min(self.subcar1_max_w, w))

        t = Twist()
        t.linear.x = v; t.angular.z = w
        self._r1_cmd_pub.publish(t)

    def _stop_pp(self):
        if self._r1_pp_timer is not None:
            self.destroy_timer(self._r1_pp_timer)
            self._r1_pp_timer = None
        self._r1_path = []; self._r1_wp_idx = 0

    # ──────────────────────────────────────────
    #  地图更新
    # ──────────────────────────────────────────
    def _handle_map_update(self, payload: dict):
        self.get_logger().info("收到远端地图更新数据，开始解析与重载...")
        try:
            if "image" in payload and "info" in payload:
                info = payload["info"]
                width = info["width"]; height = info["height"]; resolution = info["resolution"]
                origin_pos = info["origin"].get("position", {"x": 0.0, "y": 0.0, "z": 0.0})
                origin_ori = info["origin"].get("orientation", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})
                origin_x = origin_pos["x"]; origin_y = origin_pos["y"]
                yaw = self._quat_to_yaw(origin_ori["x"], origin_ori["y"], origin_ori["z"], origin_ori["w"])
                raw_bytes = base64.b64decode(payload["image"])
                grid = np.frombuffer(raw_bytes, dtype=np.int8).reshape((height, width))
            elif "map" in payload and "data" in payload["map"]:
                map_info = payload["map"]
                width = map_info["width"]; height = map_info["height"]
                resolution = map_info.get("resolution", 0.05)
                origin_x = map_info.get("origin_x", 0.0); origin_y = map_info.get("origin_y", 0.0)
                yaw = 0.0
                grid = np.array(map_info["data"], dtype=np.int8).reshape((height, width))
            else:
                self.get_logger().error("数据格式错误：无法识别地图 Payload")
                return

            img = np.zeros((height, width), dtype=np.uint8)
            img[grid == -1] = 128
            img[grid == 0] = 254
            img[grid >= 1] = 0
            img = cv2.flip(img, 0)

            map_dir = "/home/nvidia/wheeltec_ros2/src/wheeltec_robot_rtab"
            os.makedirs(map_dir, exist_ok=True)
            pgm_path = os.path.join(map_dir, "my_map.pgm")
            yaml_path = os.path.join(map_dir, "my_map.yaml")
            cv2.imwrite(pgm_path, img)
            yaml_content = f"""image: my_map.pgm
mode: trinary
resolution: {resolution}
origin: [{origin_x}, {origin_y}, {yaw}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.19
"""
            with open(yaml_path, "w") as f:
                f.write(yaml_content)
            self.get_logger().info(f"地图已覆盖: {pgm_path} (Yaw: {yaw})")
            srv_cmd = [
                "ros2", "service", "call",
                "/map_server/load_map",
                "nav2_msgs/srv/LoadMap",
                f"{{map_url: {yaml_path}}}"
            ]
            subprocess.Popen(srv_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self._mqtt_pub("/status/info", {"msg": "地图重载成功"})
        except Exception as e:
            self.get_logger().error(f"地图更新失败: {e}")

    def shutdown(self):
        self.get_logger().info("Shutting down SystemManager...")
        self._stop_pp()
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
