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
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt
import json
import subprocess
import os
import time
import signal
import threading

# ════════════════════════════════════════════
#  功能注册表 — 所有可管理的功能单元
# ════════════════════════════════════════════
FEATURE_REGISTRY = {
    # ── 基础功能 ──
    "base": {
        "name": "底盘驱动",
        "start": ["ros2", "launch", "turn_on_wheeltec_robot", "base_serial.launch.py"],
        "description": "STM32 串口通信与底盘驱动",
        "category": "base",
        "auto_start": True,
    },
    # ── 摄像头 ──
    "camera": {
        "name": "摄像头",
        "start": ["ros2", "launch", "turn_on_wheeltec_robot", "wheeltec_camera.launch.py"],
        "description": "RGB 相机驱动",
        "category": "sensor",
    },
    # ── 麦克风阵列 ──
    "mic": {
        "name": "麦克风阵列",
        "start": ["ros2", "run", "wheeltec_mic_aiui", "wheeltec_mic"],
        "description": "语音采集",
        "category": "sensor",
    },
    # ── SLAM 建图 ──
    "slam": {
        "name": "SLAM 建图",
        "start": ["ros2", "launch", "largemodel", "largemodel_slam.launch.py"],
        "description": "同步定位与地图构建",
        "category": "perception",
    },
    # ── 自主导航 ──
    "nav2": {
        "name": "自主导航",
        "start": ["ros2", "launch", "wheeltec_robot_nav2", "wheeltec_nav2.launch.py"],
        "description": "Nav2 路径规划与导航",
        "category": "navigation",
        "depends": ["slam"],
    },
    # ── KCF 视觉跟随 ──
    "kcf": {
        "name": "KCF 视觉跟随",
        "start": ["ros2", "launch", "wheeltec_robot_kcf", "wheeltec_robot_kcf.launch.py"],
        "description": "KCF 目标跟踪与跟随",
        "category": "follow",
    },
    # ── 激光跟随 ──
    "laser_follower": {
        "name": "激光跟随",
        "start": ["ros2", "run", "simple_follower_ros2", "lasertracker"],
        "description": "激光雷达人物跟随",
        "category": "follow",
    },
    # ── 路径跟随 ──
    "path_follow": {
        "name": "路径跟随",
        "start": ["ros2", "launch", "wheeltec_path_follow", "...launch.py"],
        "description": "沿预设路径行驶",
        "category": "navigation",
    },
    # ── 大模型 ──
    "largemodel": {
        "name": "大模型 AI",
        "start": ["ros2", "launch", "largemodel", "largemodel_control.launch.py"],
        "description": "语音交互+大模型任务执行",
        "category": "ai",
    },
    # ── 自动回充 ──
    "align": {
        "name": "回充对接",
        "start": [],  # 由 action_service 触发
        "description": "视觉引导充电桩自动对接",
        "category": "utility",
    },
    # ── WebRTC 推流 ──
    "webrtc": {
        "name": "视频推流",
        "start": ["ros2", "run", "mqtt_bridge_ros2", "webrtc_pusher"],
        "description": "WebRTC 摄像头视频流",
        "category": "sensor",
    },
    # ── Web Console ──
    "web_console": {
        "name": "Web 控制台",
        "start": ["ros2", "launch", "mqtt_bridge_ros2", "web_console.launch.py"],
        "description": "rosbridge + Web 可视化",
        "category": "utility",
    },
}


ROS_NODE_TO_FEATURE = {
    "/base_serial":           "base",
    "/camera/camera":         "camera",
    "/wheeltec_mic":          "mic",
    "/wheeltec_mic_aiui":     "mic",
    "/aiui_node":             "mic",
    "/rtabmap":               "slam",
    "/slam_toolbox":          "slam",
    "/lasertracker":          "laser_follower",
    "/kcf_tracker_node":      "kcf",
    "/model_service":         "largemodel",
    "/action_service":        "largemodel",
    "/web_video_server":      "web_console",
    "/rosbridge_websocket":   "web_console",
    "/webrtc_camera_node":    "webrtc",
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
        self.declare_parameter("status_interval", 3.0)

        self.broker = self.get_parameter("mqtt_broker").value
        self.port = self.get_parameter("mqtt_port").value
        self.client_id = self.get_parameter("mqtt_client_id").value
        self.prefix = self.get_parameter("topic_prefix").value
        self.status_interval = self.get_parameter("status_interval").value

        # ═══ 进程管理 ═══
        self.processes: dict[str, subprocess.Popen] = {}
        self.process_status: dict[str, str] = {}  # idle | running | crashed
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

        # ═══ ROS2 订阅者 ═══
        self._odom_sub = self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self._volt_sub = self.create_subscription(
            Float32, "/PowerVoltage", self._voltage_cb, 10
        )
        self._charge_sub = self.create_subscription(
            Bool, "/robot_charging_flag", self._charge_cb, 10
        )
        self._feedback_sub = self.create_subscription(
            String, "feedback_words", self._feedback_cb, 10
        )

        # ═══ 状态缓存 ═══
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_vx = 0.0
        self._odom_vz = 0.0
        self._battery = 0.0
        self._charging = False
        self._last_llm_feedback = ""

        # ═══ 状态定时器 ═══
        self._status_timer = self.create_timer(self.status_interval, self._report_status)
        self._proc_watchdog = self.create_timer(5.0, self._watchdog_check)
        # ROS 节点轮询（5s）把实际在跑的节点映射到 features
        self._ros_poll_timer = self.create_timer(5.0, self._poll_ros_nodes)

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
            # 订阅主题
            topics = [
                (f"{self.prefix}/cmd_vel", 0),
                (f"{self.prefix}/goal_pose", 0),
                (f"{self.prefix}/sys_cmd", 0),
                (f"{self.prefix}/initialpose", 0),
                (f"{self.prefix}/voice_input", 0),
                (f"{self.prefix}/debug_cmd", 0),
                (f"{self.prefix}/webrtc_answer", 0),
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
                self.get_logger().info(f"Set initial pose: ({data.get(x)}, {data.get(y)})")

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

            # ── 调试指令 ──
            elif topic == f"{self.prefix}/debug_cmd":
                cmd = data.get("cmd", data.get("_raw", ""))
                if cmd:
                    self._debug_pub.publish(String(data=cmd))

        except json.JSONDecodeError:
            self.get_logger().warn(f"Non-JSON on {topic}: {msg.payload[:80]}")
        except Exception as e:
            self.get_logger().error(f"Handle {topic}: {e}")

    # ────────────────────────────────────────
    #  功能启停核心逻辑
    # ────────────────────────────────────────
    def _handle_feature_cmd(self, action: str, target: str, params: dict):
        if target not in FEATURE_REGISTRY:
            self._mqtt_pub(f"/status/error", {"error": f"Unknown feature: {target}"})
            return

        feat = FEATURE_REGISTRY[target]

        if action == "start":
            if self.process_status.get(target) == "running":
                self.get_logger().warning(f"{target} already running")
                return
            if not feat.get("start"):
                self.get_logger().warning(f"{target} has no start command")
                return

            cmd = feat["start"]
            # 替换参数模板
            resolved = [c.format(**params) for c in cmd]

            self.get_logger().info("Starting " + target + ": " + " ".join(resolved))
            try:
                log_dir = os.path.expanduser("~/.ros/mqtt_logs")
                os.makedirs(log_dir, exist_ok=True)
                logfile = open(os.path.join(log_dir, f"{target}.log"), "w")
                proc = subprocess.Popen(
                    resolved, stdout=logfile, stderr=subprocess.STDOUT,
                    preexec_fn=lambda: os.setsid() if hasattr(os, "setsid") else None,
                )
                self.processes[target] = proc
                self.process_status[target] = "running"
                self.get_logger().info(f"{target} started (PID {proc.pid})")
            except Exception as e:
                self.get_logger().error(f"Start {target} failed: {e}")
                self.process_status[target] = "crashed"

        elif action == "stop":
            if target not in self.processes:
                self.get_logger().warning(f"{target} not running")
                return
            proc = self.processes[target]
            try:
                if proc.poll() is None:
                    # Kill the entire process group
                    if hasattr(os, "killpg") and hasattr(proc, "pid"):
                        pgid = os.getpgid(proc.pid)
                        os.killpg(pgid, signal.SIGTERM)
                    else:
                        proc.terminate()
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=3)
                del self.processes[target]
                self.process_status[target] = "idle"
                self.get_logger().info(f"{target} stopped")
            except Exception as e:
                self.get_logger().error(f"Stop {target} failed: {e}")

        elif action == "restart":
            self._handle_feature_cmd("stop", target, params)
            time.sleep(1)
            self._handle_feature_cmd("start", target, params)

        else:
            self.get_logger().warning(f"Unknown action: {action}")

    # ────────────────────────────────────────
    #  进程看门狗
    # ────────────────────────────────────────
    def _watchdog_check(self):
        for target, proc in list(self.processes.items()):
            if proc.poll() is not None:
                self.process_status[target] = "crashed"
                self.get_logger().warn(f"{target} has crashed (exit code {proc.returncode})")
                del self.processes[target]

    def _poll_ros_nodes(self):
        """把 ROS 图里已经在跑的节点映射到 features，标为 running。
        只覆盖 idle 状态；不动 system_manager 自己 Popen 起的进程。"""
        try:
            out = subprocess.check_output(
                ["ros2", "node", "list"],
                stderr=subprocess.DEVNULL, timeout=3,
            ).decode().strip().splitlines()
        except Exception:
            return
        running = set()
        for line in out:
            node = line.strip()
            if not node: continue
            feat = ROS_NODE_TO_FEATURE.get(node)
            if feat:
                running.add(feat); continue
            for pat, f in ROS_NODE_TO_FEATURE.items():
                if node == pat or node.endswith("/" + pat.lstrip("/")):
                    running.add(f); break
        changed = False
        for feat_id, status in list(self.process_status.items()):
            if feat_id in running and status == "idle" and feat_id not in self.processes:
                self.process_status[feat_id] = "running"
                changed = True
        if changed:
            self.get_logger().debug(f"ROS-poll running features: {running}")

    # ────────────────────────────────────────
    #  状态收集 & 上报
    # ────────────────────────────────────────
    def _report_status(self):
        if not self._mqtt_connected:
            return

        # 1. 所有功能状态
        features = {}
        for key, feat in FEATURE_REGISTRY.items():
            status = self.process_status.get(key, "idle")
            features[key] = {
                "status": status,
                "name": feat["name"],
                "category": feat.get("category", ""),
            }

        # 2. 机器人状态
        robot = {
            "position": {"x": round(self._odom_x, 3), "y": round(self._odom_y, 3)},
            "velocity": {"linear": round(self._odom_vx, 3), "angular": round(self._odom_vz, 3)},
            "battery": round(self._battery, 2),
            "charging": self._charging,
            "online": True,
        }

        # 3. 大模型反馈回传
        llm_feedback = self._last_llm_feedback

        report = {
            "features": features,
            "robot": robot,
            "llm_feedback": llm_feedback,
            "timestamp": time.time(),
        }

        self._mqtt_pub("/status/all", report)

    # ────────────────────────────────────────
    #  ROS2 订阅回调
    # ────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_vx = msg.twist.twist.linear.x
        self._odom_vz = msg.twist.twist.angular.z

    def _voltage_cb(self, msg: Float32):
        self._battery = msg.data

    def _charge_cb(self, msg: Bool):
        self._charging = msg.data

    def _feedback_cb(self, msg: String):
        """大模型语音反馈 → 转发到 MQTT"""
        self._last_llm_feedback = msg.data
        if self._mqtt_connected:
            self._mqtt_pub("/voice/response", {"text": msg.data})

    # ────────────────────────────────────────
    #  MQTT 发布 helper
    # ────────────────────────────────────────
    def _mqtt_pub(self, topic_suffix: str, data: dict, qos=0):
        full_topic = f"{self.prefix}{topic_suffix}"
        try:
            self.mqtt.publish(full_topic, json.dumps(data), qos=qos)
        except Exception:
            pass

    # ────────────────────────────────────────
    #  清理
    # ────────────────────────────────────────
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
