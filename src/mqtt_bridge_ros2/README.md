

---

# 📡 mqtt_bridge_ros2

**机器人对外通信与多机协同中枢包**。本包提供三个 ROS 2 节点 + 三个 launch 文件，将 ROS 2 机器人系统与远端前端（MQTT / WebSocket / WebRTC）双向打通，并支持**主车代规划控制算力受限的次车**。

> 版本: 0.0.2 | 维护: nvidia@wheeltec.com | License: MIT

---

## 🎯 包功能概览

| 能力 | 说明 |
|---|---|
| MQTT ↔ ROS 2 双向桥接 | 远端前端通过 MQTT 控制机器人并接收状态 |
| RViz 级别远程操控 | cmd_vel / goal_pose / initialpose 透传 |
| 5Hz 状态聚合上报 | 电池、位姿、速度、功能状态统一推送 |
| 远端地图热更新 | 支持 base64 OccupancyGrid 实时重载 Nav2 地图 |
| 功能生命周期管理 | 注册式功能启停，统一派单到 LLM 语音管线 |
| 语音桥接 | MQTT 文本 → ROS `voice_words` 话题 → LLM |
| **主车代规划控制次车** | A\* + Pure Pursuit + 激光避障兜底 |
| 多机 MQTT 透传 | 次车 2 由主车做纯 MQTT ↔ ROS 桥接 |
| WebRTC 视频推流 | aiortc + Flask 信令 |
| WebSocket ↔ MQTT 透明代理 | 浏览器原生 MQTT 通道 |

---

## 🧩 节点清单

| 可执行 | 源文件 | 角色 |
|---|---|---|
| `system_manager` | `mqtt_bridge_ros2/system_manager.py` | **主控节点** — 所有桥接 + 规划 + 状态聚合 |
| `webrtc_pusher` | `mqtt_bridge_ros2/webrtc_pusher.py` | WebRTC 摄像头视频推流 |
| `ws_mqtt_proxy` | `mqtt_bridge_ros2/ws_mqtt_proxy.py` | WebSocket ↔ MQTT 透明代理（备选） |

### 节点启动方式

| 启动方式 | 包含节点 |
|---|---|
| `ros2 launch mqtt_bridge_ros2 system_launch.py` ⭐ 主入口 | `system_manager` + `webrtc_pusher` + `rosbridge_server` + `web_video_server` |
| `ros2 launch mqtt_bridge_ros2 mqtt_bridge.launch.py` | （**已过时**，引用不存在的 `mqtt_bridge_node`） |
| `ros2 launch mqtt_bridge_ros2 web_console.launch.py` | 仅 rosbridge + web_video_server |
| 手动启动 `ros2 run mqtt_bridge_ros2 ws_mqtt_proxy` | 独立 WS ↔ MQTT 桥 |

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                          远端 (192.168.1.67)                        │
│                                                                     │
│   ┌──────────────┐    ┌─────────────────────────────────┐           │
│   │  MQTT Broker │◄──►│  前端 / 控制台 / LLM 后端       │           │
│   │ 1883         │    │  (rosbridge 9090, Web 控制台)    │           │
│   └──────▲───────┘    └─────────────┬───────────────────┘           │
│          │                          │                               │
│          │ MQTT                     │ WebSocket / WebRTC            │
└──────────┼──────────────────────────┼───────────────────────────────┘
           │                          │
┌──────────┼──────────────────────────┼───────────────────────────────┐
│   Jetson │  Orin Nano 8GB (主车)    │                               │
│          │                          │                               │
│   ┌──────▼─────────────────────────────────────────────────────┐    │
│   │  system_launch.py                                          │    │
│   │  ┌─────────────────────────────────────────────────────┐  │    │
│   │  │  system_manager (核心节点)                          │  │    │
│   │  │  ① MQTT ↔ ROS2 双向桥接                            │  │    │
│   │  │  ② 状态聚合 5Hz 上行                               │  │    │
│   │  │  ③ 功能启停派单(语音管线)                          │  │    │
│   │  │  ④ 主车 RViz 远程操控                              │  │    │
│   │  │  ⑤ 地图热更新                                      │  │    │
│   │  │  ⑥ 次车1 主车代规划                                │  │    │
│   │  │     (A* + Pure Pursuit + 激光避障)                │  │    │
│   │  │  ⑦ 次车1/2 状态汇总                                │  │    │
│   │  │  ⑧ 次车2 MQTT 透传                                 │  │    │
│   │  └─────────────────────────────────────────────────────┘  │    │
│   │  ┌────────────────────┐  ┌─────────────────────────────┐  │    │
│   │  │  webrtc_pusher     │  │  rosbridge + web_video      │  │    │
│   │  │  (WebRTC 视频推流) │  │  (外部包, Web 控制台)       │  │    │
│   │  └────────────────────┘  └─────────────────────────────┘  │    │
│   └────────────────────────────────────────────────────────────┘    │
│                                                                     │
│   ROS 2 域: /map, /cmd_vel, /voice_words, ...                       │
└──────────────┬──────────────────────────────┬──────────────────────┘
               │ /robot_1/cmd_vel             │ /robot_2/{cmd_vel,goal_pose,initialpose}
               │ (主车代规划发出)              │ (主车纯透传)
               ▼                              ▼
┌──────────────────────────┐    ┌──────────────────────────┐
│   次车 1 (算力弱)        │    │   次车 2 (算力足)        │
│  • amcl (定位)          │    │  • Nav2 完整栈            │
│  • 底盘驱动              │    │  • amcl + SLAM           │
│  • 激光雷达              │    │                          │
│  上发:                   │    │  上发:                    │
│    /robot_1/soldier_pose │    │    /robot_2/amcl_pose    │
│    /robot_1/odom         │    │    /robot_2/odom         │
│    /robot_1/scan         │    │    /robot_2/PowerVoltage │
│    /robot_1/battery_state│    │                          │
└──────────────────────────┘    └──────────────────────────┘
```

### 设计原则

1. **单一入口节点** — `system_manager` 承担 80% 的桥接/规划/状态工作，避免多节点相互通信
2. **MQTT 主题分层** — 主车 `robot_0/*`，次车直接 `robot_1/*`、`robot_2/*`
3. **次车能力差异化** — 算力弱的次车 1 由主车代规划；算力足的次车 2 自行 Nav2
4. **避障兜底** — 纯 A\* 不够，激光雷达反应式避障补位
5. **MQTT 主题与 ROS 话题 1:1 映射** — 方便前端开发者直接对应

---

## 🔄 次车 1 主车代规划数据流

```
┌─────────────────────────────────────────────────────────────────┐
│  主车 system_manager (次车 1 规划子系统)                          │
│                                                                 │
│  ROS 2 订阅:                                                    │
│    /map                    ──► self._r1_map_msg                 │
│                                   (cv2.dilate 障碍膨胀)          │
│    /robot_1/soldier_pose   ──► self._r1_state[x/y/yaw]          │
│    /robot_1/scan           ──► self._r1_front_min_dist          │
│                                                                 │
│  MQTT 触发:                                                     │
│    robot_1/goal_pose {x,y}                                          │
│            │                                                    │
│            ▼                                                    │
│      ┌──────────────┐     ┌──────────────┐                     │
│      │ _plan_for_1  │────►│ _astar       │                     │
│      │  入口        │     │ 8-邻居       │                     │
│      └──────────────┘     │ octile 启发  │                     │
│                            │ 二叉堆       │                     │
│                            └──────┬───────┘                     │
│                                   │ waypoints                    │
│                                   ▼                              │
│                            ┌──────────────┐                     │
│                            │ _simplify    │                     │
│                            │ 路径简化     │                     │
│                            └──────┬───────┘                     │
│                                   │ ~10 个 waypoint             │
│                                   ▼                              │
│      ┌────────────────────────────────────────────┐              │
│      │  create_timer(0.2, _r1_pp_tick)  5Hz      │              │
│      │  Pure Pursuit + 渐进式避障 + 角速度限幅   │              │
│      └──────────────────┬─────────────────────────┘              │
│                         │ Twist(v, w)                            │
│                         ▼                                        │
│                   /robot_1/cmd_vel ─────► 次车 1 底盘            │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📡 MQTT 协议契约

### 主车（robot_0）下行主题（前端 → 主车）

| 主题 | Payload | 行为 |
|---|---|---|
| `robot_0/cmd_vel` | `{linear_x, angular_z}` | → `/cmd_vel` |
| `robot_0/goal_pose` | `{x, y, oz, ow}` | → `/goal_pose` (主车 Nav2) |
| `robot_0/initialpose` | `{x, y, oz, ow}` | → `/initialpose` (主车 AMCL) |
| `robot_0/voice_input` | `{text}` | → `/voice_words` |
| `robot_0/cmd/voice` | `{text}` | → `/voice_words`（前端按钮 = 本地语音） |
| `robot_0/sys_cmd` | `{action, target, params}` | 派单到 LLM 语音管线 |
| `robot_0/debug_cmd` | `{cmd}` | → `/debug_cmd` |
| `robot_0/cmd/map_update` | `{image, info}` 或 `{map, data}` | 远端热更新地图 |
| `robot_0/webrtc_answer` | WebRTC SDP answer | （预留接口） |

### 主车（robot_0）上行主题（主车 → 前端）

| 主题 | Payload | 触发 |
|---|---|---|
| `robot_0/status/all` | 完整 JSON（features + robot + robot_1 + robot_2） | 5Hz 定时 |
| `robot_0/status/info` | `{msg}` | 事件消息 |
| `robot_0/status/error` | `{error}` | 错误消息 |
| `robot_0/voice/response` | `{text}` | LLM 回复 |
| `robot_0/map_data` | PNG base64 | 地图变化 |
| `robot_0/map_pgm` | 原始 int8[] base64 | 地图变化 |

### 次车 1（robot_1）主题 — 主车代规划

| 主题 | 方向 | Payload | 行为 |
|---|---|---|---|
| `robot_1/cmd_vel` | 下行 | `{linear_x, angular_z}` | 直接转发 → `/robot_1/cmd_vel` |
| `robot_1/goal_pose` | 下行 | `{x, y, oz, ow}` | **触发主车 A\* + Pure Pursuit → `/robot_1/cmd_vel`** |
| `robot_1/initialpose` | 下行 | `{x, y, oz, ow}` | → `/robot_1/initialpose` |
| `robot_1/cancel_goal` | 下行 | `{cancel: true}` | 主车停 + 销毁控制器定时器 |

### 次车 2（robot_2）主题 — 纯 MQTT 透传

| 主题 | 方向 | Payload | 行为 |
|---|---|---|---|
| `robot_2/cmd_vel` | 下行 | `{linear_x, angular_z}` | → `/robot_2/cmd_vel` |
| `robot_2/goal_pose` | 下行 | `{x, y, oz, ow}` | → `/robot_2/goal_pose`（次车 2 自己的 Nav2 算） |
| `robot_2/initialpose` | 下行 | `{x, y, oz, ow}` | → `/robot_2/initialpose` |

### 广播主题

| 主题 | 方向 | 用途 |
|---|---|---|
| `robot/all/cmd/broadcast` | 下行 | 全机器人地图广播（多机同步） |

---

## 🤖 ROS 2 话题清单（system_manager 节点内）

### 发布者

| 话题 | 类型 | 用途 |
|---|---|---|
| `/cmd_vel` | Twist | 主车运动 |
| `/goal_pose` | PoseStamped | 主车导航目标 |
| `/initialpose` | PoseWithCovarianceStamped | 主车 AMCL 初始位姿 |
| `/voice_words` | String | 语音文本 → LLM |
| `/debug_cmd` | String | 调试命令 |
| `/map_pgm` | Image | 地图副本 |
| `/robot_1/cmd_vel` | Twist | **次车 1 运动（主车规划发出）** |
| `/robot_1/goal_pose` | PoseStamped | 次车 1 初始目标（备用） |
| `/robot_1/initialpose` | PoseWithCovarianceStamped | 次车 1 初始位姿 |
| `/robot_2/cmd_vel` | Twist | 次车 2 运动（透传） |
| `/robot_2/goal_pose` | PoseStamped | 次车 2 导航目标（透传） |
| `/robot_2/initialpose` | PoseWithCovarianceStamped | 次车 2 初始位姿（透传） |

### 订阅者

| 话题 | 类型 | 用途 |
|---|---|---|
| `/odom` | Odometry | 主车速度 |
| `/odom_combined` | Odometry | 主车速度（备用） |
| `/map` | OccupancyGrid | 主车地图（次车 1 规划也用） |
| `/PowerVoltage` | Float32 | 主车电压 |
| `/robot_charging_flag` | Bool | 主车充电状态 |
| `feedback_words` | String | LLM 回复 → MQTT 上行 |
| `/robot_1/odom` | Odometry | 次车 1 速度 |
| `/robot_1/soldier_pose` | PoseWithCovarianceStamped | **次车 1 全局位姿** |
| `/robot_1/battery_state` | BatteryState | 次车 1 电池 |
| `/robot_1/scan` | LaserScan | 次车 1 激光（避障） |
| `/robot_2/odom` | Odometry | 次车 2 速度 |
| `/robot_2/amcl_pose` | PoseWithCovarianceStamped | 次车 2 全局位姿 |
| `/robot_2/PowerVoltage` | Float32 | 次车 2 电压 |

---

## ⚙️ 配置参数

`system_launch.py` 已挂载以下参数，可在 launch 中修改：

### MQTT / 基础参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `mqtt_broker` | `localhost` | MQTT Broker 地址 |
| `mqtt_port` | `1883` | MQTT Broker 端口 |
| `mqtt_client_id` | `jetson_robot` | MQTT 客户端 ID |
| `topic_prefix` | `robot_0` | MQTT 主题前缀 |
| `status_interval` | `0.2` | 状态上报周期（秒，5Hz） |

### 次车 1 导航参数（主车代规划）

| 参数 | 默认值 | 单位 | 含义 |
|---|---|---|---|
| `subcar1_robot_radius` | 0.20 | m | 机器人等效半径（A\* 障碍膨胀） |
| `subcar1_safety_margin` | 0.10 | m | 额外安全距离 |
| `subcar1_max_linear` | 0.35 | m/s | 前进速度上限 |
| `subcar1_min_linear` | 0.08 | m/s | 转弯时最低前进 |
| `subcar1_max_angular` | 1.20 | rad/s | 转向速度上限 |
| `subcar1_lookahead` | 0.35 | m | Pure Pursuit 前瞻距离 |
| `subcar1_obstacle_stop` | 0.35 | m | 前方该距离内强制停车 |
| `subcar1_obstacle_slow` | 0.80 | m | 前方该距离内开始减速 |
| `subcar1_arrival_tol` | 0.15 | m | 到达判定距离 |

### 次车 1 实际依赖的话题

| 用途 | 话题 |
|---|---|
| 规划地图 | `/map`（与主车共用） |
| 全局位姿 | `/robot_1/soldier_pose` |
| 激光雷达 | `/robot_1/scan` |

---

## 🚀 快速启动

### 1. 编译

```bash
cd /home/nvidia/wheeltec_ros2
colcon build --packages-select mqtt_bridge_ros2 --symlink-install
source install/setup.bash
```

### 2. 启动（主入口）

```bash
ros2 launch mqtt_bridge_ros2 system_launch.py
```

### 3. 启动 WebSocket ↔ MQTT 代理（可选）

```bash
ros2 run mqtt_bridge_ros2 ws_mqtt_proxy
```

---

## 🧪 验证清单

### 主车 MQTT 桥接

```bash
# 下行
ros2 topic echo /cmd_vel &
mosquitto_pub -h 192.168.1.67 -p 1883 -t "robot_0/cmd_vel" \
  -m '{"linear