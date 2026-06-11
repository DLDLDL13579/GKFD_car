# 🤖 GKFD_car — 轮趣机器人 ROS2 Humble 综合开发平台

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue) ![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange) ![Platform](https://img.shields.io/badge/Platform-Jetson%20%7C%20x86-brightgreen) ![License](https://img.shields.io/badge/License-MIT-green) [![GitHub Repo](https://img.shields.io/badge/GitHub-GKFD__car-181717?logo=github)](https://github.com/DLDLDL13579/GKFD_car)

---

**GKFD_car** 是一个基于 **ROS2 Humble** 的轮趣科技 (Wheeltec) 移动机器人全栈控制系统。项目将机器人底层驱动与上层智能算法深度融合，集成了从 **SLAM 建图**、**自主导航**、**RRT 自主探索**到**视觉伺服**、**行为树任务调度**、**大模型语音交互**的完整技术栈，覆盖了移动机器人开发的绝大多数核心场景。

> 工作路径: `~/wheeltec_ros2`  
> 目标硬件: 轮趣科技全系列底盘 + NVIDIA Jetson (Orin NX / Xavier NX / Nano) 或 x86 工控机

---

## 📋 目录

- [核心技术栈](#-核心技术栈)
- [硬件平台](#-硬件平台)
- [项目架构](#-项目架构)
- [包清单与功能索引](#-包清单与功能索引)
- [环境要求](#-环境要求)
- [快速开始](#-快速开始)
- [功能使用指南](#-功能使用指南)
- [开发者指南](#-开发者指南)
- [常见问题与排错](#-常见问题与排错)

---

## 🌟 核心技术栈

| 层级 | 技术 | 说明 |
|------|------|------|
| **建图** | RTAB-Map / Cartographer / SLAM Toolbox / Gmapping / ORB-SLAM2 | 2D 激光 + 3D RGB-D 混合 SLAM |
| **定位** | AMCL (自适应蒙特卡洛定位) | 基于粒子滤波的 2D 概率定位 |
| **导航** | Nav2 (Navigation2) | 全局/局部路径规划 + 动态避障 |
| **探索** | RRT (Rapidly-exploring Random Tree) | 前沿边界自主探索 + Mean Shift 聚类 |
| **任务编排** | Behavior Trees (行为树) | 视觉寻物 → 靠近 → 机械臂抓取完整闭环 |
| **视觉** | ArUco / KCF / DNN 检测 | 二维码定位、目标跟踪、深度学习检测 |
| **语音** | 麦克风阵列 + TTS | 语音识别、语音合成、声源定位 |
| **大模型** | Ollama + 本地 LLM | 本地大语言模型对话机器人 |
| **充电** | 自动回充 | 自主导航至充电桩并对接 |
| **多机** | 多机器人协同 | 多车编队控制与通信 |

---

## 🔧 硬件平台

### 支持底盘类型

| 类型 | 说明 |
|------|------|
| 两轮差速 | 标准差分驱动 |
| 四轮驱动 (4WD) | 四轮独立驱动 |
| **麦克纳姆轮** | 全向移动 (含 URDF 模型) |
| 全向轮 | 三/四轮全向 |
| 阿克曼转向 | 类似汽车转向 |
| 履带底盘 | 坦克式履带 |

### 传感器支持

| 传感器 | 驱动包 |
|--------|--------|
| 激光雷达 (Lidar) | `wheeltec_lidar_ros2` |
| 深度相机 (Astra) | `ros2_astra_camera` |
| 深度相机 (RealSense) | `realsense-ros` |
| USB 摄像头 | `usb_cam-ros2` |
| 麦克风阵列 | `wheeltec_mic`, `wheeltec_mic_aiui` |
| IMU | `wheeltec_imu` |
| GPS | `wheeltec_gps` |
| 摇杆 (Joystick) | `wheeltec_joy` |

### 机械臂支持

预置了多款与 MoveIt 兼容的四轴/六轴机械臂模型、URDF 描述文件，可与导航系统协同完成抓取任务。

---

## 🏗️ 项目架构

```
wheeltec_ros2/
├── src/
│   ├── turn_on_wheeltec_robot/          # 🔌 机器人底层驱动
│   ├── wheeltec_robot_urdf/             # 🦾 URDF 机器人模型库
│   │
│   ├── wheeltec_robot_slam/             # 🗺️ Gmapping & ORB-SLAM2
│   ├── wheeltec_cartographer/           # 🗺️ Cartographer 配置
│   ├── wheeltec_slam_toolbox/           # 🗺️ SLAM Toolbox 配置
│   ├── wheeltec_robot_rtab/             # 🗺️ RTAB-Map 3D 建图 + Nav2
│   │
│   ├── navigation2-humble/              # 🧭 Nav2 导航栈 (完整源码)
│   ├── wheeltec_robot_nav2/             # 🧭 Nav2 导航配置
│   ├── rrt_exploration/                 # 🌲 RRT 自主探索算法
│   ├── wheeltec_robot_rrt2/             # 🌲 RRT2 探索启动配置
│   │
│   ├── bt_plugins/                      # 🧠 行为树自定义节点
│   ├── wheeltec_path_follow/            # ➡️ 路径跟踪
│   ├── simple_follower_ros2/            # 👤 人体跟随
│   ├── wheeltec_robot_kcf/              # 🎯 KCF 目标跟踪
│   │
│   ├── auto_recharge_ros2/              # 🔋 自动回充
│   ├── nav2_waypoint_cycle/             # 🔁 多目标点巡航
│   │
│   ├── aruco_ros-humble-devel/          # 📐 ArUco 二维码检测
│   ├── dnn_detect/                      # 🧠 DNN 深度学习检测
│   │
│   ├── ollama_ros_chat/                 # 💬 本地大模型对话
│   ├── tts_make_ros2/                   # 🔊 语音合成 TTS
│   ├── wheeltec_mic/                    # 🎤 麦克风驱动
│   ├── wheeltec_mic_aiui/               # 🗣️ AIUI 智能语音
│   │
│   ├── wheeltec_imu/                    # 📐 IMU 驱动
│   ├── wheeltec_lidar_ros2/             # 📡 激光雷达驱动
│   ├── wheeltec_gps/                    # 🛰️ GPS 驱动
│   ├── wheeltec_joy/                    # 🎮 遥控器驱动
│   ├── ros2_astra_camera/               # 📷 Astra 深度相机
│   ├── realsense-ros/                   # 📷 Intel RealSense
│   ├── usb_cam-ros2/                    # 📷 USB 摄像头
│   ├── web_video_server-ros2/           # 📺 Web 视频推流
│   │
│   ├── wheeltec_robot_msg/              # 📦 自定义消息
│   ├── wheeltec_rrt_msg/                # 📦 RRT 消息/服务/动作
│   ├── interfaces/                      # 📦 通用接口定义
│   │
│   ├── wheeltec_rviz2/                  # 🎛️ Rviz2 预置配置
│   ├── wheeltec_bodyreader/             # 🧍 人体姿态识别
│   ├── wheeltec_robot_keyboard/         # ⌨️ 键盘控制
│   ├── wheeltec_multi/                  # 👥 多机器人协同
│   ├── qt_ros_test/                     # 🖥️ Qt 测试界面
│   │
│   ├── depend/                          # 📚 第三方依赖
│   │   ├── ackermann_msgs-ros2/
│   │   ├── serial_ros2/
│   │   └── tf2_tools/
│   │
│   └── ...其他 ROS 官方包
│
├── mecanum_pro.urdf                     # 麦克纳姆轮 URDF 示例
├── install.sh                           # Ollama 安装脚本
├── collect_files.sh                     # 文件收集工具
├── ollama.service                       # Ollama systemd 服务
└── combined_files.txt                   # 关键配置文件汇总
```

---

## 📦 包清单与功能索引

### 🔌 底层驱动 (Bringup)

| 包名 | 语言 | 功能 |
|------|------|------|
| `turn_on_wheeltec_robot` | Python/Launch | **核心启动包** — 串口通信、摄像头(激光雷达)启动、EKF 融合、底盘驱动 |
| `wheeltec_robot_urdf` | URDF/Xacro | 全系底盘 + 机械臂 URDF 模型、STL 网格文件，支持 Rviz2 可视化 |

### 🗺️ SLAM 建图

| 包名 | 算法 | 传感器 | 特点 |
|------|------|--------|------|
| `wheeltec_robot_rtab` | RTAB-Map | RGB-D + LiDAR | **3D 点云 + 2D 栅格**混合建图，闭环检测强，带 Nav2 导航配置 |
| `wheeltec_cartographer` | Cartographer | 2D LiDAR | Google 开源方案，**低漂移、强闭环**，适合长廊等退化环境 |
| `wheeltec_slam_toolbox` | SLAM Toolbox | 2D LiDAR | ROS2 官方推荐，支持**建图/定位/地图序列化**生命周期管理 |
| `wheeltec_robot_slam` | Gmapping / ORB-SLAM2 | LiDAR / 单目-双目-RGBD | 经典 2D 建图 + 视觉 SLAM (Astra/Realsense) |

### 🧭 导航与探索

| 包名 | 功能 |
|------|------|
| `navigation2-humble` | Nav2 完整框架: AMCL 定位 + 全局/局部规划 + 行为树导航 |
| `wheeltec_robot_nav2` | Nav2 参数配置与 launch 封装 |
| `rrt_exploration` | **RRT 前沿探索**核心算法: Global RRT + Local RRT + Mean Shift 聚类 |
| `wheeltec_robot_rrt2` | RRT 探索启动配置与 Nav2 集成 |
| `wheeltec_path_follow` | 路径跟踪控制器 |
| `simple_follower_ros2` | 人体跟随算法 |

### 🧠 智能任务

| 包名 | 功能 |
|------|------|
| `bt_plugins` | **行为树 C++ 插件**: `find_coloured_box`, `approach_coloured_box`, `pick_coloured_box` 等自定义节点 |
| `auto_recharge_ros2` | 自动充电: 保存充电桩位置 → 自主导航返回 → 红外/视觉对接 |
| `nav2_waypoint_cycle` | **多点巡航**: 设置多个航点并循环导航 |
| `wheeltec_robot_kcf` | KCF 目标跟踪 (视觉跟随) |
| `wheeltec_bodyreader` | 人体骨骼姿态识别与交互 |
| `dnn_detect` | 深度学习目标检测 (YOLO 等) |

### 📷 视觉传感

| 包名 | 功能 |
|------|------|
| `aruco_ros-humble-devel` | ArUco 二维码检测与定位 (ROS2 移植版)，含 4x4_1000, 582 等多系列词典 |
| `ros2_astra_camera` | Orbbec Astra / Astra Pro 深度相机驱动 |
| `realsense-ros` | Intel RealSense D400/T200 系列驱动 |
| `usb_cam-ros2` | 通用 USB 摄像头驱动 |
| `web_video_server-ros2` | HTTP Web 视频流推送 (浏览器实时查看画面) |

### 💬 语音与大模型

| 包名 | 功能 |
|------|------|
| `ollama_ros_chat` | Ollama 本地大语言模型 ROS 封装, 实现**语音 → LLM → 机器人动作**闭环 |
| `tts_make_ros2` | 文本转语音 TTS |
| `wheeltec_mic` | 麦克风阵列驱动 (声源定位 + 语音采集) |
| `wheeltec_mic_aiui` | 讯飞 AIUI 智能语音交互 |

### 📦 自定义接口

| 包名 | 内容 |
|------|------|
| `wheeltec_robot_msg` | 基础消息定义 |
| `wheeltec_rrt_msg` | **RRT 专用接口** — `PointArray.msg`, `ChangePosition.srv`, 动作接口等 |
| `interfaces` | 通用消息/服务/动作自定义 |

### 🎛️ 工具与可视化

| 包名 | 功能 |
|------|------|
| `wheeltec_rviz2` | 各功能模块的 Rviz2 预配置 (SLAM / Nav2 / RRT / RTAB / 模型) |
| `wheeltec_joy` | 遥控/摇杆控制支持 |
| `wheeltec_robot_keyboard` | 键盘远程控制 |
| `wheeltec_multi` | 多机器人通信与编队控制 |
| `qt_ros_test` | Qt 图形界面测试工具 |

---

## 💻 环境要求

### 系统

| 项目 | 要求 |
|------|------|
| 操作系统 | **Ubuntu 22.04** (推荐) 或兼容 Linux |
| ROS | **ROS2 Humble Hawksbill** (必须) |
| 架构 | `amd64` (x86) 或 `arm64` (Jetson Orin NX / Xavier NX / Nano) |
| Python | ≥ 3.10 |
| C++ 编译器 | GCC ≥ 11 (与 ROS2 Humble 一致) |

### 推荐硬件

- 轮趣科技 (Wheeltec) 底盘套装（含电机驱动板、STM32 底层控制器）
- NVIDIA Jetson Orin NX / Xavier NX（作为上位机）
- 激光雷达（思岚 / 万集 / 镭神等，已适配）
- 深度相机（Orbbec Astra 或 Intel RealSense D435 等）

### 核外依赖

| 依赖 | 用途 |
|------|------|
| `serial` (C++库) | 串口通信与底层驱动板交互 |
| `ackermann_msgs` | 阿克曼转向消息类型 |
| `nav2` (Navigation2) | 导航框架 |
| `BehaviorTree.CPP` | 行为树引擎 (v3.x) |
| `rtabmap_ros` | RTAB-Map SLAM |
| `cartographer_ros` | Cartographer SLAM |
| `slam_toolbox` | SLAM Toolbox |
| `orbslam2_ros` | ORB-SLAM2 视觉 SLAM |
| `moveit2` | 机械臂运动规划 |
| `ollama` | 本地大语言模型推理 |

---

## 🚀 快速开始

### 1️⃣ 创建工作空间 & 获取代码

```bash
mkdir -p ~/wheeltec_ros2/src
cd ~/wheeltec_ros2/src
git clone https://github.com/DLDLDL13579/GKFD_car.git .

# 或使用您自己的仓库地址
# git clone https://github.com/您的用户名/GKFD_car.git .
```

### 2️⃣ 安装系统依赖

```bash
cd ~/wheeltec_ros2

# 1) 更新 rosdep 并安装依赖
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 2) 安装额外系统依赖
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    ros-humble-rtabmap-ros \
    ros-humble-slam-toolbox \
    ros-humble-behavior-tree \
    ros-humble-moveit2 \
    python3-serial \
    python3-yaml \
    python3-colcon-common-extensions
```

### 3️⃣ 安装 Ollama (大模型)

```bash
# 方式一: 使用项目提供的安装脚本
cd ~/wheeltec_ros2
chmod +x install.sh
./install.sh

# 方式二: 官方脚本
curl -fsSL https://ollama.com/install.sh | sh

# 拉取轻量模型
ollama pull qwen2.5:1.5b   # 适合边缘设备
# ollama pull llama3.2:1b   # 备用选择
```

### 4️⃣ 编译工作空间

```bash
cd ~/wheeltec_ros2

# 安装 colcon 编译工具 (如未安装)
sudo apt install python3-colcon-common-extensions

# 编译全部包 (--symlink-install 允许修改 Python 文件后免重新编译)
colcon build --symlink-install

# 如遇编译失败，可单独编译关键包排查
colcon build --packages-select wheeltec_rrt_msg
colcon build --packages-select turn_on_wheeltec_robot

# 刷新环境
source install/setup.bash
```

> 💡 建议将 source 命令加入 `~/.bashrc`:
> ```bash
> echo "source ~/wheeltec_ros2/install/setup.bash" >> ~/.bashrc
> source ~/.bashrc
> ```

### 5️⃣ 快速验证

```bash
# 查看所有可用 launch 文件
find src -name "*.launch.py" | head -20

# 启动 Rviz2 查看机器人模型
ros2 launch wheeltec_rviz2 wheeltec_rviz.launch.py
```

---

## 📖 功能使用指南

### 🗺️ SLAM 建图

#### 方案 A: Cartographer (2D 激光 — 推荐日常使用)

```bash
# 终端 1: 启动底盘与传感器
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 终端 2: 启动 Cartographer 建图
ros2 launch wheeltec_cartographer cartographer.launch.py

# 终端 3: 键盘控制移动建图
ros2 run wheeltec_robot_keyboard wheeltec_keyboard
```

- 一手遥控机器人走遍环境
- Cartographer 会自动闭环修正漂移
- 建图完成后保存地图:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

#### 方案 B: RTAB-Map (3D RGB-D + 激光 — 高精度)

```bash
ros2 launch wheeltec_robot_rtab rtabmap.launch.py
```

- 同时生成 2D 栅格地图 + 3D 点云
- 支持视觉词袋闭环检测
- 地图自动保存在 `~/.ros/rtabmap.db`

#### 方案 C: SLAM Toolbox (2D 激光 — 生命周期管理)

```bash
ros2 launch wheeltec_slam_toolbox slam_toolbox.launch.py
```

#### 方案 D: ORB-SLAM2 (仅视觉)

```bash
# 首次需解压词典文件
tar -xzf src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Vocabulary/ORBvoc.txt.tar.gz

ros2 launch wheeltec_robot_slam orb_slam2.launch.py
```

---

### 🧭 导航 (Nav2)

#### 使用已有地图导航

```bash
# 终端 1: 启动底盘
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 终端 2: 启动 RTAB-Map 导航 (含 AMCL 定位)
ros2 launch wheeltec_robot_rtab wheeltec_nav2_rtab.launch.py

# 终端 3: 在 Rviz2 中点击 "Nav2 Goal" 下发目标点
```

- 机器人在 Rviz2 中自动定位
- 支持动态避障、代价地图更新
- 可在导航过程中切换全局规划器

---

### 🌲 RRT 自主探索

无需人工干预，机器人自动对未知区域进行边界探索:

```bash
# 终端 1: 启动底盘 + 激光雷达
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 终端 2: 启动 RRT 探索 (含 Nav2)
ros2 launch wheeltec_robot_nav2 rrt_exploration.launch.py
```

**工作流程**:

1. **Global RRT** 在地图空闲区域生长随机树
2. **Frontier Detection** 检测前沿边界点
3. **Mean Shift 聚类** 将前沿点聚簇
4. **Nav2 导航** 自动下发最优目标点
5. 重复直至全空间覆盖

---

### 🔁 多点巡航

```bash
# 启动导航
ros2 launch wheeltec_robot_rtab wheeltec_nav2_rtab.launch.py

# 运行巡航节点
ros2 run nav2_waypoint_cycle waypoint_cycle
```

通过 Rviz2 的 "Publish Point" 工具依次点击目标点，机器人会按顺序循环导航。

---

### 🔋 自动回充

```bash
# 保存充电桩位置 (将机器人开到充电桩旁)
ros2 run auto_recharge_ros2 auto_recharger

# 脚本交互:
# - 按 's' 保存当前位置为充电桩坐标
# - 按 'g' 出发前往充电桩
# - 靠近后自动红外对接充电
```

充电桩位置保存在 `auto_recharge_ros2/Charger_Position.json`。

---

### 👤 人体跟随

```bash
ros2 launch simple_follower_ros2 simple_follower.launch.py
```

使用激光雷达或深度相机，机器人自动跟踪前方行人。

---

### 📐 ArUco 二维码检测

```bash
ros2 launch aruco_ros aruco_recognize.launch.py
```

支持多种二维码字典: `4x4_1000`, `582` 等，可用于:
- 导航目标点标记
- 机械臂抓取定位
- 移动机器人视觉着陆

---

### 💬 大模型对话 (Ollama)

```bash
# 确保 Ollama 服务运行中
ollama serve

# 启动 ROS 对话节点
ros2 run ollama_ros_chat ollama_chat_node
```

**集成链路**:

```
用户语音 → 麦克风 → ROS → Ollama LLM → ROS → 导航/机械臂/语音回复
```

支持离线运行，无网络依赖，适合工业 / 室内场景。

---

### 📺 Web 视频监控

```bash
ros2 launch web_video_server-ros2 web_video_server.launch.py
```

浏览器访问 `http://<机器人IP>:8080` 实时查看摄像头画面。

---

### 🔊 TTS 语音合成

```bash
ros2 run tts_make_ros2 tts_node
```

机器人语音播报导航状态、检测结果等。

---

## 🛠️ 开发者指南

### 添加新包

```bash
cd ~/wheeltec_ros2/src

# Python 包
ros2 pkg create my_package --build-type ament_python --dependencies rclpy

# C++ 包
ros2 pkg create my_package --build-type ament_cmake --dependencies rclcpp
```

### 添加自定义接口

```bash
cd ~/wheeltec_ros2/src

# 消息包标准结构
ros2 pkg create my_interfaces --build-type ament_cmake
```

创建后在 `my_interfaces/msg/` 下添加 `.msg` 文件，并在 `CMakeLists.txt` 中注册:
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyMessage.msg"
  "srv/MyService.srv"
)
```

> ⚠️ 修改接口后必须完全重新编译该包:
> ```bash
> colcon build --packages-select my_interfaces --cmake-clean-cache
> ```

### 行为树开发

`bt_plugins` 包提供自定义 BT 节点模板:

```cpp
// 示例: 自定义动作节点
class FindColouredBox : public BT::StatefulActionNode {
public:
  FindColouredBox(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

  BT::NodeStatus onStart() override {
    // 发起视觉检测请求
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    // 轮询检测结果
    if (found) return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {
    // 取消任务
  }
};
```

### 文件与数据管理

- **地图数据库**: `*.db` (RTAB-Map) 体积较大, 已在 `.gitignore` 中排除
- **数据包**: `*.bag` 已排除, 切勿推送到 GitHub
- **编译产物**: `build/`, `install/`, `log/` 已排除
- **IDE 配置**: `.vscode/`, `.idea/` 已排除

### 开发辅助脚本

```bash
# 收集关键配置文件到 combined_files.txt (方便审查)
./collect_files.sh
```

---

## ❓ 常见问题与排错

### Q1: 编译失败 — `wheeltec_rrt_msg` 找不到

```
解决方法:
colcon build --packages-select wheeltec_rrt_msg --cmake-clean-cache
source install/setup.bash
colcon build --symlink-install
```

### Q2: 串口无法打开 — `/dev/ttyUSB0` 权限不足

```bash
# 将当前用户加入 dialout 组
sudo usermod -a -G dialout $USER
# 重新登录或重启后生效

# 临时提权
sudo chmod 666 /dev/ttyUSB0
```

### Q3: Ollama 下载模型太慢

```bash
# 使用国内镜像 (环境变量)
export OLLAMA_HOST=0.0.0.0

# 手动从 ModelScope 下载:
# https://modelscope.cn/models/qwen/Qwen2.5-1.5B-Instruct-GGUF
```

### Q4: RTAB-Map 建图卡顿

- 降低地图分辨率: 在 `rtabmap_nav_params.yaml` 中调整 `map_occupancy_resolution: 0.05`
- 关闭可视化: 在 launch 文件中设置 `visualization:=false`
- Jetson 设备考虑开启 `NVMM` 硬件编码

### Q5: Nav2 导航无法规划路径

- 确认 `map` 话题有正确发布
- 检查 `transform` 树: `ros2 run tf2_tools view_frames.py`
- 确保里程计话题 `odom` 数据正常
- 查看代价地图: Rviz2 中添加 `/global_costmap` 和 `/local_costmap` 显示

### Q6: 行为树节点不执行

```bash
# 检查自定义 BT 节点是否被插件系统加载
ros2 run bt_plugins list_nodes  # (如可用)

# 确认 XML 文件中的节点名称与 C++ 注册名称一致
# 检查 wheeltec_rrt_msg 是否已编译
```

---

## 📄 License

本项目基于 **MIT License** 开源。

- 核心导航框架 (Nav2、Cartographer 等) 遵循其各自开源协议
- 轮趣科技 (Wheeltec) 底层驱动相关代码版权归原厂商所有
- 用户自己的二次开发代码遵循 MIT License

---

## 🙏 致谢

- [轮趣科技 (Wheeltec)](https://www.wheeltec.net/) — 底盘硬件与底层 SDK
- [ROS2 Navigation2](https://github.com/ros-planning/navigation2) — 导航框架
- [RTAB-Map](https://github.com/introlab/rtabmap) — 3D SLAM
- [Cartographer](https://github.com/ros2/cartographer) — 2D SLAM
- [Ollama](https://ollama.com) — 本地大模型推理
- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) — 视觉 SLAM

---

<p align="center">
  <sub>Powered by ROS2 Humble · Maintained by <a href="https://github.com/DLDLDL13579">@DLDLDL13579</a></sub><br>
  <sub>项目地址: <a href="https://github.com/DLDLDL13579/GKFD_car">github.com/DLDLDL13579/GKFD_car</a></sub>
</p>
