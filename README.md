

# 🤖 GKFD_car (Wheeltec ROS2 Humble 综合机器人开发平台)

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue) ![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange) ![License](https://img.shields.io/badge/License-MIT-green)

本项目是一个基于 **ROS2 Humble** 版本的移动机器人控制系统，深度整合了轮趣科技 (Wheeltec) 的底层驱动与丰富的上层算法。项目不仅涵盖了多维度的环境建图与智能导航，还深入实现了基于**快速扩展随机树 (RRT)** 的自主环境探索，以及基于**行为树 (Behavior Trees)** 的高级任务级调度（如视觉寻物、机械臂抓取等）。



## 🌟 核心技术栈与功能特性

### 1. 🧠 高级任务调度与行为树 (Behavior Trees)
本项目不仅实现了点到点的移动，还具备执行复杂逻辑流的能力：
* **BT 插件定制 (`bt_plugins`)**：实现了 `find_coloured_box`（寻找特定颜色物体）、`approach_coloured_box`（精准靠近）和 `pick_coloured_box`（控制机械臂抓取）等自定义行为树节点。
* **Action Servers**：包含 `find_robot_action_server` 和 `pick_robot_action_server`，用于异步处理耗时任务并提供实时反馈。
* **导航与抓取协同**：`robot_navigator.py` 与 `robot_picker.cpp` 结合，实现了从“寻找”到“导航”再到“抓取”的完整闭环。

### 2. 🗺️ 混合多算法 SLAM (2D & 3D)
为了适应不同的硬件传感器与场景需求，系统集成了全套主流 SLAM 算法：
* **RTAB-Map (`wheeltec_robot_rtab`)**：支持 RGB-D / 激光雷达混合建图，生成高精度 3D 点云与 2D 栅格地图，适用于复杂室内环境。
* **Cartographer (`wheeltec_cartographer`)**：谷歌开源的高性能 2D 激光建图方案，拥有极强的闭环检测能力。
* **SLAM Toolbox (`wheeltec_slam_toolbox`)**：ROS2 官方推荐，支持在线建图与离线地图的生命周期管理。
* **Gmapping & ORB-SLAM2 (`wheeltec_robot_slam`)**：提供经典的 2D 激光建图，以及针对单目/双目/RGBD（Astra, Realsense）相机的视觉 SLAM 解决方案。

### 3. 🧭 RRT 自主探索与智能导航 (Nav2)
* **自主前沿探索 (`rrt_exploration`)**：无需人工干预！实现了 Global RRT 与 Local RRT 算法，机器人会自动寻找地图上的未知区域边界 (Frontier)，配合 Mean Shift 聚类算法，自动下发导航目标点，直至遍历整个空间。
* **Nav2 动态规划**：基于代价地图 (Costmap)，支持局部动态避障、全局路径规划及多点巡航。

### 4. 🦿 多底盘兼容与模型支持 (`wheeltec_robot_urdf`)
极其完善的机器人模型库，支持 Rviz2 实时可视化与物理仿真：
* **底盘结构**：两轮差速、四轮驱动 (4WD)、麦克纳姆轮、全向轮、阿克曼转向、履带 (Tank) 底盘。
* **机械臂扩展**：预置了多款 MoveIt 兼容的四轴/六轴机械臂模型。



## 📂 核心工作空间结构

```text
wheeltec_ros2/
├── src/
│   ├── rrt_exploration/           # RRT 自主探索算法核心源码 (Global/Local RRT, 聚类过滤)
│   ├── bt_plugins/                # 行为树 (Behavior Tree) 动作节点 C++ 插件
│   ├── wheeltec_robot_rtab/       # RTAB-Map 3D建图与 Nav2 导航配置
│   ├── wheeltec_cartographer/     # Cartographer 2D 激光建图配置
│   ├── wheeltec_slam_toolbox/     # SLAM Toolbox 2D 建图配置
│   ├── wheeltec_robot_slam/       # Gmapping 算法与 ORB-SLAM2 视觉建图
│   ├── wheeltec_rrt_msg/          # 自定义消息、服务与动作接口 (PointArray, ChangePosition 等)
│   ├── wheeltec_robot_urdf/       # 所有型号机器人的 URDF/xacro 模型及 STL 网格文件
│   └── wheeltec_rviz2/            # 各个功能模块的 Rviz2 可视化配置
├── collect_files.sh               # 辅助开发：文件批量提取与处理脚本
└── ROS2-V3.5(humble)常用指令.txt    # 开发备忘录：常用 ROS2 调试与运行指令

```


## 🚀 快速开始

### 1. 依赖安装与工作空间编译

建议在 Ubuntu 22.04 (ROS2 Humble) 环境下运行。

```bash
# 进入工作空间
cd ~/wheeltec_ros2

# 安装项目所需的 ROS2 依赖包
rosdep install --from-paths src --ignore-src -r -y

# 编译整个工作空间 (包含自定义的 C++ 节点和 Msg)
colcon build --symlink-install

# 刷新环境变量
source install/setup.bash

```

### 2. 核心功能运行指令速查

| 功能模块 | 运行指令示例 | 说明 |
| --- | --- | --- |
| **基础可视化** | `ros2 launch wheeltec_rviz2 wheeltec_rviz.launch.py` | 启动基础的 Rviz2 界面，加载默认机器人模型 |
| **Cartographer 建图** | `ros2 launch wheeltec_cartographer cartographer.launch.py` | 启动 2D 激光高精度建图 |
| **RTAB-Map 视觉建图** | `ros2 launch wheeltec_robot_rtab rtabmap.launch.py` | 启动 3D 视觉与激光融合建图 |
| **自主导航 (Nav2)** | `ros2 launch wheeltec_robot_rtab wheeltec_nav2_rtab.launch.py` | 加载已有地图，启动全局路径规划与避障 |
| **RRT 自主探索** | `ros2 launch wheeltec_robot_nav2 rrt_exploration.launch.py` | *(需配合导航包)* 启动随机树扩张，自动探索未知区域 |

*(💡 注：详细参数调整和更多进阶指令，请参考根目录下的 `ROS2-V3.5(humble)常用指令.txt` 文件)*

---

## ⚠️ 开发者注意事项

1. **大文件拦截**：由于地图数据库（如 RTAB-Map 生成的 `*.db`）和录制的数据包（`*.bag`）体积庞大，本项目已在 `.gitignore` 中配置了严格的忽略规则。切勿强制推送超过 100MB 的文件至 GitHub，以免触发 LFS 报错。
2. **自定义消息编译**：本项目包含 `wheeltec_rrt_msg` 接口定义，在初次拉取代码或修改 `msg/srv/action` 文件后，请务必完全重新编译该包，否则行为树和 Action Server 将无法正常通信。
3. **ORB-SLAM2 词典**：如果使用 ORB 视觉 SLAM，请确保提前解压缩 `src/wheeltec_robot_slam/orb_slam_2_ros-ros2/orb_slam2/Vocabulary/ORBvoc.txt.tar.gz`。

---

*Powered by ROS2 Humble | Maintained by @DLDLDL13579*


### 💡 提交这个文件的步骤：
你只需要在 VS Code 中点开根目录下的 `README.md`，把以上内容全部粘贴进去，保存。然后在左侧的源代码管理（Git）栏里填写提交信息，点击 **“提交”**，再点击 **“同步更改”** 推送到你的 GitHub 仓库即可。
