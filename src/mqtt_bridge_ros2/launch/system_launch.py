from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share = get_package_share_directory("mqtt_bridge_ros2")
    return LaunchDescription([
        Node(package="mqtt_bridge_ros2", executable="system_manager", output="screen",
             parameters=[{"mqtt_broker":"localhost","mqtt_port":1883,
                          "mqtt_client_id":"jetson_robot","topic_prefix":"robot_0",
                          "status_interval":0.2,
                          # ▼ 次车1 主车代规划:导航参数 ▼
                          "subcar1_robot_radius":   0.20,   # 机器人等效半径 m
                          "subcar1_safety_margin":  0.10,   # 额外安全距离 m
                          "subcar1_max_linear":     0.35,   # 前进速度上限 m/s
                          "subcar1_min_linear":     0.08,   # 转弯时最低前进 m/s
                          "subcar1_max_angular":    1.20,   # 转向速度上限 rad/s
                          "subcar1_lookahead":      0.35,   # Pure Pursuit 前瞻 m
                          "subcar1_obstacle_stop":  0.35,   # 该距离内强制停车 m
                          "subcar1_obstacle_slow":  0.80,   # 该距离内开始减速 m
                          "subcar1_arrival_tol":    0.15,   # 到达判定距离 m
                         }]),
        Node(package="mqtt_bridge_ros2", executable="webrtc_pusher", output="screen",
             parameters=[{"image_topic":"/camera/color/image_raw","webrtc_port":8082}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share,"launch","web_console.launch.py"))),
        #         ExecuteProcess(cmd=["python3","-m","http.server","8081","--directory","/home/nvidia/wheeltec_ros2/src/wheeltec_robot_rtab"],output="screen"),
    ])