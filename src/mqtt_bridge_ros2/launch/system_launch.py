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
                          "status_interval":3.0}]),
        Node(package="mqtt_bridge_ros2", executable="webrtc_pusher", output="screen",
             parameters=[{"image_topic":"/camera/color/image_raw","webrtc_port":8082}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share,"launch","web_console.launch.py"))),
        #         ExecuteProcess(cmd=["python3","-m","http.server","8081","--directory","/home/nvidia/wheeltec_ros2/src/wheeltec_robot_rtab"],output="screen"),
    ])
