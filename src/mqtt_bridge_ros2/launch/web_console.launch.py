import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # ==========================================
    # 1. 引入 rosbridge_server 的 launch 文件
    # ==========================================
    rosbridge_dir = get_package_share_directory('rosbridge_server')
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')
        )
    )

    # ==========================================
    # 2. 直接启动 web_video_server 节点
    # ==========================================
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        # 如果你想把视频端口改成其他的(比如 8081)，可以取消下面这行的注释
        # parameters=[{'port': 8080}] 
    )

    # 将两个服务打包在一起启动
    return LaunchDescription([
        rosbridge_launch,
        web_video_server_node
    ])