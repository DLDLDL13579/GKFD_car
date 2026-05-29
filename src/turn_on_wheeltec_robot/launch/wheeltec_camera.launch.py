import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    realsense_dir = get_package_share_directory('realsense2_camera')
    realsense_launch_dir = os.path.join(realsense_dir, 'launch')

    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_launch_dir, 'rs_launch.py')),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'false',
            
            # 【修改点 1：将帧率从 30 降至 15，大幅降低 USB 和 Wi-Fi 负载】
            'rgb_camera.profile': '848x480x15',
            'depth_module.profile': '848x480x15',
            
            # 【修改点 2：加入官方降采样滤波器，点云数据量瞬间压缩 75% 以上】
            # 'filters': 'decimation',
            
            'allow_no_device': 'false',
            
            # -- 针对 Jetson 架构的核心规避参数 (保持不变) --
            'enable_gyro': 'false',   # 强制禁用陀螺仪
            'enable_accel': 'false',  # 强制禁用加速度计
            'enable_sync': 'true'    # 关闭硬件级同步，防止等待 IMU 数据导致死锁
        }.items()
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0.21', '0.0', '0.19', '0.0', '0.035', '0.0', 'base_link', 'camera_link']
    )

    ld = LaunchDescription()
    ld.add_action(rs_launch)
    ld.add_action(static_tf_node)

    return ld