import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    voc_file = LaunchConfiguration('voc_file')

    # 【核心修改】：精准匹配 D455 相机的原生输出话题
    remappings = [
        # 彩色图映射
        ('/camera/rgb/image_raw', '/camera/color/image_raw'),
        # 深度图映射（必须使用对齐后的深度图，否则 RGB-D SLAM 无法提取 3D 特征）
        ('/camera/depth_registered/image_raw', '/camera/aligned_depth_to_color/image_raw'),
        # 相机内参映射
        ('/camera/camera_info', '/camera/color/camera_info'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory("orb_slam2_ros"),
                'ros', 'config', 'params_d435_rgbd.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'voc_file',
            default_value=os.path.join(
                get_package_share_directory("orb_slam2_ros"),
                'orb_slam2', 'Vocabulary', 'ORBvoc.txt'),
            description='Full path to vocabulary file to use'),

        Node(
            parameters=[
                params_file,
                {"voc_file": voc_file,
                 "use_sim_time": use_sim_time},
            ],
            package='orb_slam2_ros',
            executable='orb_slam2_ros_rgbd',  # 已修复 ROS2 Humble 的语法警告
            name='orb_slam2_rgbd',            # 已修复 ROS2 Humble 的语法警告
            output='screen',
            remappings=remappings
        )
    ])