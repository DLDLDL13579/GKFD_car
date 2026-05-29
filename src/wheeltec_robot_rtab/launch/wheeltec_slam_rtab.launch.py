import os
import launch
import launch.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    Localization = LaunchConfiguration('Localization')

    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch','turn_on_wheeltec_robot.launch.py')),
    )
    
    # 【已修改】：雷达延迟 3 秒启动，避开相机抢电
    wheeltec_lidar = TimerAction(
        period=3.0, 
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'wheeltec_lidar.launch.py'))
            )
        ]
    )
    
    wheeltec_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir,'launch', 'wheeltec_camera.launch.py')),
    )
    
    parameters={
          'frame_id':'base_footprint', 
          'use_sim_time':use_sim_time,
          
          'subscribe_rgbd':True,
          'subscribe_scan':True, 
          
          'use_action_for_goal':True,
          'qos_image':qos,
          'qos_imu':qos,
          
          'Reg/Strategy':'1',           
          'Reg/Force3DoF':'true',       
          'RGBD/NeighborLinkRefining':'True',
          'Optimizer/GravitySigma':'0',
          
          'Grid/FromDepth': 'false',         
          'Grid/3D': 'false',                
          'Grid/RangeMax': '10.0',           
          'Grid/RangeMin': '0.2',            
          
          'Grid/MaxObstacleHeight': '1.2',   
          'Grid/MinObstacleHeight': '0.05',  
          
          'Grid/NoiseFilteringMinNeighbors': '2', 
          'Grid/NoiseFilteringRadius': '0.05',
          'Grid/RayTracing': 'true',         
          'Grid/CellSize': '0.05',           
    }
    
    remappings=[
          ('odom', '/odom_combined'),
          ('scan', '/scan'),
          ('rgb/image', '/camera/color/image_raw'), 
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw')]

    return LaunchDescription([
        wheeltec_robot, wheeltec_lidar, wheeltec_camera,

        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('qos',default_value='2'),
        DeclareLaunchArgument('Localization', default_value='false'),        
        
        # 【核心修复 3】：添加雷达的物理位置 TF 转换！(x=0.12, z=0.23)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0.11', '0.0', '0.21', '0.0', '0.0', '0.0', 'base_link', 'laser']
        ),

        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':True, 'approx_sync_max_interval':0.1, 'use_sim_time':use_sim_time, 'qos':qos}],
            remappings=remappings),

        Node(
            condition=IfCondition(Localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),      
            
        Node(
            condition=UnlessCondition(Localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),
    ])