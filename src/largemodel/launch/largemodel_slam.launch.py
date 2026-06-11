import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    Localization = LaunchConfiguration('Localization')
    
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
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('qos',default_value='2'),
        DeclareLaunchArgument('Localization', default_value='false'),        
        
        # 核心 TF 转换
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0.11', '0.0', '0.21', '0.0', '0.0', '0.0', 'base_link', 'laser']
        ),

        # 深度图同步节点
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':True, 'approx_sync_max_interval':0.1, 'use_sim_time':use_sim_time, 'qos':qos}],
            remappings=remappings),

        # RTAB-Map 建图节点
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