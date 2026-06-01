import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml
from launch.actions import TimerAction

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    
    # 新增 qos 参数变量，与建图文件保持一致
    qos = LaunchConfiguration('qos')
    
    # 直接指向你的自定义地图路径
    my_map_dir = '/home/nvidia/wheeltec_ros2/src/wheeltec_robot_rtab'
    my_map_file = 'my_map.yaml'
    
    rtabmap_nav_dir = get_package_share_directory('wheeltec_robot_rtab')
    my_param_dir = os.path.join(rtabmap_nav_dir, 'params')
    my_param_file = 'rtabmap_nav_params.yaml'
    
    
    wheeltec_bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # ==========================================
    # 【核心注入】：与建图完全一致的 RTAB-Map 参数配置
    # ==========================================
    rtabmap_remappings = [
          ('odom', '/odom_combined'),
          ('scan', '/scan'),
          ('rgb/image', '/camera/color/image_raw'), 
          ('rgb/camera_info', '/camera/color/camera_info'),
          # 【已修复】：换回最稳定的深度图原始话题
          ('depth/image', '/camera/aligned_depth_to_color/image_raw')]

    rtabmap_parameters = {
           # 【新增：直接挂载你保存好的视觉雷达融合数据库】
          'database_path': '/home/nvidia/wheeltec_ros2/src/wheeltec_robot_rtab/my_room.db',
          
          'frame_id': 'base_footprint', 
          'use_sim_time': use_sim_time,
          
          'subscribe_rgbd': True,
          'subscribe_scan': True, 
          
          'use_action_for_goal': True,
          'qos_image': qos,
          'qos_imu': qos,
          
          # 继承你的调优策略
          'Reg/Strategy': '1',           
          'Reg/Force3DoF': 'true',       
          'RGBD/NeighborLinkRefining': 'True',
          'Optimizer/GravitySigma': '0',
          
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
          
          # 【导航模式核心】：关闭增量建图，使用已建立的地图进行纯定位
          'Mem/IncrementalMemory': 'False',
          'Mem/InitWMWithAllNodes': 'True'
    }
    # ==========================================

    # 1. 立即启动底盘
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(wheeltec_bringup_dir, 'launch','turn_on_wheeltec_robot.launch.py')),
    )
    
    # 2. 延迟 3 秒启动雷达
    wheeltec_lidar = TimerAction(
        period=3.0, 
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(wheeltec_bringup_dir, 'launch', 'wheeltec_lidar.launch.py'))
            )
        ]
    )
    
    # 3. 延迟 6 秒启动高功耗的深度相机
    wheeltec_camera = TimerAction(
        period=6.0, 
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(wheeltec_bringup_dir, 'launch', 'wheeltec_camera.launch.py'))
            )
        ]
    )
  
    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # --- 启动参数声明 ---
    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
    declare_use_namespace_cmd = DeclareLaunchArgument('use_namespace', default_value='false', description='Whether to apply a namespace to the navigation stack')
    declare_slam_cmd = DeclareLaunchArgument('slam', default_value='False', description='Whether run a SLAM')
    declare_map_yaml_cmd = DeclareLaunchArgument('map', default_value=os.path.join(my_map_dir, my_map_file), description='Full path to map yaml file to load')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true')
    declare_qos_cmd = DeclareLaunchArgument('qos', default_value='2', description='QoS profiles for RTAB-Map')
    declare_params_file_cmd = DeclareLaunchArgument('params_file', default_value=os.path.join(my_param_dir, my_param_file), description='Full path to the ROS2 parameters file to use for all launched nodes')
    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack')
    declare_use_composition_cmd = DeclareLaunchArgument('use_composition', default_value='True', description='Whether to use composed bringup')
    declare_use_respawn_cmd = DeclareLaunchArgument('use_respawn', default_value='False', description='Whether to respawn if a node crashes.')
    declare_log_level_cmd = DeclareLaunchArgument('log_level', default_value='info', description='log level')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'use_respawn': use_respawn,
                              'params_file': params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0.11', '0.0', '0.21', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )
    ld.add_action(base_to_laser_tf)
    # 声明所有参数
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_qos_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # 启动底层驱动和传感器
    ld.add_action(wheeltec_robot)
    ld.add_action(wheeltec_lidar)
    ld.add_action(wheeltec_camera)

    # 【已修复】：放宽 approx_sync_max_interval 至 0.1，防止 Jetson CPU 满载时丢帧
    ld.add_action(Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[{'approx_sync':True, 'approx_sync_max_interval':0.1, 'use_sim_time':use_sim_time, 'qos':qos}],
        remappings=rtabmap_remappings
    ))

    # 【新增】RTAB-Map 纯定位节点
    ld.add_action(Node(
        condition=IfCondition(PythonExpression(['not ', slam])),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[rtabmap_parameters],
        remappings=rtabmap_remappings
    ))

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld