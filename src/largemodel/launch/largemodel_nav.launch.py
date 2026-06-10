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

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

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
    qos = LaunchConfiguration('qos')
    
    my_map_dir = '/home/nvidia/wheeltec_ros2/src/wheeltec_robot_rtab'
    my_map_file = 'my_map.yaml'
    
    rtabmap_nav_dir = get_package_share_directory('wheeltec_robot_rtab')
    my_param_dir = os.path.join(rtabmap_nav_dir, 'params')
    my_param_file = 'rtabmap_nav_params.yaml'

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    rtabmap_remappings = [
          ('odom', '/odom_combined'),
          ('scan', '/scan'),
          ('rgb/image', '/camera/color/image_raw'), 
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw')]

    rtabmap_parameters = {
          'database_path': '/home/nvidia/wheeltec_ros2/src/wheeltec_robot_rtab/my_room.db',
          'frame_id': 'base_footprint', 
          'use_sim_time': use_sim_time,
          'subscribe_rgbd': True,
          'subscribe_scan': True, 
          'use_action_for_goal': True,
          'qos_image': qos,
          'qos_imu': qos,
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
          'Mem/IncrementalMemory': 'False',
          'Mem/InitWMWithAllNodes': 'True'
    }

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='')
    declare_use_namespace_cmd = DeclareLaunchArgument('use_namespace', default_value='false')
    declare_slam_cmd = DeclareLaunchArgument('slam', default_value='False')
    declare_map_yaml_cmd = DeclareLaunchArgument('map', default_value=os.path.join(my_map_dir, my_map_file))
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_qos_cmd = DeclareLaunchArgument('qos', default_value='2')
    declare_params_file_cmd = DeclareLaunchArgument('params_file', default_value=os.path.join(my_param_dir, my_param_file))
    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true')
    declare_use_composition_cmd = DeclareLaunchArgument('use_composition', default_value='True')
    declare_use_respawn_cmd = DeclareLaunchArgument('use_respawn', default_value='False')
    declare_log_level_cmd = DeclareLaunchArgument('log_level', default_value='info')

    bringup_cmd_group = GroupAction([
        PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
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
            launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time, 'autostart': autostart, 'use_respawn': use_respawn, 'params_file': params_file}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time, 'autostart': autostart, 'params_file': params_file, 'use_composition': use_composition, 'use_respawn': use_respawn, 'container_name': 'nav2_container'}.items()),
    ])

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)

    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0.11', '0.0', '0.21', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )
    ld.add_action(base_to_laser_tf)
    
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

    # 仅启动同步节点和算法节点，不再重复拉起硬件
    ld.add_action(Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[{'approx_sync':True, 'approx_sync_max_interval':0.1, 'use_sim_time':use_sim_time, 'qos':qos}],
        remappings=rtabmap_remappings
    ))

    ld.add_action(Node(
        condition=IfCondition(PythonExpression(['not ', slam])),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[rtabmap_parameters],
        remappings=rtabmap_remappings
    ))

    ld.add_action(bringup_cmd_group)

    return ld