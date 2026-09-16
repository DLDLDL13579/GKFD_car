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

# ============================================================================
# 主车导航启动（AMCL 定位版）  2026-09-15
#
# 与 largemodel_nav.launch.py 的唯一区别：定位由 rtabmap 换成 AMCL。
#   原因：~/.ros/rtabmap.db 是 09-08 建的图，而 my_map.yaml 是 09-14 换成
#   的狗图，两者不是同一次建图的产物，rtab-map 的重定位在该坐标系里对不上。
#   AMCL 直接使用 my_map.yaml（= dog_map_20260910，与 robot1/机器狗同图），
#   使三车处于同一 map 坐标系，send_tfodom 分享的位姿才有可比性。
#
# 帧链：map --(AMCL)--> odom_combined --(EKF)--> base_footprint --(static)--> laser
#   注意 odom_combined 这个名字由主车 EKF 决定（ekf.yaml: odom_frame=odom_combined），
#   rtabmap_nav_params.yaml 的 amcl 段已按此配置 odom_frame_id，勿改成默认 odom，
#   否则会再次出现 "Tf has two or more unconnected trees"。
#
# 旧的 largemodel_nav.launch.py（rtabmap 版）完整保留，可随时回退。
# ============================================================================

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

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    # 禁用 FastDDS SHM 传输（项目惯例，与本车 robot1-dl-* 单元、小车/狗的 unit 一致）。
    # 背景：SHM 端口锁残留会报 "open_and_lock_file failed"，导致 DDS 服务发现失败，
    # lifecycle_manager 调不动 map_server/amcl，两者永远停在 unconfigured。
    disable_shm_envvar = SetEnvironmentVariable('RMW_FASTRTPS_USE_SHM', 'false')

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

    # ==== 全局地图发布者 map_server（加载 my_map.yaml = 狗图）====
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file},
                    {'use_sim_time': use_sim_time}]
    )

    # ==== AMCL 定位（替换原 rtabmap）====
    # 参数取自 rtabmap_nav_params.yaml 的 amcl 段：
    #   global_frame_id: map / odom_frame_id: odom_combined / scan_topic: scan
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    # 注意：节点名沿用 lifecycle_manager_map_server（不改名），
    # 因为 system_manager 的功能状态识别依赖节点名，改名会影响前端显示。
    # 但管辖范围已扩展到 amcl —— AMCL 是 lifecycle 节点，无人管理则永不 active。
    lifecycle_manager_map_server = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(disable_shm_envvar)

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

    ld.add_action(bringup_cmd_group)

    # 挂载 map_server、amcl 及其 lifecycle 管理器
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager_map_server)

    return ld
