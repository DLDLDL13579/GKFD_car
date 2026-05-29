import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

     use_sim_time = LaunchConfiguration('use_sim_time', default=False)
     use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
     
     wheeltec_rrt_dir = get_package_share_directory('wheeltec_robot_rrt')
     # 【修改 1】：把你之前调教好的 rtabmap 导航参数目录找出来
     wheeltec_rtab_dir = get_package_share_directory('wheeltec_robot_rtab')
     
     # 【核心修改 2】：改造 Nav2 的启动
     # 我们不再让它启动官方默认的导航，而是启动我们定制好的 RTAB-Map 导航架构
     nav2_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
               # 注意：这里调用的是 Nav2 官方的 bringup，非常纯净
               os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
               launch_arguments={
                              # 'slam': "False",  <-- Nav2 专心导航，不抢建图的活
                              'use_sim_time': use_sim_time,
                              # 【强制使用你的 50x50 避障参数】
                              'params_file': os.path.join(wheeltec_rtab_dir, 'params', 'rtabmap_nav_params.yaml'),
                              'autostart': 'True'
                              }.items()
     )

     rrt_exploration = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(wheeltec_rrt_dir, 'launch', 'rrt_exploration', 'rrt_exploration.launch.py')
          ),
     )
     
     # 【修改 3】：完全注释掉官方的 slam_toolbox，因为我们会自己开 RTAB-Map
     # wheeltec_slam = IncludeLaunchDescription(...)
     
     action_servers = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(wheeltec_rrt_dir, 'launch', 'rrt_exploration','action_servers.launch.py')
          )
     )

     robot_picker = Node(
               package='wheeltec_robot_rrt',
               executable='robot_picker',
               name='robot_picker',
               parameters=[{'bt_xml_filename':os.path.join(wheeltec_rrt_dir, 'behaviour_trees', 'robot_picker_behaviour_tree.xml')}])

     robot_pose_publisher = Node(
            package='wheeltec_robot_rrt',
            executable='robot_pose_publisher',
            name='robot_pose_publisher',
            )
            
     return LaunchDescription([
          #wheeltec_slam,  <-- 保持注释
          nav2_include,
          rrt_exploration,
          action_servers,
          robot_picker,
          robot_pose_publisher,
          use_sim_time_arg
     ])