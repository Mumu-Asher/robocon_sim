"""导航：仿真 + Nav2（规划 + 控制 + 行为树）+ 固定的 map->odom + RViz2。

前提：已经有导航地图 maps/robocon_nav.yaml（由 scripts/make_nav_map.py 生成）。

用法:
    ros2 launch robocon_sim nav.launch.py
    ros2 launch robocon_sim nav.launch.py gui:=false rviz:=false   # 完全无界面

在 RViz2 里点工具栏的 "Nav2 Goal"，在地图上点击并拖动设定目标位置和朝向，小车会自己走过去。
速度链路:  Nav2 -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> cmd_vel_watchdog -> /cmd_vel_safe -> planar_move 插件

关于定位（为什么没有 AMCL）：
  AMCL 靠激光扫描和地图对齐来估计 map->odom。这辆车的雷达扫描面离地 0.23 m，看不到 0.08 m 高的外圈边界墙；
  在出生点只能看到中央高台的一面墙，沿墙方向的位置分不出来，AMCL 实测发散了 3 m 以上。
  （试过把扫描面降到 0.03 m 来看到墙，但车体自己挡住了 72% 的射线，建图失败。）
  仿真里 planar_move 发布的 odom 就是 Gazebo 的真值，没有漂移，所以直接把 map 和 odom 重合：
  发布一个恒等的 map->odom。真车上要换回 AMCL 或 SLAM Toolbox 的定位模式。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('robocon_sim')
    nav2_dir = get_package_share_directory('nav2_bringup')
    params = os.path.join(share_dir, 'config', 'nav2_params.yaml')

    args = [
        DeclareLaunchArgument('gui', default_value='true', description='是否显示 Gazebo 窗口'),
        DeclareLaunchArgument('rviz', default_value='true', description='是否启动 RViz2'),
        DeclareLaunchArgument('map', default_value=os.path.join(share_dir, 'maps', 'robocon_nav.yaml'),
                              description='导航地图'),
    ]

    # 1. 仿真：场地 + 小车 + 传感器 + 速度看门狗
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_dir, 'launch', 'sim.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items())

    # 2. 固定的 map -> odom（恒等变换）。仿真里 odom 是真值，所以 map 和 odom 重合。
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],   # 平移和旋转默认全 0
        parameters=[{'use_sim_time': True}])

    # 3. 地图服务器：把 .yaml/.pgm 发布成 /map，供全局代价地图使用
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params, {'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': True}])
    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}])

    # 4. Nav2 的导航部分：planner、controller、behavior、bt_navigator、velocity_smoother 等，不含 AMCL
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': params,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items())

    # 5. RViz2：用 Nav2 自带的显示配置（地图、代价地图、路径、Nav2 Goal 按钮）
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(nav2_dir, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log')

    return LaunchDescription(args + [sim, map_to_odom, map_server, map_lifecycle, navigation, rviz])
