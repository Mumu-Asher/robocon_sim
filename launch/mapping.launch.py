"""建图：仿真 + SLAM Toolbox (+ RViz2)。

用法:
    ros2 launch robocon_sim mapping.launch.py
    ros2 launch robocon_sim mapping.launch.py gui:=false rviz:=false   # 完全无界面

建图时另开终端遥控小车:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

建完后保存地图（见 docs/两周项目计划.md 的 M3）。
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
    slam_dir = get_package_share_directory('slam_toolbox')

    args = [
        DeclareLaunchArgument('gui', default_value='true', description='是否显示 Gazebo 窗口'),
        DeclareLaunchArgument('rviz', default_value='true', description='是否启动 RViz2'),
    ]

    # 1. 仿真：场地 + 小车 + 传感器。直接复用 sim.launch.py。
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_dir, 'launch', 'sim.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items())

    # 2. SLAM Toolbox：复用它自带的 launch 文件，只替换参数文件。
    #    use_sim_time 必须为 true，原因见 learning-records/0003。
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(share_dir, 'config', 'slam_toolbox.yaml'),
            'use_sim_time': 'true',
        }.items())

    # 3. RViz2：打开预先配置好的显示项。
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(share_dir, 'config', 'mapping.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log')

    return LaunchDescription(args + [sim, slam, rviz])
