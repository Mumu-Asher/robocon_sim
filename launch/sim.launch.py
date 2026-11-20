"""启动 Gazebo 仿真：场地 + 小车(br) + 雷达/IMU/里程计。

用法:
    ros2 launch robocon_sim sim.launch.py            # 带 Gazebo 窗口
    ros2 launch robocon_sim sim.launch.py gui:=false # 无窗口（SSH、批量测试）
"""
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build_robot_description(share_dir):
    """读取原始 br.urdf，加工成 Gazebo 能直接用的版本。"""
    models_dir = os.path.join(share_dir, 'models')
    with open(os.path.join(models_dir, 'robot_urdf', 'br.urdf'), encoding='utf-8') as f:
        urdf = f.read()

    # 加工 0：去掉第一行 <?xml ... encoding='utf-8'?>。
    # spawn_entity.py 内部的 XML 解析器不接受带 encoding 声明的字符串。
    urdf = re.sub(r'^\s*<\?xml[^>]*\?>', '', urdf)

    # 加工 1：mesh 路径。br.urdf 里写的是相对路径 "meshes/xxx.STL"，
    # Gazebo 不知道相对谁，必须换成绝对的 file:// 路径。
    mesh_root = os.path.join(models_dir, 'robot_urdf', 'meshes')
    urdf = urdf.replace('filename="meshes/', f'filename="file://{mesh_root}/')

    # 加工 2：冻结舵轮转向和伸缩机构。仿真里没有控制器，
    # 这些关节会在重力下乱晃，所以先固定在 URDF 零位（前支撑放下）。
    # continuous（轮子）保留，让轮子可以空转。
    urdf = re.sub(r'type="(revolute|prismatic)"', 'type="fixed"', urdf)

    # 加工 2.5：降低轮子摩擦。车体由 planar_move 直接给速度，轮子不需要靠摩擦驱动；
    # 摩擦为 1.0 时，轮子和地面的侧向摩擦会给车体一个多余的力矩，空闲时车会慢慢自转。
    urdf = re.sub(r'<mu([12])>[^<]*</mu[12]>', r'<mu\1>0.01</mu\1>', urdf)

    # 加工 3：把仿真插件插到 </robot> 之前。
    with open(os.path.join(share_dir, 'urdf', 'sim_plugins.xml'), encoding='utf-8') as f:
        plugins = f.read()
    return urdf.replace('</robot>', plugins + '\n</robot>')


def generate_launch_description():
    share_dir = get_package_share_directory('robocon_sim')
    gazebo_dir = get_package_share_directory('gazebo_ros')
    models_dir = os.path.join(share_dir, 'models')

    args = [
        DeclareLaunchArgument('gui', default_value='true', description='是否显示 Gazebo 窗口'),
        DeclareLaunchArgument('x', default_value='-4.5', description='小车出生点 x'),
        DeclareLaunchArgument('y', default_value='-4.5', description='小车出生点 y'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='小车出生朝向'),
    ]

    # 让 Gazebo 找得到 model://robocon_ground
    model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        models_dir + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', ''))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(share_dir, 'worlds', 'robocon.world'),
            'gui': LaunchConfiguration('gui'),
        }.items())

    # 发布 robot_description 话题和 TF（base_link -> 各 link）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': build_robot_description(share_dir),
            'use_sim_time': True,
        }],
        output='screen')

    # 从 robot_description 话题把小车放进 Gazebo。z 给 0.3，让它自己落到地面。
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'br', '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.3', '-Y', LaunchConfiguration('yaw')],
        output='screen')

    # 速度看门狗：松开遥控后自动停车，见 scripts/cmd_vel_watchdog.py
    watchdog = Node(
        package='robocon_sim',
        executable='cmd_vel_watchdog.py',
        parameters=[{'timeout': 0.5}],
        output='screen')

    return LaunchDescription(args + [model_path, gazebo, robot_state_publisher, spawn, watchdog])
