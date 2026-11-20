"""启动 Gazebo 仿真：场地 + 小车(br) + 雷达/IMU/里程计。

用法:
    ros2 launch robocon_sim sim.launch.py            # 带 Gazebo 窗口
    ros2 launch robocon_sim sim.launch.py gui:=false # 无窗口（SSH、批量测试）
"""
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def ascii_models_dir(models_dir):
    """返回一个只含英文字符的目录，里面是指向 models/ 下各模型的符号链接。

    为什么需要：Gazebo 处理含中文的路径会出错。实测工作区放在 ~/ROS2基础/ 下，Gazebo 看到的网格路径变成
    "ROS2\\372@"，小车和场地的外观文件读不到，Gazebo 窗口里看不到小车（物理、雷达、TF 都正常，所以很隐蔽）。
    交给 Gazebo 的路径一律走这个目录，工作区放在什么名字的目录下都能用。
    符号链接不复制文件（小车网格有 43 MB），重复启动只会刷新链接。
    """
    stage = os.path.join(tempfile.gettempdir(), 'robocon_sim_%d' % os.getuid())
    os.makedirs(stage, exist_ok=True)
    for name in os.listdir(models_dir):
        src = os.path.realpath(os.path.join(models_dir, name))
        dst = os.path.join(stage, name)
        if os.path.islink(dst):
            if os.path.realpath(dst) == src:
                continue
            os.unlink(dst)               # 只删链接本身，不碰目标
        os.symlink(src, dst)
    return stage


def build_robot_description(share_dir):
    """读取原始 br.urdf，加工成 Gazebo 能直接用的版本。"""
    models_dir = os.path.join(share_dir, 'models')
    with open(os.path.join(models_dir, 'robot_urdf', 'br.urdf'), encoding='utf-8') as f:
        urdf = f.read()

    # 加工 0：去掉第一行 <?xml ... encoding='utf-8'?>。
    # spawn_entity.py 内部的 XML 解析器不接受带 encoding 声明的字符串。
    urdf = re.sub(r'^\s*<\?xml[^>]*\?>', '', urdf)

    # 加工 1：mesh 路径。br.urdf 里写的是相对路径 "meshes/xxx.STL"，
    # Gazebo 不知道相对谁，必须换成绝对的 file:// 路径，并且路径里不能有中文（见 ascii_models_dir）。
    mesh_root = os.path.join(ascii_models_dir(models_dir), 'robot_urdf', 'meshes')
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

    # 让 Gazebo 找得到 model://robocon_ground。同样走纯英文路径，原因见 ascii_models_dir。
    model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        ascii_models_dir(models_dir) + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', ''))

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
