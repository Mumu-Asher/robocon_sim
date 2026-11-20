# Robocon 仿真：SLAM 建图 + Nav2 导航

在 Gazebo Classic 里，让赛事小车（`robot_urdf`）在赛场（`robocon_ground`）里完成 2D 激光建图和自主导航。

**当前能做到的**：一条命令启动仿真；遥控建图并保存；在 RViz2 里点目标点，小车自己走过去。
**当前做不到的**：仿真里不用 AMCL 定位、不模拟舵轮转向、不上斜坡。详见[已知限制](#已知限制)。

## 环境要求

- Ubuntu 22.04，ROS 2 Humble，Gazebo Classic 11
- 需要的 ROS 包：

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-nav2-bringup ros-humble-slam-toolbox \
                 ros-humble-teleop-twist-keyboard ros-humble-rviz2 ros-humble-robot-state-publisher
```

- 生成导航地图的脚本需要 Python 的 `numpy` 和 `Pillow`（`pip install numpy pillow` 或 `apt install python3-numpy python3-pil`）。

## 快速开始

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

三种启动方式，**一次只开一种**（它们都会启动 Gazebo，同时开两个会冲突）：

| 目的 | 命令 |
|---|---|
| 只看仿真、手动遥控 | `ros2 launch robocon_sim sim.launch.py` |
| 建图（仿真 + SLAM Toolbox + RViz2） | `ros2 launch robocon_sim mapping.launch.py` |
| 导航（仿真 + Nav2 + RViz2） | `ros2 launch robocon_sim nav.launch.py` |

三个 launch 都支持 `gui:=false`（不开 Gazebo 窗口）；建图和导航还支持 `rviz:=false`。

想"清掉旧构建、重编、再启动"，用 `ros2_ws/src/robocon_sim/scripts/` 里的两个脚本，参数会传给 launch：

```bash
./ros2_ws/src/robocon_sim/scripts/reset.sh            # 重编 + 启动仿真
./ros2_ws/src/robocon_sim/scripts/reset_mapping.sh    # 重编 + 启动建图
```

## 完整流程

### 1. 建图

```bash
ros2 launch robocon_sim mapping.launch.py
```

另开一个终端遥控小车：

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

注意：
- **不要开上斜坡**。车身倾斜会让水平扫描面抬高，地图会出现重影和黑点。
- 绕场地一圈，最后回到出生点，让 SLAM 形成闭环。
- 西侧走廊（x 约 −5.4 到 −3.2）很窄，只比车半宽宽 2 cm，不要往里开。

建完后，在 RViz2 里确认地图和场地轮廓一致，再保存（路径换成你自己的）：

```bash
cd ros2_ws
ros2 run nav2_map_server map_saver_cli -f src/robocon_sim/maps/robocon --ros-args -p use_sim_time:=true
```

### 2. 生成导航地图

SLAM 建出的原始地图不能直接导航，需要加工：

```bash
python3 src/robocon_sim/scripts/make_nav_map.py
colcon build                       # 让 install/ 里的地图是最新的
```

它读 `maps/robocon.yaml`，输出 `maps/robocon_nav.yaml`（并生成预览图 `robocon_nav_preview.png`），做四件事：
1. 去掉 3 格以下的孤立黑点；
2. 补上外圈边界墙（墙只有 0.08 m 高，雷达看不到）；
3. 把中央高台、台阶、斜坡标成不可通行；
4. 场地内没扫到的地方一律当作可通行。

**原始地图不会被改动。** 场地几何（平台、斜坡的位置）写死在脚本顶部，改场地时要同步改。

### 3. 导航

```bash
ros2 launch robocon_sim nav.launch.py
```

在 RViz2 工具栏点 **Nav2 Goal**，在地图上点击并拖动，设定目标位置和朝向。

> 导航时没有启动 AMCL，而是发布一个恒等的 `map→odom`。这只在仿真里成立，原因见下面。

## 目录

```
ros2_ws/src/robocon_sim/     ROS 2 包（整个 ros2_ws 可以单独拷贝，不依赖外面的文件夹）
  models/
    robot_urdf/              小车模型（原样保留，仿真时由 launch 临时加工）
    robocon_ground/          赛场模型
  launch/
    sim.launch.py            场地 + 小车 + 传感器 + 速度看门狗
    mapping.launch.py        sim + SLAM Toolbox + RViz2
    nav.launch.py            sim + Nav2 + 固定 map→odom + RViz2
  config/
    slam_toolbox.yaml        建图参数（只写和默认值不同的项）
    nav2_params.yaml         Nav2 参数，每处修改都有"【改】"注释说明原因
    mapping.rviz             建图用的 RViz2 配置
  urdf/sim_plugins.xml       运动、雷达、IMU 插件，启动时插入 br.urdf
  worlds/robocon.world       光照 + 场地
  maps/                      robocon.*（原始）、robocon_nav.*（导航用）
  scripts/
    make_nav_map.py          原始地图 → 导航地图
    cmd_vel_watchdog.py      0.5 秒没新速度指令就停车
    auto_drive.py            按航点自动开车（建图用，不避障）
    reset.sh, reset_mapping.sh
docs/两周项目计划.md          里程碑、实测结论、已知问题
lessons/, reference/          学习用的课程和速查表
```

## 仿真里做了哪些简化

- **运动**：用 `planar_move` 插件直接给车体速度（全向），不模拟舵轮。舵轮转向和伸缩机构在 URDF 里被固定在零位。
- **雷达**：MID360 的位置上放一个 2D 一圈扫描，输出 `/scan`，不是真正的 3D 点云。
- **定位**：导航时 `map→odom` 恒等。仿真里程计读的是 Gazebo 的真值，不会漂。

## 已知限制

完整列表和实测数据见 [docs/两周项目计划.md](docs/两周项目计划.md)，这里列最影响使用的：

1. **仿真里不能用 AMCL**。小车出生点只能看到中央高台的一面直墙，沿墙方向的位置分不出来，AMCL 的估计和真实位置相差 1.4 到 3.5 m。**上真车必须换回 AMCL 或 SLAM Toolbox 的定位模式**，并且要选一个能同时看到两面不平行墙的出生点。
2. **边界墙雷达看不到**（墙只有 0.08 m 高，扫描面离地约 0.23 m）。试过把扫描面降低，车体会挡住 72% 的射线，建图失败。
3. **偶发 `Goal failed`**：偶尔导航会失败，常在东侧圆柱附近，重新点一次目标通常就恢复。原因未查清。
4. IMU 的 z 加速度噪声大，现在没用到；做 FAST-LIO2 前要先解决。

## 常见问题

| 现象 | 怎么办 |
|---|---|
| 小车完全不动 | `ros2 lifecycle get /controller_server` 要看到 `active`。不是 active 就是 Nav2 没启动完，等一会或看终端里的报错 |
| 松开键盘后小车停不住 | 正常情况下 0.5 秒内会自动停（`cmd_vel_watchdog`）。没停就检查这个节点是否在运行：`ros2 node list \| grep watchdog` |
| 启动时 `gzserver` 报错退出 | 通常是上一次的仿真还没关干净。`ps -eo pid,args \| grep gzserver` 找到后按 PID 关掉，不要用 `pkill -f`（容易误杀自己） |
| 车往错的方向走 | 先看 RViz2 里激光红点有没有盖在地图黑线上，盖不上就是位置错了，别去调参数 |
| `ros2 run` 的程序关了还在占资源 | `ros2 run` 会留下子进程（比如 `tf2_echo`），关掉终端不一定杀得掉，用 `ps` 查 |
| Gazebo 窗口里看不到小车，但 RViz2 里 TF、雷达都正常 | 多半是工作区路径里有中文（比如 `~/ROS2基础/`）。Gazebo 处理中文路径会出错，读不到模型的外观文件，但物理、雷达、TF 不读外观文件，所以只有窗口里看不到。`sim.launch.py` 已经绕过这个问题：启动时在 `/tmp/robocon_sim_<uid>/` 下建英文路径的链接，交给 Gazebo 的模型路径都走它。如果还出现，先确认用的是最新的 `sim.launch.py`，再用 `gz model -m br -i \| grep filename` 看网格路径是不是乱码 |
| 日志里有 `no ros2_control tag` | 无害，可以忽略 |

## 改了东西之后要重新构建吗

**要。改任何文件（launch、yaml、脚本、插件、地图、模型）后，都先重新 `colcon build` 再启动**，不然 `install/` 里还是旧文件，改动不会生效。

原因：`install/` 里放的是文件的复制，不是指向源文件的链接。`reset.sh` 用的是普通 `colcon build`，它会把所有文件复制进去；`colcon build --symlink-install` 对有些目录（比如 `models/`）也不会建链接。想确认当前 `install/` 是不是最新的：

```bash
diff -q src/robocon_sim/config/nav2_params.yaml install/robocon_sim/share/robocon_sim/config/nav2_params.yaml && echo 一致
```



把全局规划器从 NavFn 换成 **Smac 2D**
