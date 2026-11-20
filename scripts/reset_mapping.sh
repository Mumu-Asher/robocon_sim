#!/usr/bin/env bash
# 清掉旧构建，重新编译并启动仿真。可以在任何目录运行；额外参数会传给 launch，
# 例如: ./reset.sh gui:=false

# 先进入工作区根目录（本脚本在 src/robocon_sim/scripts/ 下，往上三级）。
# 用脚本自己的位置定位，不依赖你在哪个目录运行它。
cd "$(dirname "$(readlink -f "$0")")/../../.." || exit 1

# 必须先 source ROS，colcon 才找得到 ament_cmake 等依赖。
source /opt/ros/humble/setup.bash &&
rm -rf build/ log/ install/ &&
colcon build &&
source ./install/setup.bash &&
ros2 launch robocon_sim mapping.launch.py "$@"
