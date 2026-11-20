#!/usr/bin/env python3
"""按航点自动开车，用于建图。简单的比例控制，不避障：航点要提前确认过安全。

用法:
    python3 auto_drive.py                 # 默认路线：沿南、东、北三面走，回到起点
    python3 auto_drive.py --speed 0.25

只在仿真里用。小车是全向的，所以直接朝航点平移，不需要先转向；
朝向保持不变，这样雷达的朝向稳定，对 SLAM 更友好。
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

# 世界坐标 (x, y)。每个点离障碍都 >= 0.45 m（用 SDF 真值检查过）。
# 注意速度指令是在车体坐标系下的，所以下面要用当前朝向做旋转。
ROUTE = [
    (-4.5, -4.9), (0.0, -4.9), (4.2, -4.9),      # 沿南墙向东
    (4.9, -4.0), (4.9, 0.0), (4.9, 4.0),         # 沿东侧走廊向北（x=4.9 离圆柱 0.6 m）
    (4.2, 4.9), (0.0, 4.9), (-4.2, 4.9),         # 沿北墙向西
    (-4.5, 4.0),                                  # 西北角，从这里往南"看"西侧走廊
    (-4.5, -4.5),                                 # 回到起点，形成闭环
]


class Driver(Node):
    def __init__(self, speed):
        super().__init__('auto_drive')
        self.speed = speed
        self.pose = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

    def on_odom(self, m):
        p = m.pose.pose
        yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
        self.pose = (p.position.x, p.position.y, yaw)

    def go(self, tx, ty, tol=0.2, timeout=90.0):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is None:
                continue
            x, y, yaw = self.pose
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)
            if dist < tol:
                return True
            v = min(self.speed, 0.8 * dist + 0.05)          # 接近航点时减速
            wx, wy = v * dx / dist, v * dy / dist           # 世界系速度
            c, s = math.cos(-yaw), math.sin(-yaw)           # 转到车体系
            cmd = Twist()
            cmd.linear.x = c * wx - s * wy
            cmd.linear.y = s * wx + c * wy
            self.pub.publish(cmd)
        return False

    def stop(self):
        self.pub.publish(Twist())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.3)
    args = ap.parse_args()
    rclpy.init()
    node = Driver(args.speed)
    while node.pose is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    print('起点 (%.2f, %.2f)' % node.pose[:2])
    ok = True
    for i, (x, y) in enumerate(ROUTE):
        ok = node.go(x, y)
        print('航点 %d/%d (%.1f, %.1f): %s，当前 (%.2f, %.2f)' % (
            i + 1, len(ROUTE), x, y, '到达' if ok else '超时', *node.pose[:2]))
        if not ok:
            break
    node.stop()
    time.sleep(0.6)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
