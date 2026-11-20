#!/usr/bin/env python3
"""速度看门狗：转发 /cmd_vel 到 /cmd_vel_safe，超过 timeout 秒没有新指令就发零速度。

为什么需要它：
  - teleop_twist_keyboard 只在按键时发一条速度，松手后什么也不发。
  - Gazebo 的 planar_move 插件没有超时，会一直保持最后一条速度，所以小车停不住。
  - Nav2 持续发速度，结束目标时发零速度，经过这个节点也正常工作。

数据流:  teleop / Nav2 -> /cmd_vel -> [本节点] -> /cmd_vel_safe -> planar_move 插件
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

class CmdVelWatchdog(Node):
    def __init__(self):
        super().__init__('cmd_vel_watchdog')
        self.declare_parameter('timeout', 0.5)     # 秒，超过这么久没收到新指令就停车
        self.timeout = self.get_parameter('timeout').value
        self.last_rx = None                        # 最近一次收到指令的时间（用系统时钟，不依赖仿真时间）
        self.stopped = True                        # 已经发过零速度，避免一直重复发

        self.pub = self.create_publisher(Twist, 'cmd_vel_safe', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_timer(0.05, self.check)        # 20 Hz 检查一次

    def on_cmd(self, msg):
        self.last_rx = time.monotonic()
        self.stopped = False
        self.pub.publish(msg)

    def check(self):
        if self.stopped or self.last_rx is None:
            return
        if time.monotonic() - self.last_rx > self.timeout:
            self.pub.publish(Twist())              # 全零 = 停车
            self.stopped = True


def main():
    rclpy.init()
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
