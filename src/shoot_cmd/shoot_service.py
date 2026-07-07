#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
统一射击服务节点 (shoot_service) — 优化版

从 improved/ 版本移植, 用于 optimized/ 目录.

功能:
  - 唯一拥有串口 /dev/shoot 的节点
  - 提供 /shoot_cmd 话题订阅 (String, "shoot_once")
  - 互斥保护 + 冷却保护
  - 支持通过 ROS 参数配置

用法:
  rosrun shoot_cmd shoot_service.py
  # 或: rostopic pub /shoot_cmd std_msgs/String "shoot_once"
"""

import rospy
import serial
import time
from std_msgs.msg import String


class ShootService:
    def __init__(self):
        rospy.init_node('shoot_service', anonymous=False)

        port = rospy.get_param('~port', '/dev/shoot')
        baudrate = rospy.get_param('~baudrate', 9600)
        self.cooldown = rospy.get_param('~cooldown', 2.0)

        self.ser = None
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                parity=serial.PARITY_NONE,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.5
            )
            rospy.loginfo("[ShootService] 串口 %s 打开成功 (baud=%d)", port, baudrate)
        except Exception as e:
            rospy.logerr("[ShootService] 串口 %s 打开失败: %s", port, str(e))

        self.last_shoot_time = 0.0
        self.busy = False

        self.sub = rospy.Subscriber('/shoot_cmd', String, self.cmd_callback, queue_size=1)
        self.heartbeat_pub = rospy.Publisher('/shoot_service/heartbeat', String, queue_size=1)

        rospy.loginfo("[ShootService] 初始化完成, 冷却: %.1fs", self.cooldown)

    def cmd_callback(self, msg):
        if msg.data != "shoot_once":
            rospy.logwarn("[ShootService] 未知指令: %s", msg.data)
            return
        if self.busy:
            rospy.logwarn("[ShootService] 射击进行中, 忽略")
            return

        now = time.time()
        elapsed = now - self.last_shoot_time
        if elapsed < self.cooldown:
            rospy.logwarn("[ShootService] 冷却中 (%.1f/%.1fs)", elapsed, self.cooldown)
            return

        self._do_shoot()

    def _do_shoot(self):
        self.busy = True
        self.last_shoot_time = time.time()

        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                rospy.logdebug("[ShootService] ON")
                rospy.sleep(0.08)
                self.ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                rospy.loginfo("[ShootService] 射击完成")
                rospy.sleep(0.02)
            except serial.SerialException as e:
                rospy.logerr("[ShootService] 串口通信失败: %s", str(e))
        else:
            rospy.logwarn("[ShootService] 串口未打开, 模拟射击")

        self.busy = False

    def run(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.heartbeat_pub.publish("alive")
            rate.sleep()


if __name__ == '__main__':
    try:
        ShootService().run()
    except rospy.ROSInterruptException:
        pass
