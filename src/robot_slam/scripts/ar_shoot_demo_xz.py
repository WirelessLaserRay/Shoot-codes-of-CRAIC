#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 上面这行是为了告诉操作系统，这是一个Python脚本，可以直接运行

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
import serial
import time
from std_msgs.msg import String

# 设置串口和波特率
serialPort = "/dev/shoot"
baudRate = 9600

# 打开串口
ser = serial.Serial(port=serialPort, baudrate=baudRate, parity="N", bytesize=8, stopbits=1)
# 定义Yaw阈值
Yaw_th = 0.03
Min_y = -0.195
Max_y = -0.18
class ARTracker:
    def __init__(self):
        # 初始化ROS节点，命名为'ar_tracker_node'，并设置为匿名节点
        rospy.init_node('ar_tracker_node', anonymous=True)
        # 创建一个订阅者，订阅AR标记的消息，消息类型为AlvarMarkers，回调函数为ar_cb
        self.sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_cb)
        # 创建一个发布者，用于发布Twist类型的消息到/cmd_vel话题
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)

    # AR标记消息的回调函数
    def ar_cb(self, data):
        global ar_x,ar_y, ar_x_abs, Yaw_th,Min_y,Max_y
        # 获取所有AR标记
        ar_markers = data
        # 遍历接收到的所有AR标记
        for marker in data.markers:
            # 如果AR标记的ID为0
            if marker.id == 2:
                # 获取AR标记的x坐标
                ar_x = marker.pose.pose.position.x
		ar_y = marker.pose.pose.position.y
                # 计算AR标记x坐标的绝对值
                ar_x_abs = abs(ar_x)
                # 如果AR标记的x坐标绝对值大于等于Yaw阈值
                if ar_x_abs >= Yaw_th:
                    # 创建一个Twist消息
                    msg = Twist()
                    # 设置消息的角速度AR标记x坐标的相反值（*-1）
                    msg.angular.z = -6.0 * ar_x
                    # 发布Twist消息
                    self.pub.publish(msg)
		    if ar_y>=Min_y and ar_y<=Max_y:
       		 	# 发送射击指令
			ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
			print ('ok')
			# 等待0.1秒
			time.sleep(0.08)
			# 发送停止射击指令
			ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')

                # 如果AR标记的x坐标绝对值小于Yaw阈值
                #elif ar_x_abs < Yaw_th:
                    #print "ok"

if __name__ == '__main__':
    try:
        # 创建ARTracker对象
        ar_tracker = ARTracker()
        # 进入ROS事件循环
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

