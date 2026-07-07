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
# 定义Yaw阈值 0.03
Yaw_th = 0.029
stander = 0.025
offset = 0.01
Yaw_high = stander + offset
Yaw_low =  -stander + offset

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
        global ar_x, ar_x_abs, Yaw_low, flog,Yaw_high
        # 获取所有AR标记
        ar_markers = data
        # 遍历接收到的所有AR标记
        for marker in data.markers:
      
            if marker.id == 1:
      
                ar_x = marker.pose.pose.position.x
               
                if (ar_x >= Yaw_high or ar_x <= Yaw_low):
                    
                    msg = Twist()
                   
                    msg.angular.z = -2.0 * (ar_x)
                    print (ar_x )
                    print (ar_x + offset)
                    print ('noshoot')
                    self.pub.publish(msg)
                
                elif (ar_x < Yaw_high and ar_x > Yaw_low):
                    
                    ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                    print ('shoot')
                    time.sleep(0.08)
                    ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                    time.sleep(0.15)
if __name__ == '__main__':
    try:
        # 创建ARTracker对象
        ar_tracker = ARTracker()
        # 进入ROS事件循环
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

