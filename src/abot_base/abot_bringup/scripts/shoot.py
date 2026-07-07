#!/usr/bin/env python
#coding=utf-8

import rospy
import serial
import time
from std_msgs.msg import String

serialPort = "/dev/ttyUSB2"
baudRate = 9600
ser = serial.Serial(port=serialPort, baudrate=baudRate, parity="N", bytesize=8, stopbits=1)


class AbotShoot():
    def __init__(self):
        # Give the node a name
        rospy.init_node('abot_shoot', anonymous=False)
        
        # Subscribe to the /shoot topic
        rospy.Subscriber('/shoot', String, self.shoot_continue)
        
        rospy.loginfo("Shoot to ar_tag")
        
    def shoot_continue(self, msg):
        ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')   #开
        print (0)      
        time.sleep(0.1)
        ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')  #关

if __name__ == '__main__':
    try:
        AbotShoot()
        rospy.spin()
    except:
        pass
        

        

