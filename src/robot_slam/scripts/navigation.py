#!/usr/bin/env python

#coding: utf-8

import rospy
import math
import actionlib
import serial
import time
from std_msgs.msg import String
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi
from std_msgs.msg import String
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import os
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Twist


serialPort = "/dev/shoot"
baudRate = 9600


ser = serial.Serial(port=serialPort, baudrate=baudRate, parity="N", bytesize=8, stopbits=1)

Yaw_th_1 = 0.03
flog0 = None
flog1 = None
flog = None
Yaw_th_2 = 0.029
Min_y = -0.30
Max_y = -0.28   
class navigation_demo:
    def __init__(self):
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_cb)
        self.sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_cb_xz)
        self.find_sub = rospy.Subscriber('/object_position', Point, self.find_cb)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
    def find_cb(self, data):
        global flog0, flog1, flog
        
        #point_msg = data 
        
        #flog0 = point_msg.x - 320
        
        #flog1 = abs(flog0)
        
        #if abs(flog1) > 0.5 and flog == 0:
            
            #msg = Twist()
            
            #msg.angular.z = -0.01 * flog0
            
            #self.pub.publish(msg)
        
        if flog == 0:
           
            ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
            print ('shoot')
            time.sleep(0.08)
            ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
            flog = 1
	    navi.goto(goals[1])
	    flog = 2
    def move_cb(self):
        global time
        
        time = 0
        
        msg = Twist()
        msg.linear.x = -1.7
        msg.linear.y = -0.4
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        while time < 10:
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time += 1
    def ar_cb(self, data):
        global ar_x, ar_x_abs, Yaw_th_1, flog
  
        ar_markers = data
   
        for marker in data.markers:
      
            if marker.id == 2 and flog == 4:
      
                ar_x = marker.pose.pose.position.x
               
                ar_x_abs = abs(ar_x)
                
                if ar_x_abs >= Yaw_th_1:
                    
                    msg = Twist()
                   
                    msg.angular.z = -2.0 * ar_x
                    print ('noshoot')
                    self.pub.publish(msg)
                
                elif ar_x_abs < Yaw_th_1 and flog == 4:
                    
                    ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                    print ('shoot')
                    time.sleep(0.08)
                    ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                    flog = 5
		    navi.goto(goals[3])
		    flog = 6
		    print('go')
		    move = navigation_demo()
        	    move.move_cb()

    def ar_cb_xz(self, data):
        global ar_x_0, ar_y_0, Yaw_th_2,ar_x0_abs,Min_y,Max_y, flog
      
        ar_markers = data
  
        for marker in data.markers:
           
            if marker.id == 2 and flog == 2:
                
                ar_x_0 = marker.pose.pose.position.x
                ar_y_0 = marker.pose.pose.position.y
                ar_x0_abs = abs(ar_x_0)
                
                if ar_x0_abs >= Yaw_th_2:
                    
                    msg = Twist()
                    
                    msg.angular.z = -2.0 * ar_x_0
                    print ('noshoot')
                    self.pub.publish(msg)
                	
                    if ar_y_0 >= Min_y and ar_y_0 <= Max_y and  flog == 2:
                         
                        ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                        print ('shoot')
                        time.sleep(0.08)
                        ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                        flog = 3
			navi.goto(goals[2])
			flog = 4

    def set_pose(self, p):
        if self.move_base is None:
            return False

        x, y, th = p

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0.0, 0.0, th/180.0*pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        self.set_pose_pub.publish(pose)
        return True

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s"%(status, result))

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        msg = feedback
        #rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)

    def goto(self, p):
        rospy.loginfo("[Navi] goto %s"%p)
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!"%p)
        return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True
if __name__ == "__main__":
    rospy.init_node('navigation_demo',anonymous=True)
    goalListX = rospy.get_param('~goalListX', '2.0, 2.0,2.0')
    goalListY = rospy.get_param('~goalListY', '2.0, 4.0,2.0')
    goalListYaw = rospy.get_param('~goalListYaw', '0, 90.0,2.0')

    goals = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalListX.split(","),goalListY.split(","),goalListYaw.split(","))]
    print ('Please 1 to continue: ')
    input = raw_input()
    r = rospy.Rate(1)
    r.sleep()
    print('go to 1')
    navi = navigation_demo()
    navi.goto(goals[0])
    rospy.sleep(1)
    flog = 0
    while not rospy.is_shutdown():
          r.sleep()
