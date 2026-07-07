#!/usr/bin/env python2 
#coding: utf-8

import rospy
import actionlib
import serial
from std_msgs.msg import String, Int32
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Point
from tf_conversions import transformations
from math import pi
import sys
import os
from ar_track_alvar_msgs.msg import AlvarMarkers

# 强制设置编码
reload(sys)
sys.setdefaultencoding('utf-8')

# --- 硬件配置 ---
serialPort = "/dev/ttyUSB2"
baudRate = 9600
try:
    ser = serial.Serial(port=serialPort, baudrate=baudRate, parity="N", bytesize=8, stopbits=1, timeout=1)
except Exception as e:
    print("Serial port error: %s" % e)
    ser = None

# --- 核心调优参数 ---
CALIB_X = 0.01          # 枪口偏置校准（正数向右补，负数向左补）
YAW_TH_ROTATING = 0.08  # 旋转靶对准阈值
YAW_TH_MOVING = 0.11    # 移动靶对准阈值 (改小以更仔细瞄准)
LEAD_SEC_ROTATING = 0.18 # 旋转靶提前量预测 (改大以解决滞后打击)
LEAD_SEC_MOVING = 0.22   # 移动靶提前量预测

SHOOT_ON = b'\x55\x01\x12\x00\x00\x00\x01\x69'
SHOOT_OFF = b'\x55\x01\x11\x00\x00\x00\x01\x68'

# 状态变量
target_id_rotating = 255
target_id_moving = 255

class navigation_demo:
    def __init__(self):
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_cb, queue_size=1)
        
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for move_base server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        
        self.target_id_rotating_sub = rospy.Subscriber("target_id_rotating", Int32, self.target_id_rotating_callback)
        self.target_id_moving_sub = rospy.Subscriber("target_id_moving", Int32, self.target_id_moving_callback)
        
        self.marker_cache = {}
        self.shoot_locked = False

    def ar_cb(self, data):
        """记录 AR 标签位姿并计算滤波后的速度"""
        alpha = 0.4 # 滤波系数
        for marker in data.markers:
            m_id = marker.id
            x = marker.pose.pose.position.x
            y = marker.pose.pose.position.y
            stamp = marker.header.stamp
            
            old = self.marker_cache.get(m_id)
            vx, vy = 0.0, 0.0
            if old is not None:
                dt = (stamp - old['stamp']).to_sec()
                if 0.05 < dt < 1.0: # 确保时间步长有效且数据不陈旧
                    raw_vx = (x - old['x']) / dt
                    raw_vy = (y - old['y']) / dt
                    vx = old['vx'] * (1 - alpha) + raw_vx * alpha
                    vy = old['vy'] * (1 - alpha) + raw_vy * alpha
            
            self.marker_cache[m_id] = {
                'x': x, 'y': y, 'vx': vx, 'vy': vy, 'stamp': stamp
            }

    def shoot_burst(self, count=1, duration=0.1):
        """连射逻辑，增加命中概率"""
        if self.shoot_locked or ser is None:
            return
        self.shoot_locked = True
        
        # 射击前停止机器人
        self.pub.publish(Twist())
        rospy.sleep(0.2)
        
        for i in range(count):
            ser.write(SHOOT_ON)
            rospy.loginfo("[Shoot] Fire burst %d/%d" % (i+1, count))
            rospy.sleep(duration)
            ser.write(SHOOT_OFF)
            if count > 1:
                rospy.sleep(0.15) # 连射间隔
                
        rospy.sleep(0.1)
        self.shoot_locked = False

    def get_marker(self, m_id):
        """获取缓存中的标签信息，检查是否过期"""
        marker = self.marker_cache.get(m_id)
        if marker is None:
            return None
        # 如果数据超过 0.8s 未更新，视为过期
        if (rospy.Time.now() - marker['stamp']).to_sec() > 0.8:
            return None
        return marker

    def aim_and_shoot(self, m_id, yaw_th, gain, timeout, lead_sec=0.0, burst=1, label="target", required_centered_frames=1):
        """对准并射击：使用预测位置和脉冲式转向"""
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        last_stamp = rospy.Time(0)
        centered_count = 0
        
        rospy.loginfo("[Aim] Start pulse aiming for %s" % label)
        
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            marker = self.get_marker(m_id)
            if marker is None:
                self.pub.publish(Twist())
                rospy.sleep(0.05)
                continue
            
            if marker['stamp'] == last_stamp:
                rospy.sleep(0.01)
                continue
            last_stamp = marker['stamp']
            
            # --- 恢复原有脚本的 X/Y 打印信息 ---
            x = marker['x']
            y = marker['y']
            print("X")
            print(x)
            print("Y")
            print(y)
            
            # 预测位置计算
            x_calib = x + CALIB_X
            pred_x = x_calib + marker['vx'] * lead_sec
            
            if abs(pred_x) <= yaw_th:
                centered_count += 1
                if centered_count >= required_centered_frames:
                    # 进入射击窗口
                    rospy.loginfo("[Aim] %s centered (pred_x=%.3f) for %d frames, shooting..." % (label, pred_x, centered_count))
                    self.shoot_burst(count=burst)
                    return True
                else:
                    rospy.loginfo("[Aim] %s centered %d/%d, waiting for stability..." % (label, centered_count, required_centered_frames))
                    rospy.sleep(0.1)
                    continue
            else:
                centered_count = 0
                # 脉冲式转向
                msg = Twist()
                # 增加基础速度以克服小车的静摩擦力
                base_vel = 0.28
                msg.angular.z = -1.0 * (pred_x / abs(pred_x)) * base_vel + (-gain * pred_x)
                
                # 增加单次脉冲的最短时间，确保电机能真正驱动轮子转动
                pulse_time = 0.15 + min(abs(pred_x) * 0.15, 0.15)
                
                self.pub.publish(msg)
                rospy.sleep(pulse_time)
                self.pub.publish(Twist())
                
                # 略微缩短等待时间，让连续微调更连贯
                rospy.sleep(0.15)
        # 超时处理：在规定时间之后如果依然未能完美对准，直接强制射击
        rospy.logwarn("[Aim] Timeout reached (%.1fs) for %s. Forcing shoot." % (timeout, label))
        self.shoot_burst(count=burst)
        return False

    def target_id_rotating_callback(self, msg):
        global target_id_rotating
        target_id_rotating = msg.data

    def target_id_moving_callback(self, msg):
        global target_id_moving
        target_id_moving = msg.data

    def goto(self, p):
        rospy.loginfo("[Navi] Goto %s" % str(p))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = q
        
        self.move_base.send_goal(goal)
        self.move_base.wait_for_result(rospy.Duration(60))
        return self.move_base.get_state() == GoalStatus.SUCCEEDED

    def end(self):
        """比赛结束动作：通过开环控制强行进入狭窄终点，避免局部避障失效"""
        rospy.loginfo("Competition ended, executing open-loop backup to final zone...")
        msg = Twist()
        msg.linear.x = -0.28 # 放慢倒车速度，提升直线稳定性
        msg.linear.y = -0.2
        msg.angular.z = 0.0
        for _ in range(20): # 加长倒退时间，共计1.8秒
            self.pub.publish(msg)
            rospy.sleep(0.1)
        self.pub.publish(Twist())

def publish_audio():
    """原有音频发布函数"""
    pub = rospy.Publisher('audio_topic', String, queue_size=10)
    for _ in range(2):
        pub.publish("audio message")
        rospy.sleep(1)

if __name__ == "__main__":
    rospy.init_node('navigation_demo', anonymous=True)

    # 获取任务点坐标
    goalListX = rospy.get_param('~goalListX', '1.30, 1.43, 1.55, 1.12')
    goalListY = rospy.get_param('~goalListY', '-0.52, -1.69, -2.97, -3.35')
    goalListYaw = rospy.get_param('~goalListYaw', '0, 0, 0, 0')
    goals = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalListX.split(","), goalListY.split(","), goalListYaw.split(","))]
    print(goals) # 恢复原有打印

    rotating_default = rospy.get_param('~target_id_rotating_default', 1)
    moving_default = rospy.get_param('~target_id_moving_default', 2)

    navi = navigation_demo()

    offset_x = 0.0
    offset_y = 0.0

    while not rospy.is_shutdown():
        print ('\n=========================================')
        print ('Please select battery status to start:')
        print (' [1] Normal battery (No offsets)')
        print (' [2] Low battery (Compensate for left/up drift)')
        print ('=========================================')
        user_in = raw_input('Input (1 or 2): ').strip()
        
        if user_in == '1':
            rospy.set_param('/start', True)
            break
        elif user_in == '2':
            # 针对不同任务点的独立补偿矩阵 [X补偿, Y补偿]
            # 假设坐标系：X正向为上(前方)，Y正向为左
            # 如果车实际停得偏上，说明X冲过头，补偿应该设为负(减X)
            # 如果车实际停得偏左，说明Y冲过头，补偿应该设为负(减Y)
            offsets = [
                [-0.07, -0.07],  # 任务点1：偏左多(Y-0.16)，偏上少(X-0.10)
                [-0.12, -0.09],  # 任务点2：偏左(Y-0.15)，微有一点偏上(X-0.05)
                [-0.1,  0.01],  # 任务点3：偏上(X-0.14)，左右无影响
                [-0.13,  0.00]   # 任务点4/终点：暂不干预
            ]
            rospy.loginfo("Low battery mode selected! Applying independent compensations...")
            for i in range(min(len(goals), len(offsets))):
                goals[i][0] += offsets[i][0]
                goals[i][1] += offsets[i][1]
                rospy.loginfo("Goal %d compensated by [X:%.2f, Y:%.2f]", i+1, offsets[i][0], offsets[i][1])
            print("New compensated goals:", goals)
            rospy.set_param('/start', True)
            break
        else:
            print ('Invalid input. Please enter 1 or 2.')

    rospy.loginfo("开始比赛")
    
    # 语音启动序列 (不调用 say，仅保留音频和延迟)
    publish_audio()
    rospy.sleep(28)
    
    # 任务点 1
    navi.goto(goals[0])
    rospy.sleep(1.0)
    navi.shoot_burst(count=1, duration=0.15)
    
    # 任务点 2
    navi.goto(goals[1])
    rospy.sleep(0.5)
    rid = target_id_rotating if target_id_rotating != 255 else rotating_default
    navi.aim_and_shoot(rid, YAW_TH_ROTATING, 1.2, 10.0, lead_sec=LEAD_SEC_ROTATING, burst=2, label="Rotating")
    
    # 任务点 3
    navi.goto(goals[2])
    rospy.sleep(0.5)
    mid = target_id_moving if target_id_moving != 255 else moving_default
    navi.aim_and_shoot(mid, YAW_TH_MOVING, 1.5, 12.0, lead_sec=LEAD_SEC_MOVING, burst=2, label="Moving", required_centered_frames=3)
    
    # 由于终点可能是狭窄的口袋区域，容易触发局部避障失效
    # 我们跳过使用 move_base 去目标点4，直接调用 end() 执行盲退
    # if len(goals) > 3: 
    #     navi.goto(goals[3])

    navi.goto(goals[3])
    rospy.sleep(0.5)
    navi.end()
    rospy.loginfo("!!! All Tasks Done !!!")
