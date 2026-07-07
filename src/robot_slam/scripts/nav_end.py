#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Int32

# 全局状态变量
target_id_rotating = None  # 存储检测到的导航终点编号
target_id_moving = None  # 存储检测到的运算符

# 语音关键词映射表（可扩展）
target_id_rotating_mapping = {
    "一": 1,  # 关键词到导航点编号的映射
    "二": 2,
    "三": 3,
    "四": 4,
    "五": 5,
}

# 运算符映射表（可扩展）
target_id_moving_mapping = {
    "六": 6,  # 中文运算符到符号的映射
    "七": 7,
    "八": 8
}

def chinese_callback(msg):
	global arrive_pub, target_id_rotating, target_id_moving
	for keyword, value in target_id_rotating_mapping.items():
		if keyword in msg.data:
			target_id_rotating = value
			rospy.loginfo(f"检测到旋转靶关键词: {keyword}, 设置 target_id_rotating = {target_id_rotating}")
			arrive_str = "旋转靶为{}号".format(keyword)
			arrive_pub.publish(arrive_str)

			try:
				target_id_rotating_pub.publish(target_id_rotating)
			except Exception as e:
				rospy.logerr(f"发布旋转靶 target_id 时出错: {e}")
			break

	for keyword, value in target_id_moving_mapping.items():
		if keyword in msg.data:
			target_id_moving = value
			rospy.loginfo(f"检测到移动靶关键词: {keyword}, 设置 target_id_moving = {target_id_moving}")
			arrive_str = "移动靶为{}".format(keyword)
			arrive_pub.publish(arrive_str)
			arrive_str = "比赛开始".format(keyword)
			arrive_pub.publish(arrive_str)

			try:
				target_id_moving_pub.publish(target_id_moving)
			except Exception as e:
				rospy.logerr(f"发布移动靶 target_id 时出错: {e}")
				break



def chinese_subscriber():
    """
    节点初始化函数
    功能：
    1. 初始化ROS节点
    2. 创建发布者/订阅者
    3. 启动消息循环
    """
    global arrive_pub, target_id_rotating_pub, target_id_moving_pub
    # 节点初始化
    rospy.init_node('chinese_subscribr', anonymous=True)
    
    # 创建发布者
    arrive_pub = rospy.Publisher('/voiceWords', String, queue_size=10)  # 语音反馈
    target_id_rotating_pub = rospy.Publisher('target_id_rotating', Int32, queue_size=10)  # 旋转靶
    target_id_moving_pub = rospy.Publisher('target_id_moving', Int32, queue_size=10)  # 移动靶
    
    # 创建订阅者
    rospy.Subscriber("chinese_topic", String, chinese_callback)
    
    rospy.loginfo("Chinese Subscriber node started")
    rospy.spin()

if __name__ == '__main__':
    try:
        chinese_subscriber()
    except rospy.ROSInterruptException:
        pass

