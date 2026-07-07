#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
MH5YR2 优化版任务导航节点 (navigation_optimized.py)

基于 MH5YR2_Optimization_Plan.md 实现:

  方案二.1 — 导航盲射:
    1号固定靶弃用 find_object_2d, 精确设置导航位姿,
    到达后直接射击, 不做视觉对准. 收益: 释放 ~50MB 内存.

  方案二.2 — Stop-and-Shoot (单次采样对准):
    2/3号靶: 到达导航点 → 停稳 → rospy.wait_for_message 获取单帧AR数据
    → 计算旋转补偿量 → 执行一次旋转动作 → 停稳 → 射击.
    收益: 避免 5fps 下连续控制反馈导致的左右震荡.

  方案三.1 — 空间锚定法:
    当 AR/camera 捕获到目标时, 结合当前 EKF 位姿,
    计算目标在 map 坐标系下的绝对坐标.
    后续对准参考 map 系下的静态点, 而非实时图像像素.

  方案三.2 — 控制频率解耦:
    视觉更新: 5Hz (回调被动更新缓存)
    控制循环: 20Hz (主动 Timer 维持恒定底盘指令)

话题接口:
  订阅:
    /odom              (Odometry)    EKF 融合里程计, ~20Hz+
    /ar_pose_marker    (AlvarMarkers) AR 标签位姿, ~5Hz
    target_id_rotating (Int32)       语音解析的旋转靶 ID
    target_id_moving   (Int32)       语音解析的移动靶 ID
  发布:
    /cmd_vel           (Twist)       底盘控制 (queue_size=1)
    /shoot_cmd         (String)      射击指令 → /shoot_service
    /voiceWords        (String)      语音反馈
"""

import rospy
import math
import actionlib
import threading
from std_msgs.msg import String, Int32
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from nav_msgs.msg import Odometry
from tf_conversions import transformations
from math import pi


# ============================================================================
#  配置参数
# ============================================================================

# 瞄准容差
YAW_TH_ROTATING = 0.065   # 旋转靶对准阈值 (rad, ~3.7deg)
YAW_TH_MOVING   = 0.02    # 移动靶对准阈值 (rad, ~1.1deg)

# 控制参数
CONTROL_RATE       = 20     # 控制频率 Hz (方案三.2)
VISION_STALE_SEC   = 0.3    # AR 观测数据过期时间 (5fps ~= 200ms 间隔)
PID_P_ROTATING     = 1.8    # 旋转靶 P 控制增益
PID_P_MOVING       = 2.4    # 移动靶 P 控制增益

# Stop-and-Shoot 参数 (方案二.2)
STOP_DURATION      = 1.0    # 停稳等待时间 (秒)
ROTATE_SETTLE_SEC  = 0.5    # 旋转后停稳时间 (秒)
SHOOT_COOLDOWN_SEC = 2.0    # 射击冷却 (秒)

# 导航超时
NAV_TIMEOUT_SEC = 60.0


# ============================================================================
#  任务状态机 (替代原始 globals flog/case)
# ============================================================================

class MissionState:
    WAIT_START    = "WAIT_START"
    NAV_P1        = "NAV_P1"
    SHOOT_P1      = "SHOOT_P1"        # 盲射 (方案二.1)
    NAV_P2        = "NAV_P2"
    ALIGN_SAMPLE  = "ALIGN_SAMPLE"    # Stop-and-Shoot 单次采样 (方案二.2)
    ROTATE_P2     = "ROTATE_P2"       # 补偿旋转
    SHOOT_P2      = "SHOOT_P2"
    NAV_P3        = "NAV_P3"
    ALIGN_SAMPLE3 = "ALIGN_SAMPLE3"
    ROTATE_P3     = "ROTATE_P3"
    SHOOT_P3      = "SHOOT_P3"
    NAV_END       = "NAV_END"
    FINISHED      = "FINISHED"
    ERROR         = "ERROR"


# ============================================================================
#  空间锚定目标缓存 (方案三.1)
# ============================================================================

class AnchoredTarget:
    """
    空间锚定法:
      当视觉捕获目标时, 结合当前 EKF 位姿,
      将目标位置从相机坐标系转换到 map 坐标系.
      后续控制参考此静态 map 坐标, 实现视觉→控制解耦.
    """

    def __init__(self):
        self.valid = False
        self.map_x = 0.0
        self.map_y = 0.0
        self.stamp = rospy.Time(0)

    def is_fresh(self, max_age_sec=0.3):
        return self.valid and (rospy.Time.now() - self.stamp).to_sec() < max_age_sec

    def clear(self):
        self.valid = False

    def update(self, ar_x, ar_y, robot_x, robot_y, robot_yaw):
        """
        将 AR 标签在相机坐标系中的位置 (ar_x, ar_y) 转换到 map 坐标系.

        AR 坐标系: x=向前(深度), y=向左(水平偏移)
        机器人坐标系: x=向前, y=向左, yaw=逆时针从x轴

        简化模型 (AR 标签靠近相机时 depth 较小, 可近似):
          target_map_x = robot_x + cos(yaw) * ar_dist + sin(yaw) * offset
          target_map_y = robot_y - sin(yaw) * ar_dist + cos(yaw) * offset

        其中 ar_dist 为 AR 标签在前进方向的距离 (约 1-2m),
        offset = ar_x 为水平偏移.
        """
        # AR 的 ar_x 在相机坐标系中是横向偏移 (左右)
        # ar_z (或 ar_y 在 alvar 中) 是深度方向
        # 简化: ar_y 是深度(Z), ar_x 是横向(Y)
        depth = ar_y          # 目标在机器人前方的距离 (~1-2m)
        offset = ar_x         # 目标相对相机中心的横向偏移

        # 计算目标在 map 坐标系的位置
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)

        self.map_x = robot_x + depth * cos_yaw + offset * (-sin_yaw)
        self.map_y = robot_y + depth * sin_yaw + offset * cos_yaw
        self.valid = True
        self.stamp = rospy.Time.now()


# ============================================================================
#  观测缓存 (线程安全)
# ============================================================================

class ObservationCache:
    """AR/视觉观测缓存, 由回调被动更新 (方案三.2)"""

    def __init__(self):
        self.lock = threading.Lock()
        self.data = None
        self.stamp = rospy.Time(0)

    def update(self, **kwargs):
        with self.lock:
            self.data = kwargs
            self.stamp = rospy.Time.now()

    def get(self):
        with self.lock:
            if self.data is None:
                return None
            age = (rospy.Time.now() - self.stamp).to_sec()
            if age > VISION_STALE_SEC:
                return None
            return dict(self.data)

    def clear(self):
        with self.lock:
            self.data = None


# ============================================================================
#  主任务节点
# ============================================================================

class OptimizedMission:
    def __init__(self):
        # ── 状态 ──
        self.state = MissionState.WAIT_START
        self.state_entry_time = rospy.Time.now()

        # ── 机器人当前位姿 (由 /odom 回调更新, 20Hz+) ──
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.pose_lock = threading.Lock()

        # ── 观测缓存 ──
        self.ar_cache = ObservationCache()

        # ── 空间锚定目标 ──
        self.anchored_target = AnchoredTarget()

        # ── 语音解析的目标 ID ──
        self.target_id_rotating = 255
        self.target_id_moving = 255

        # ── 射击状态 ──
        self.shots_fired = {1: 0, 2: 0, 3: 0}
        self.last_shoot_time = rospy.Time(0)

        # ── ROS 发布者 ──
        self.cmd_vel_pub   = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.shoot_pub     = rospy.Publisher("/shoot_cmd", String, queue_size=1)
        self.voice_pub     = rospy.Publisher("/voiceWords", String, queue_size=1)
        self.arrive_pub    = rospy.Publisher("/task/arrive", String, queue_size=1)

        # ── ROS 订阅者 ──
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=1)
        self.ar_sub   = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self._ar_cb, queue_size=1)
        self.rot_sub  = rospy.Subscriber("target_id_rotating", Int32,
                                         self._rotating_id_cb, queue_size=1)
        self.mov_sub  = rospy.Subscriber("target_id_moving", Int32,
                                         self._moving_id_cb, queue_size=1)

        # ── move_base 客户端 ──
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("[MISSION] 等待 move_base 服务...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("[MISSION] 初始化完成, 等待启动指令")

    # ──────────────────────────────────────────
    #  回调: 只更新缓存, 不做阻塞操作
    # ──────────────────────────────────────────

    def _odom_cb(self, msg):
        """EKF 融合里程计回调 (20Hz+)"""
        with self.pose_lock:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            _, _, self.robot_yaw = transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w])

    def _ar_cb(self, msg):
        """AR 标签回调 (~5Hz): 仅更新观测缓存"""
        markers = msg.markers
        if not markers:
            self.ar_cache.clear()
            return

        # 缓存所有检测到的标签
        ar_dict = {}
        for m in markers:
            ar_dict[m.id] = {
                'x': m.pose.pose.position.x,
                'y': m.pose.pose.position.y,
                'z': m.pose.pose.position.z,
            }
        self.ar_cache.update(markers=ar_dict)

    def _rotating_id_cb(self, msg):
        self.target_id_rotating = msg.data
        rospy.loginfo("[MISSION] 语音设定 旋转靶 ID=%d", msg.data)

    def _moving_id_cb(self, msg):
        self.target_id_moving = msg.data
        rospy.loginfo("[MISSION] 语音设定 移动靶 ID=%d", msg.data)

    def _get_pose(self):
        """线程安全获取当前位姿"""
        with self.pose_lock:
            return self.robot_x, self.robot_y, self.robot_yaw

    # ──────────────────────────────────────────
    #  导航
    # ──────────────────────────────────────────

    def goto(self, goal_xyy, timeout=NAV_TIMEOUT_SEC):
        """
        导航到目标点位 (x, y, yaw_deg).
        返回 True 表示成功到达.
        """
        x, y, yaw_deg = goal_xyy
        rospy.loginfo("[NAV] 目标: (%.3f, %.3f, %.1f deg)", x, y, yaw_deg)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0, 0, math.radians(yaw_deg))
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal)
        finished = self.move_base.wait_for_result(rospy.Duration(timeout))

        if not finished:
            self.move_base.cancel_goal()
            rospy.logerr("[NAV] 导航超时: %s", goal_xyy)
            return False

        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("[NAV] 到达: %s", goal_xyy)
            return True
        else:
            rospy.logerr("[NAV] 导航失败, state=%d: %s", state, goal_xyy)
            return False

    # ──────────────────────────────────────────
    #  底盘控制
    # ──────────────────────────────────────────

    def _stop_robot(self, duration=1.0):
        """发送零速度指令并等待停稳"""
        rospy.loginfo("[CTRL] 停车 %.1fs...", duration)
        end = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(CONTROL_RATE)
        zero = Twist()
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(zero)
            rate.sleep()

    def _rotate_to_yaw(self, target_yaw, tolerance=0.03, max_duration=8.0):
        """
        方案二.2: 执行一次旋转动作, 不依赖连续视觉反馈.
        使用当前 EKF 位姿 (20Hz) 做角度闭合控制.
        """
        rospy.loginfo("[CTRL] 旋转到 yaw=%.3f rad (%.1f deg)",
                      target_yaw, math.degrees(target_yaw))

        end_time = rospy.Time.now() + rospy.Duration(max_duration)
        rate = rospy.Rate(CONTROL_RATE)

        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            _, _, current_yaw = self._get_pose()

            # 计算角度误差 (最短路径)
            error = target_yaw - current_yaw
            error = math.atan2(math.sin(error), math.cos(error))

            if abs(error) < tolerance:
                rospy.loginfo("[CTRL] 旋转到位, error=%.4f rad", error)
                break

            # P 控制
            cmd = Twist()
            cmd.angular.z = 1.5 * error  # P gain
            # 限幅
            cmd.angular.z = max(-0.8, min(0.8, cmd.angular.z))
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self._stop_robot(ROTATE_SETTLE_SEC)
        return True

    # ──────────────────────────────────────────
    #  射击 (通过 /shoot_cmd 统一接口)
    # ──────────────────────────────────────────

    def _shoot(self, target_num=0):
        """请求射击, 带冷却保护"""
        now = rospy.Time.now()
        elapsed = (now - self.last_shoot_time).to_sec()
        if elapsed < SHOOT_COOLDOWN_SEC:
            rospy.logwarn("[SHOOT] 冷却中 (%.1fs), 跳过", elapsed)
            return

        rospy.loginfo("[SHOOT] 射击 (目标%d)", target_num)
        self.shoot_pub.publish("shoot_once")
        self.last_shoot_time = now
        self.shots_fired[target_num] += 1

    # ──────────────────────────────────────────
    #  方案二.1: 导航盲射 (1号固定靶)
    # ──────────────────────────────────────────

    def _blind_shoot_static(self, goal):
        """
        方案二.1: 精确导航到达后直接射击, 不使用任何视觉反馈.
        完全移除 find_object_2d 依赖.
        """
        rospy.loginfo("[BLIND] === 方案二.1: 1号固定靶 — 导航盲射 ===")

        # 导航到精确位姿
        if not self.goto(goal):
            return False

        # 停车
        self._stop_robot(STOP_DURATION)

        # 语音播报
        self.voice_pub.publish("到达1号任务点")

        # 直接射击 (不做视觉对准)
        rospy.sleep(0.3)
        self._shoot(target_num=1)
        rospy.sleep(0.2)
        self._shoot(target_num=1)  # 双发确保命中

        rospy.loginfo("[BLIND] 1号靶盲射完成")
        return True

    # ──────────────────────────────────────────
    #  方案二.2 + 方案三.1: Stop-and-Shoot
    # ──────────────────────────────────────────

    def _stop_and_shoot(self, goal, target_id, target_num, yaw_th):
        """
        方案二.2 & 方案三.1: Stop-and-Shoot 单次采样对准.

        流程:
          1. 导航到目标点
          2. 完全停稳
          3. 等待单帧 AR 数据 (rospy.wait_for_message)
          4. 空间锚定: 计算目标在 map 坐标系的绝对位置
          5. 根据锚定位置计算所需朝向
          6. 执行一次旋转动作
          7. 停稳
          8. 射击
        """
        rospy.loginfo("[SNS] === Stop-and-Shoot: 目标%d (AR id=%d) ===",
                      target_num, target_id)

        # Step 1: 导航
        if not self.goto(goal):
            return False

        # Step 2: 停稳
        self._stop_robot(STOP_DURATION)
        self.voice_pub.publish("到达{}号任务点".format(target_num))

        # Step 3: 等待单帧 AR 数据 (阻塞等待, 最多3秒)
        rospy.loginfo("[SNS] 等待 AR 标签单帧数据...")
        try:
            ar_msg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers, timeout=3.0)
        except rospy.ROSException:
            rospy.logerr("[SNS] 等待 AR 数据超时, 使用当前位姿盲射")
            self._shoot(target_num=target_num)
            return True

        # Step 4: 空间锚定 — 找到目标标签并计算 map 坐标
        found = False
        for marker in ar_msg.markers:
            if marker.id == target_id:
                ar_x = marker.pose.pose.position.x   # 横向偏移
                ar_y = marker.pose.pose.position.y   # AR 坐标 y
                ar_z = marker.pose.pose.position.z   # 深度方向

                rx, ry, ryaw = self._get_pose()

                # 方案三.1: 空间锚定 — 计算目标在 map 坐标系的位置
                # ar_z 是目标相对于相机的深度 (前进方向距离)
                # ar_x 是目标相对于相机中心的横向偏移
                self.anchored_target.update(
                    ar_x=ar_x,
                    ar_y=ar_z,   # 使用深度作为 y 分量
                    robot_x=rx,
                    robot_y=ry,
                    robot_yaw=ryaw,
                )

                rospy.loginfo("[SNS] AR标签%d 检测: offset_x=%.3f, depth=%.3f",
                              target_id, ar_x, ar_z)
                rospy.loginfo("[SNS] 锚定 map 坐标: (%.3f, %.3f)",
                              self.anchored_target.map_x,
                              self.anchored_target.map_y)
                found = True
                break

        if not found:
            rospy.logwarn("[SNS] 未检测到目标标签%d, 盲射", target_id)
            self._shoot(target_num=target_num)
            return True

        # Step 5: 根据锚定位置计算目标朝向
        rx, ry, ryaw = self._get_pose()
        target_dx = self.anchored_target.map_x - rx
        target_dy = self.anchored_target.map_y - ry
        target_yaw = math.atan2(target_dy, target_dx)

        rospy.loginfo("[SNS] 目标方向: %.3f rad (%.1f deg), "
                      "当前朝向: %.3f rad",
                      target_yaw, math.degrees(target_yaw), ryaw)

        # Step 6: 执行一次旋转
        self._rotate_to_yaw(target_yaw, tolerance=yaw_th)

        # Step 7: 停稳
        self._stop_robot(0.5)

        # Step 8: 射击
        rospy.sleep(0.2)
        self._shoot(target_num=target_num)
        rospy.sleep(0.15)
        self._shoot(target_num=target_num)

        rospy.loginfo("[SNS] 目标%d Stop-and-Shoot 完成", target_num)
        return True

    # ──────────────────────────────────────────
    #  方案三.2: 控制频率解耦 — 使用锚定目标做 20Hz 对准
    # ──────────────────────────────────────────

    def _aim_to_anchored_target(self, yaw_th, max_duration=15.0):
        """
        方案三.2: 控制频率解耦对准.
        使用 20Hz EKF 位姿 + 5Hz 更新的锚定目标,
        即使视觉没有新帧, 也基于锚定坐标和当前位姿差进行补算.
        """
        rospy.loginfo("[AIM] 解耦对准: 20Hz 控制, 锚定目标驱动")

        end_time = rospy.Time.now() + rospy.Duration(max_duration)
        rate = rospy.Rate(CONTROL_RATE)

        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            # 检查锚定目标是否有效
            if not self.anchored_target.valid:
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
                rate.sleep()
                continue

            # 20Hz: 用当前位姿和锚定目标计算角度误差
            rx, ry, ryaw = self._get_pose()
            target_dx = self.anchored_target.map_x - rx
            target_dy = self.anchored_target.map_y - ry
            target_yaw = math.atan2(target_dy, target_dx)
            error = target_yaw - ryaw
            error = math.atan2(math.sin(error), math.cos(error))

            if abs(error) < yaw_th:
                rospy.loginfo("[AIM] 对准完成, error=%.4f rad", error)
                break

            # 如果视觉有新的观测, 更新锚定目标
            ar_data = self.ar_cache.get()
            if ar_data is not None:
                target_id = self.target_id_rotating  # 简化: 用 rotating ID
                if target_id in ar_data['markers']:
                    m = ar_data['markers'][target_id]
                    self.anchored_target.update(
                        ar_x=m['x'], ar_y=m['z'],
                        robot_x=rx, robot_y=ry, robot_yaw=ryaw,
                    )

            # P 控制
            cmd = Twist()
            cmd.angular.z = 1.8 * error
            cmd.angular.z = max(-0.6, min(0.6, cmd.angular.z))
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self._stop_robot(0.3)
        return True

    # ──────────────────────────────────────────
    #  主状态循环 (20Hz 控制)
    # ──────────────────────────────────────────

    def run(self, goals):
        """
        主任务循环.

        goals = [[x0, y0, yaw0],   # 1号任务点 (固定靶, 盲射)
                 [x1, y1, yaw1],   # 2号任务点 (旋转靶, Stop-and-Shoot)
                 [x2, y2, yaw2],   # 3号任务点 (移动靶, Stop-and-Shoot)
                 [x3, y3, yaw3]]   # 终点
        """

        rate = rospy.Rate(CONTROL_RATE)
        rospy.loginfo("[MISSION] 进入主循环 (20Hz), 状态: %s", self.state)

        while not rospy.is_shutdown():

            # ── 状态: 等待启动 ──
            if self.state == MissionState.WAIT_START:
                if rospy.get_param('/start', False):
                    rospy.loginfo("[MISSION] 收到启动信号!")
                    self.state = MissionState.NAV_P1
                rate.sleep()
                continue

            # ── 状态: 导航到任务点1 + 盲射 ──
            elif self.state == MissionState.NAV_P1:
                rospy.loginfo("[MISSION] === 第1阶段: 1号固定靶 (盲射) ===")
                if self._blind_shoot_static(goals[0]):
                    self.state = MissionState.NAV_P2
                else:
                    rospy.logerr("[MISSION] 1号靶任务失败")
                    self.state = MissionState.NAV_P2  # 不中断, 继续下一个

            # ── 状态: 导航到任务点2 + Stop-and-Shoot ──
            elif self.state == MissionState.NAV_P2:
                rospy.loginfo("[MISSION] === 第2阶段: 2号旋转靶 (Stop-and-Shoot) ===")
                if self._stop_and_shoot(goals[1], self.target_id_rotating,
                                        target_num=2, yaw_th=YAW_TH_ROTATING):
                    self.state = MissionState.NAV_P3
                else:
                    rospy.logerr("[MISSION] 2号靶任务失败")
                    self.state = MissionState.NAV_P3

            # ── 状态: 导航到任务点3 + Stop-and-Shoot ──
            elif self.state == MissionState.NAV_P3:
                rospy.loginfo("[MISSION] === 第3阶段: 3号移动靶 (Stop-and-Shoot) ===")
                if self._stop_and_shoot(goals[2], self.target_id_moving,
                                        target_num=3, yaw_th=YAW_TH_MOVING):
                    self.state = MissionState.NAV_END
                else:
                    rospy.logerr("[MISSION] 3号靶任务失败")
                    self.state = MissionState.NAV_END

            # ── 状态: 导航到终点 ──
            elif self.state == MissionState.NAV_END:
                rospy.loginfo("[MISSION] === 导航至终点 ===")
                if len(goals) > 3:
                    if self.goto(goals[3]):
                        self.voice_pub.publish("任务完成")
                self.state = MissionState.FINISHED

            # ── 状态: 完成 ──
            elif self.state == MissionState.FINISHED:
                rospy.loginfo("[MISSION] ====== 全部任务完成! ======")
                self.voice_pub.publish("全部任务完成")
                self._stop_robot(0.5)
                break

            # ── 状态: 错误 ──
            elif self.state == MissionState.ERROR:
                rospy.logerr("[MISSION] 错误状态, 终止任务")
                break

            rate.sleep()

        rospy.loginfo("[MISSION] 节点退出")


# ============================================================================
#  主入口
# ============================================================================

if __name__ == "__main__":
    rospy.init_node('navigation_optimized', anonymous=True)

    # 读取导航目标参数
    goalX  = rospy.get_param('~goalListX',   '1.24, 1.062, 0.945, 0.28')
    goalY  = rospy.get_param('~goalListY',   '-0.473, -1.695, -2.976, -2.98')
    goalYaw = rospy.get_param('~goalListYaw', '0.0, 0.0, 0.0, 0.0')

    goals = [[float(x), float(y), float(yaw)]
             for (x, y, yaw) in zip(goalX.split(","),
                                     goalY.split(","),
                                     goalYaw.split(","))]
    rospy.loginfo("[MISSION] 导航目标点: %s", goals)

    rospy.sleep(0.5)

    # 创建任务节点
    mission = OptimizedMission()

    # 等待用户启动指令
    rospy.loginfo("请在另一个终端执行:  rostopic pub /start std_msgs/Bool \"data: true\"")
    print("")
    print("=" * 60)
    print("  请输入 1 启动比赛: ")
    print("=" * 60)

    try:
        user_input = raw_input()
    except NameError:
        user_input = input()

    if user_input == '1':
        rospy.set_param('/start', True)
        rospy.loginfo("[MISSION] 比赛开始!")

        # 播放启动语音
        rospy.sleep(1)
        pub = rospy.Publisher('audio_topic', String, queue_size=1)
        for _ in range(2):
            pub.publish("audio message")
            rospy.sleep(0.5)

        # 等待 TTS 播放完成
        rospy.sleep(25)

        # 运行任务
        mission.run(goals)
    else:
        rospy.loginfo("[MISSION] 已取消")

    # 保持节点存活
    while not rospy.is_shutdown():
        rospy.sleep(0.5)
