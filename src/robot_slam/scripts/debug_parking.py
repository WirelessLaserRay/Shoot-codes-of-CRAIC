#!/usr/bin/env python2
#coding: utf-8

import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from tf_conversions import transformations
from math import pi
import sys
import os

# Force UTF-8 encoding in Python 2
reload(sys)
sys.setdefaultencoding('utf-8')

class DebugParkingDemo:
    def __init__(self):
        rospy.loginfo("Initializing DebugParkingDemo...")
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        # Subscribe to AMCL pose to display current robot location relative to target
        self.current_pose = None
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move_base server.")

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def goto(self, p):
        rospy.loginfo("Sending goal: X=%.3f, Y=%.3f, Yaw=%.3f degrees", p[0], p[1], p[2])
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        
        # Convert Yaw degrees to Quaternion orientation
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
        self.move_base.send_goal(goal)
        
        # Wait for the result, allowing Ctrl+C to cancel
        while not rospy.is_shutdown():
            finished = self.move_base.wait_for_result(rospy.Duration(0.5))
            if finished:
                break
                
        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Successfully reached goal!")
            return True
        else:
            rospy.logwarn("Failed to reach goal or cancelled. Status: %d", state)
            return False

    def stop_robot(self):
        self.pub_vel.publish(Twist())

    def end_open_loop(self, linear_x=-0.25, angular_z=0.0, duration=2.5):
        """Execute open-loop command to force entry into the end zone."""
        rospy.loginfo("Executing open-loop end sequence: linear.x = %.3f, angular.z = %.3f, duration = %.1f s", 
                      linear_x, angular_z, duration)
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        
        # Publish velocity at 10Hz
        rate = rospy.Rate(10)
        steps = int(duration * 10)
        for i in range(steps):
            if rospy.is_shutdown():
                break
            self.pub_vel.publish(msg)
            rate.sleep()
            
        self.stop_robot()
        rospy.loginfo("Open-loop sequence completed and robot stopped.")

    def print_pose_comparison(self, target):
        if self.current_pose is None:
            rospy.logwarn("No AMCL pose received yet. Cannot compare current pose.")
            return
            
        pos = self.current_pose.position
        ori = self.current_pose.orientation
        euler = transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        yaw_deg = euler[2] * 180.0 / pi
        
        # Normalize yaw differences to [-180, 180]
        yaw_diff = yaw_deg - target[2]
        while yaw_diff > 180.0: yaw_diff -= 360.0
        while yaw_diff < -180.0: yaw_diff += 360.0

        print("\n=================== POSITION ANALYSIS ===================")
        print("  TARGET GOAL  : X = %7.3f, Y = %7.3f, Yaw = %7.2f deg" % (target[0], target[1], target[2]))
        print("  ROBOT POSE   : X = %7.3f, Y = %7.3f, Yaw = %7.2f deg" % (pos.x, pos.y, yaw_deg))
        print("  DIFFERENCE   : dX = %6.3f, dY = %6.3f, dYaw = %6.2f deg" % (pos.x - target[0], pos.y - target[1], yaw_diff))
        print("=========================================================")

if __name__ == "__main__":
    rospy.init_node('debug_parking', anonymous=True)

    # Load waypoints (default to the current ones inside multi_goal.launch)
    goalListX = rospy.get_param('~goalListX', '1.24, 1.062, 0.945, 0.28')
    goalListY = rospy.get_param('~goalListY', '-0.473, -1.695, -2.976, -2.98')
    goalListYaw = rospy.get_param('~goalListYaw', '0.0, 0.0, 0.0, 0.0')
    
    # Open-loop parameters
    open_loop_linear_x = rospy.get_param('~open_loop_linear_x', -0.25)
    open_loop_angular_z = rospy.get_param('~open_loop_angular_z', 0.0)
    open_loop_duration = rospy.get_param('~open_loop_duration', 2.5)

    # Parse list of goals
    try:
        goals = [[float(x.strip()), float(y.strip()), float(yaw.strip())] 
                 for (x, y, yaw) in zip(goalListX.split(","), goalListY.split(","), goalListYaw.split(","))]
    except ValueError as e:
        rospy.logerr("Error parsing goals: %s. Check coordinate parameters format.", str(e))
        sys.exit(1)

    print("\n=========================================================")
    print("             WAYPOINT DEBUGGING MODE LOADED")
    print("=========================================================")
    print("Total Waypoints Parsed: %d" % len(goals))
    for idx, g in enumerate(goals):
        print("  Waypoint [%d]: X = %6.3f, Y = %6.3f, Yaw = %6.1f deg" % (idx, g[0], g[1], g[2]))
    print("=========================================================\n")

    navi = DebugParkingDemo()
    current_index = 0
    num_goals = len(goals)
    
    # Wait for starting command
    while not rospy.is_shutdown():
        print("Please input '1' or press Enter to start debugging from Waypoint 0: ")
        start_input = raw_input().strip()
        if start_input in ['', '1']:
            break

    while not rospy.is_shutdown():
        target = goals[current_index]
        print("\n---------------------------------------------------------")
        print("  CURRENT TARGET: Waypoint [%d] of %d" % (current_index, num_goals - 1))
        print("  Target coordinates: X = %.3f, Y = %.3f, Yaw = %.1f deg" % (target[0], target[1], target[2]))
        print("---------------------------------------------------------")
        print("  Available Commands:")
        print("    [Enter] or 'g' - Navigate to this Waypoint")
        print("    'n'          - Select NEXT Waypoint")
        print("    'p'          - Select PREVIOUS Waypoint")
        print("    'r'          - Retry current Waypoint")
        print("    's'          - Stop robot (cmd_vel = 0)")
        print("    'e'          - Run open-loop end/backup sequence")
        print("    [0-%d]        - Jump directly to Waypoint index" % (num_goals - 1))
        print("    'q'          - Quit script")
        print("---------------------------------------------------------")
        
        try:
            cmd = raw_input("Command >> ").strip().lower()
        except KeyboardInterrupt:
            rospy.loginfo("KeyboardInterrupt received. Stopping robot and exiting.")
            navi.stop_robot()
            break

        if cmd == 'q':
            rospy.loginfo("Quitting. Stopping robot.")
            navi.stop_robot()
            break
        elif cmd == 's':
            rospy.loginfo("Stopping robot...")
            navi.stop_robot()
        elif cmd == 'e':
            rospy.loginfo("Triggering open-loop end sequence...")
            navi.end_open_loop(linear_x=open_loop_linear_x, 
                               angular_z=open_loop_angular_z, 
                               duration=open_loop_duration)
        elif cmd == 'n':
            if current_index + 1 < num_goals:
                current_index += 1
                rospy.loginfo("Selected NEXT Waypoint index %d", current_index)
            else:
                rospy.logwarn("Already at the last Waypoint index (%d)!", current_index)
        elif cmd == 'p':
            if current_index - 1 >= 0:
                current_index -= 1
                rospy.loginfo("Selected PREVIOUS Waypoint index %d", current_index)
            else:
                rospy.logwarn("Already at the first Waypoint index (%d)!", current_index)
        elif cmd in ['', 'g', 'r']:
            rospy.loginfo("Starting navigation to Waypoint %d...", current_index)
            success = navi.goto(target)
            navi.stop_robot()
            
            # Print physical analysis of pose after arriving/failing
            rospy.sleep(0.5)  # Wait for pose update to stabilize
            navi.print_pose_comparison(target)
            
            if success:
                print("\n>>> Robot arrived at Waypoint %d! <<<" % current_index)
                # Auto-advance selection to next waypoint to make workflow faster
                if current_index + 1 < num_goals:
                    current_index += 1
                    rospy.loginfo("Auto-selected NEXT Waypoint index %d. Press Enter to navigate.", current_index)
                else:
                    # Arrived at the final waypoint, prompt if they want to run the end sequence
                    print("You have reached the final waypoint!")
                    print("Press 'e' to execute the open-loop end sequence, or press Enter to finish.")
            else:
                print("\n>>> Navigation to Waypoint %d failed, cancelled, or timed out. <<<" % current_index)
        else:
            # Check if user entered a specific index number
            try:
                val = int(cmd)
                if 0 <= val < num_goals:
                    current_index = val
                    rospy.loginfo("Selected Waypoint index %d", current_index)
                else:
                    rospy.logerr("Invalid index %d. Must be between 0 and %d.", val, num_goals - 1)
            except ValueError:
                rospy.logwarn("Unknown command: %s", cmd)

