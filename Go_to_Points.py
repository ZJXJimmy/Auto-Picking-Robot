#!/usr/bin/env python

# TurtleBot must run roslaunch turtlebot_bringup minimal.launch and then run
# roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot/map_2.yaml and last run
# python Go_to_Points.py
import rospy
import time
import serial
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion


class GoToPose():
    def __init__(self):
        self.goal_sent = False
		rospy.on_shutdown(self.shutdown)
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Wait for the action server to come up")
		self.move_base.wait_for_server(rospy.Duration(5))
		self.ser = serial.Serial(port='/dev/ttyACM0', baudrate=9600) 

    def goto(self, pos, quat):
        # Send a goal
        self.goal_sent = True
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
		# Start moving
        self.move_base.send_goal(goal)
		# Allow TurtleBot up to 60 seconds to complete task
		success = self.move_base.wait_for_result(rospy.Duration(60)) 
        state = self.move_base.get_state()
        result = False
        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()
        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()
		######################Go to Pose1######################
		position = {'x': 0.00, 'y' : 0.00}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        rospy.loginfo("Go to (%s, %s) pose1", position['x'], position['y'])
        success = navigator.goto(position, quaternion)
        if success:
	    	rospy.loginfo("Reached the pose1")
        else:
            rospy.loginfo("The base failed to reach the desired pose1")
		rospy.sleep(10)
		######################Go to Pose2######################
		position = {'x': 2.00, 'y' : 0.00}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        rospy.loginfo("Go to (%s, %s) pose2", position['x'], position['y'])
        success = navigator.goto(position, quaternion)
        if success:
		    rospy.loginfo("Reached the pose2")
		    navigator.ser.write('Q')
		    time.sleep(1)
		    navigator.ser.write('Q')
		    time.sleep(1)
        else:
            rospy.loginfo("The base failed to reach the desired pose2")
		######################Go to Pose3######################
		position = {'x': 0.00, 'y' : 0.00}
	    #quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
		quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 1.00000000e+00, 'r4' : 6.12323400e-17}
        rospy.loginfo("Go to (%s, %s) pose3", position['x'], position['y'])
        success = navigator.goto(position, quaternion)
        if success:
		    rospy.loginfo("Reached the pose3")
		    navigator.ser.write('H')
		    time.sleep(1)
		    navigator.ser.write('H')
		    time.sleep(1)
        else:
            rospy.loginfo("The base failed to reach the desired pose3")
		rospy.sleep(5)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

