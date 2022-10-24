#!/usr/bin/env python

import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import Odometry

global_move_base = None
# Move base using navigation stack
class MoveBaseClient(object):
    destination = []
    current_pos = None
    moving = False
    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        self.destination = [x,y,theta]
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.oving = True
        self.client.wait_for_result()

    def cancel_goals(self):
        distance = [999999,9999999,9999999]
        self.client.cancel_all_goals()
        self.moving = False

    def distanceCalc(self,msg):
        if self.moving:
            current_pos = msg.pose.pose.position
            if(abs(self.destination[0] - msg.pose.pose.position.x) < 0.1) or (abs(self.destination[1] - msg.pose.pose.position.y) < 0.1):
                self.cancel_goals()

if __name__ == "__main__":
    # Create a node
    rospy.init_node("snack_controller")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    move_base = MoveBaseClient()
    global_move_base = move_base
    # Move the base to be in front of the table
    # Demonstrates the use of the navigation stack
    rospy.loginfo("Moving to snack dispenser...")
    rospy.Subscriber('odom',Odometry,move_base.distanceCalc)
    move_base.goto(4.03, -3, 0.0)

    # Move to second table
    rospy.loginfo("Moving to table...")
    move_base.goto(-3.53, 3.75, 0.0)
   

    rospy.loginfo("comp..")

