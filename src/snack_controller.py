#!/usr/bin/env python

import copy
import actionlib
import rospy

import socket
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


class SnackBowl(object):
    def __init__(self):
        self.max_volume = 1
        self.current_volume = 0
        self.wasted = 0

    def add_snack(self, volume):
        rospy.loginfo(f"Added {volume} snack volume!")
        self.current_volume += volume
        if self.current_volume > self.max_volume: #overflow!
            self.wasted += self.current_volume - self.max_volume
            self.current_volume = self.max_volume
            rospy.loginfo(f"We have now wasted {self.wasted} snack volume :(")
    def full(self):
        return self.max_volume <= self.current_volume

class SnackDispenser:
    #creates a snack dispenser that drops into snack_bowl, that drops rate units/sec of snacks into it
    #there is a period of delay seconds where the snack is in the air: it will only increase the number of snacks in the bowl after delay seconds. 
    def __init__(self, snack_bowl: SnackBowl, delay: float, rate: float):
        self.snack_bowl = snack_bowl
        self.delay = delay
        self.rate = rate
        self.check_interval = 0.05
        self.dispense_time_elapsed = 0
        self.start_dispensing()
        self.dispensing = True
        rospy.Timer(rospy.Duration(self.check_interval), self.try_dispense) #check often if we are under a snack dispenser
        
    def robot_under(self):
        return self.dispensing

    def start_dispensing(self):
        self.dispense_time_elapsed = 0
        self.dispensing = True

    def stop_dispensing(self):
        self.dispensing = False

    def update_dispensing(self):
        if self.snack_bowl.full():
            self.stop_dispensing()

    def try_dispense(self, event):
        if self.dispensing:
            self.dispense_time_elapsed += self.check_interval
            self.update_dispensing()
            self.drop_snack(self.rate*self.check_interval)
    def drop_snack(self, volume):
        rospy.Timer(rospy.Duration(self.delay), lambda event: self.snack_bowl.add_snack(volume), oneshot=True)

# Move base using navigation stack
class MoveBaseClient(object):
    destination = []
    current_pos = None
    moving = False
    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.moving = False
        self.odom = None
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
        self.moving = True
        self.client.wait_for_result()
    
    def under_snack_dispenser(self):
        return False

    def cancel_goals(self):
        rospy.loginfo("cancelling!!!!")
        self.client.cancel_all_goals()
        self.moving = False

    def get_odom(self, msg):
        self.odom = msg
    def distanceCalc(self,event):
        if self.moving and self.odom != None:
            #rospy.loginfo(f"odom: ({self.odom.pose.pose.position.x},{self.odom.pose.pose.position.y}) x: {abs(self.destination[0] - self.odom.pose.pose.position.x)}, y: {abs(self.destination[1] - self.odom.pose.pose.position.y)}")
            
            if(abs(self.destination[0] - self.odom.pose.pose.position.x) < 0.1) or (abs(self.destination[1] - self.odom.pose.pose.position.y) < 0.1):
                self.cancel_goals()



if __name__ == "__main__":
    # Create a node
    rospy.init_node("snack_controller")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass
    snack_bowl = SnackBowl()
    snack_bowl.max_volume = 1
    snacK_dispenser = SnackDispenser(snack_bowl, 0.5, 0.5)
    snacK_dispenser.start_dispensing()
    HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
    PORT = 65432 #Port to listen on (non-privileged ports are > 1023)
    
    move_base = MoveBaseClient()
    rospy.Subscriber('odom',Odometry,move_base.get_odom)
    rospy.Timer(rospy.Duration(0.1), move_base.distanceCalc)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Ready on {HOST}:{PORT}")
        while True:
            conn, addr = (None, None)
            try:
                conn, addr = s.accept()
            except:
                continue #retry on timeout
            with conn:
                print(f"Connected by {addr}")
                data = conn.recv(1024)
                if not data:
                    break
                try:
                    coords = data.decode('utf-8')
                    x = float(coords.split(',')[0])
                    y = float(coords.split(',')[1])
                    print(f"moving to {x}, {y}")
                    move_base.goto(x,y,0)
                    print("Done!")
                except Exception as e:
                    print("Error occurred while trying to move: " + str(e))
"""
    # Setup clients
    move_base = MoveBaseClient()
    # Move the base to be in front of the table
    # Demonstrates the use of the navigation stack
    rospy.loginfo("Moving to snack dispenser...")
    rospy.Subscriber('odom',Odometry,move_base.get_odom)
    rospy.Timer(rospy.Duration(0.1), move_base.distanceCalc)
    #move_base.goto(4.03, -3, 0.0)
    move_base.goto(2.250, 3.118, 0.0)

    # Move to second table
    rospy.loginfo("Moving to table...")
    #move_base.goto(-3.53, 3.75, 0.0)
   

    rospy.loginfo("comp..")
    """

