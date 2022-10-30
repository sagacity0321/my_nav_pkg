#!/usr/bin/env python

import sys
import rospy
import actionlib

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from darknet_ros_msgs.msg import BoundingBoxes 

class NavCommanderNode:
    def __init__(self):
        rospy.init_node("nav_commander")
        self.pub_init_pose = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.header.frame_id = "map"
        self.init_pose.header.stamp = rospy.Time.now()
        self.init_pose.pose.pose.position.x = 0.0
        self.init_pose.pose.pose.position.y = 0.0
        self.init_pose.pose.pose.orientation.w = 1.0

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"

        self.update_init_pose(0.0, 0.0, 0.0)
        self.pub_init_pose.publish(self.init_pose)
        self.send_goal(2.5,-0.5,0.0)
        wait = self.client.wait_for_result()
        if not wait:
            print('Error')
        else:
            print(self.client.get_result())

        self.send_goal(0.0,0.0,0.0)
        wait = self.client.wait_for_result()
        if not wait:
            print('Error')
        else:
            print(self.client.get_result())

    
    def update_init_pose(self, x, y, theta):
        self.init_pose.header.stamp = rospy.Time.now()
        self.init_pose.pose.pose.position.x = x
        self.init_pose.pose.pose.position.y = y
        self.init_pose.pose.pose.orientation.w = 1.0
        q = quaternion_from_euler(0.0, 0.0, theta)
        self.init_pose.pose.pose.orientation.x = q[0]
        self.init_pose.pose.pose.orientation.y = q[1]
        self.init_pose.pose.pose.orientation.z = q[2]
        self.init_pose.pose.pose.orientation.w = q[3]

    def send_goal(self,x,y,theta):
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0.0, 0.0, theta)
        self.goal.target_pose.pose.orientation.x=q[0]
        self.goal.target_pose.pose.orientation.y=q[1]
        self.goal.target_pose.pose.orientation.z=q[2]
        self.goal.target_pose.pose.orientation.w=q[3]
        self.client.send_goal(self.goal)
    
    def new_bb(self, bb_msg):
        for box in bb_msg.bounding_boxes:
            if box.Class == 'clock':
                self.x = (box.xmin + box.xmax)/2
                self.y = (box.ymin + box.ymax)/2
                self.vx = 0.0
                self.vw = 0.0
                if self.x < 600: # Image is in the left
                    self.vx = 0.0
                    self.vw = 0.1 #To the left
                elif self.x > 800: # image is in the right
                    self.vx = 0.0
                    self.vw = -0.1 # To the right
                else:
                    self.vx = 0.1
                    self.vw = 0.0
                self.twist = self.Twist()
                self.twist.linear.x = self.vx
                self.twist.angular.z = self.vw
                self.pub_cmd.publish(self.wist)
            


    def main(self):
        rospy.spin()


if __name__== '__main__':
    try:
        rospy.init_node('nav_commander')
        node = NavCommanderNode()
        node.main()
    except rospy.ROSInterruptException:
        pass