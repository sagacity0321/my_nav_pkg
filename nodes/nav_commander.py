#!/usr/bin/env python

import rospy
import actionlib

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from darknet_ros_msgs.msg import BoundingBoxes 

# class define
class NavCommanderNode:
    
    # NavCommanderNode Constructor
    def __init__(self):
        rospy.init_node("nav_commander")
        self.pub_init_pose = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=10)
        # self.sub_cmd = rospy.Subscriber('my_nav_pkg/msg', BoundingBoxes, self.new_bb, queue_size=10)
        # self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
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

        # Initialize this pose
        self.update_init_pose(0.0, 0.0, 0.0)
        self.pub_init_pose.publish(self.init_pose)

        # To the first goal
        self.send_goal(1.3,3.5,0.0)
        wait = self.client.wait_for_result()
        if not wait:
            print('Error')
        else:
            print(self.client.get_result())

        # # Check the something in the first goal
        # self.pub_cmd.publish(self.new_bb)

        # To the second goal
        self.send_goal(1.8,2.9,0.0)
        wait = self.client.wait_for_result()
        if not wait:
            print('Error')
        else:
            print(self.client.get_result())

        # # Check the someting in the second goal
        # self.pub_cmd.publish(self.new_bb)

        # Return to the init pose
        self.send_goal(0.0,0.0,0.0)
        wait = self.client.wait_for_result()
        if not wait:
            print('Error')
        else:
            print(self.client.get_result())

    # Define init pose
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

    # Define goal for moving
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
    
    # # Define identify something
    # def new_bb(self, bb_msg):
    #     for box in bb_msg.bounding_boxes:
    #         if box.Class == 'bottle':
    #             self.x = (box.xmin + box.xmax)/2
    #             self.y = (box.ymin + box.ymax)/2
    #             self.vx = 0.0
    #             self.vw = 0.0
    #             if self.x < 600: # Image is in the left
    #                 self.vx = 0.0
    #                 self.vw = 0.1 #To the left
    #             elif self.x > 800: # image is in the right
    #                 self.vx = 0.0
    #                 self.vw = -0.1 # To the right
    #             else:
    #                 self.vx = 0.1
    #                 self.vw = 0.0
    #             self.twist = Twist()
    #             self.twist.linear.x = self.vx
    #             self.twist.angular.z = self.vw
    #             self.pub_cmd.publish(self.twist)


    # Define main
    def main(self):
        # Repeat
        rospy.spin()

# Define main
if __name__== '__main__':
    try:
        rospy.init_node('nav_commander')
        node = NavCommanderNode() # Create NavcommanderNode for node
        node.main() # Excution main
    
    # Caused rospy error
    except rospy.ROSInterruptException:
        pass