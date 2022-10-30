#!/usr/bin/env python
# license removed for brevity

import rospy

from std_msgs.msg import String

class MessagePublisherNode:
    def __init__(self):
        self.count = 0
        rospy.loginfo("Starting: message publisher node")
        self.pub_msg = rospy.Publisher('msg_tx', String, queue_size=10)
        rospy.Timer(rospy.Duration(0.5), self.timer_update)

    def timer_update(self, event):
        hello_str = "hello world %s" % self.count
        self.pub_msg.publish(hello_str)
        self.count+=1

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    try :
        rospy.init_node('message_publisher_node')
        node = MessagePublisherNode()
        node.main()
    except rospy.ROSInterruptException:
        pass