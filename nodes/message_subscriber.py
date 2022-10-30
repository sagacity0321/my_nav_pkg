#!/usr/bin/env python
# license removed for brevity

import rospy

from std_msgs.msg import String

class MessageSubscriberNode:
    def __init__(self):
        self.msg_count = 0
        rospy.loginfo("Starting: message subscriber node")
        self.pub_stb = rospy.Subscriber('msg_tx', String, self.cb_subMsg, queue_size=10)
    
    def cb_subMsg(self, msg):
        rospy.loginfo("Recevied: %s"%msg.data)
        self.msg_count+=1

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    try :
        rospy.init_node('message_subscriber_node')
        node = MessageSubscriberNode()
        node.main()
    except rospy.ROSInterruptException:
        pass