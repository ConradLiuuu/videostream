#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import UInt8

class Listener:
    def __init__(self):
        self.sub = rospy.Subscriber("/chatter", String, self.callback)

    def callback(sef, data):
        rospy.loginfo(" I heard %s", data.data)
'''
if __name__ == '__main__':
    rospy.init_node('listener', anonymous = True)
    Listener()
    rospy.spin()

'''
def callback_left(data):
    rospy.loginfo("left count %d", data.data)
def callback_right(data):
    rospy.loginfo("right count %d", data.data)
def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber("count_left", UInt8, callback_left)
    rospy.Subscriber("count_right", UInt8, callback_right)
    rospy.spin()
if __name__ == '__main__':
    listener()

