#!/usr/bin/env python3
# This communicates with terra-controller and transforms events into ROS messages

import time

from geometry_msgs.msg import Twist, TwistStamped
import os
import signal, sys
import rospy
import numpy as np


class MotionCmdNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('motion_cmd_node')
        
        # Publishers
        self.pub_motion_cmd = rospy.Publisher('motion_command', TwistStamped, queue_size=1)
        
        # Subscriber for joystick input
        rospy.Subscriber('/controller/motion_command', Twist, self.motionCmdCb)

        
        rospy.loginfo("MotionCmdNode node initialized")

    # PUBLISHERS TO NAV STACK 
    def motionCmdCb(self, msg):
        motion_msg = TwistStamped()
        motion_msg.header.stamp = rospy.Time.now()
        motion_msg.header.frame_id = "base_link"
        motion_msg.twist = msg
        motion_msg.twist.angular.z = -msg.angular.z

        self.pub_motion_cmd.publish(motion_msg)

if __name__ == '__main__':
    try:
        MotionCmdNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
