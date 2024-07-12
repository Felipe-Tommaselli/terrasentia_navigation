#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

class JoyToCmdVel:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('joy_to_cmd_vel')
        
        # Parameters for joystick axes and buttons
        self.scale_linear = rospy.get_param("~scale_linear", 1.0)
        self.scale_angular = rospy.get_param("~scale_angular", 1.0)
        
        # Publisher for cmd_vel
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', TwistStamped, queue_size=10)
        
        # Subscriber for joystick input
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        
        rospy.loginfo("JoyToCmdVel node initialized")

    def joy_callback(self, joy):
        twist = TwistStamped()
        twist.twist.linear.x = self.scale_linear * joy.axes[1]
        twist.twist.angular.z = self.scale_angular * joy.axes[2]
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        JoyToCmdVel()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass