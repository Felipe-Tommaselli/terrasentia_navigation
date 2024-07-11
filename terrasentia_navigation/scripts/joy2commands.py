#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist

class JoyToCmdVel:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('joy_to_cmd_vel')
        
        # Parameters for joystick axes and buttons
        self.scale_linear = rospy.get_param("~scale_linear", 1.0)
        self.scale_angular = rospy.get_param("~scale_angular", 1.0)
        
        # Publisher for cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscriber for joystick input
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Service client for toggling recording
        rospy.wait_for_service('toggle_recording')
        self.toggle_recording_service = rospy.ServiceProxy('toggle_recording', Trigger)
        
        rospy.loginfo("JoyToCmdVel node initialized")

    def joy_callback(self, joy):
        twist = Twist()
        twist.linear.x = self.scale_linear * joy.axes[1]
        twist.angular.z = self.scale_angular * joy.axes[2]
        self.cmd_vel_pub.publish(twist)

        # Check if the specified buttons are pressed
        current_button_state = (joy.buttons[self.toggle_button1], joy.buttons[self.toggle_button2])
        if all(current_button_state) and not all(self.previous_button_state):
            # Both buttons pressed simultaneously, call the service
            rospy.loginfo("Toggling recording via service call")
            try:
                response = self.toggle_recording_service()
                if response.success:
                    rospy.loginfo("Successfully toggled recording")
                else:
                    rospy.logwarn("Failed to toggle recording")
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)

        # Update the previous button state
        self.previous_button_state = current_button_state

if __name__ == '__main__':
    try:
        JoyToCmdVel()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
