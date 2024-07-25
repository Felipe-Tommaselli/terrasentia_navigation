#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8, String
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped

class JoyToCmdVel:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('joy_to_cmd_vel')
        
        # Parameters for joystick axes and buttons
        self.scale_linear = rospy.get_param("~scale_linear", 1.0)
        self.scale_angular = rospy.get_param("~scale_angular", 1.0)
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', TwistStamped, queue_size=10)
        self.auto_mode_pub = rospy.Publisher('enable_auto_mode', Int8, queue_size=10)
        self.pub_setString = rospy.Publisher('/msg_setString', String, queue_size=1)
        
        # Subscriber for joystick input
        rospy.Subscriber('joy', Joy, self.joy_callback)
        rospy.Subscriber('cmd_vel', TwistStamped, self.cmd_callback)

        # Service client for toggling recording
        rospy.wait_for_service('toggle_recording')
        self.toggle_recording_service = rospy.ServiceProxy('toggle_recording', Trigger)
        
        rospy.loginfo("JoyToCmdVel node initialized")
        self.auto_mode = False

    def cmd_callback(self, cmd_msg):
        # Publish the control commands to the controller
        if self.auto_mode:
            msg = {
                "name": "setNormalizedVehicleMotion_nav",
                "args": {
                    "speed": cmd_msg.twist.linear.x,
                    "turnRate": -cmd_msg.twist.angular.z
                }
            }
        else:
            msg = {
                "name": "setNormalizedVehicleMotion_nav",
                "args": {
                    "speed": 0.0,
                    "turnRate": 0.0
                }
            }

        self.sendMessage(msg, False)

    def joy_callback(self, joy):
        # Publish the joystick commands to the cmd_vel topic
        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = "base_link"
        twist.twist.linear.x = self.scale_linear * joy.axes[1]
        twist.twist.angular.z = self.scale_angular * joy.axes[2]
        self.cmd_vel_pub.publish(twist)

        # Check if L1 + R1 buttons are pressed
        current_button_state = (joy.buttons[6], joy.buttons[7])
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

        # Check if the combination L2 + X is pressed
        if joy.buttons[8] and joy.buttons[3]:
            # Publish the new message
            msg = {
                "name": "startAutoMode"
            }
            self.sendMessage(msg)
            rospy.loginfo("Autonomy status started message sent")
            self.auto_mode = True

        # Check if the combination L2 + Y is pressed
        elif joy.buttons[8] and joy.buttons[4]:
            # Publish the new message
            msg = {
                "name": "stoptAutoMode"
            }
            self.sendMessage(msg)
            rospy.loginfo("Autonomy status stopped message sent")
            self.auto_mode = False

        # Update the previous button state
        self.previous_button_state = current_button_state

    def sendMessage(self, msg, debug=True):
        msg = str(msg).replace("'", '"')
        try:
            req = String()
            req.data = msg
            self.pub_setString.publish(req)
        except Exception as e:
            rospy.logerr('failed to send to controller the msg ' + str(msg))
            print(str(e))
            raise(e)

if __name__ == '__main__':
    try:
        JoyToCmdVel()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
