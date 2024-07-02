#!/usr/bin/env python3
import rospy
import numpy as np

from sensor_msgs.msg import Imu

class IMUConversion(object):
    '''
    Converting signals in IMU acceleration and angular velocity 
    ''' 
    def __init__(self):
        self._imu_sub = rospy.Subscriber("/controller/imu", Imu, self.receiveIMU)
        self._imu_pub = rospy.Publisher("/terrasentia/imu", Imu, queue_size=1)

    def receiveIMU(self, msg):
        imu_msg = Imu()
        imu_msg.header = msg.header

        imu_msg.header.frame_id = "imu_link"
        # imu_msg.header.stamp = rospy.Time.now()

        # Changing the signal of the IMU
        imu_msg.orientation = msg.orientation

        imu_msg.angular_velocity.x = msg.angular_velocity.x
        imu_msg.angular_velocity.y = msg.angular_velocity.y
        imu_msg.angular_velocity.z = msg.angular_velocity.z

        imu_msg.linear_acceleration.x = -msg.linear_acceleration.x #* 0.981
        imu_msg.linear_acceleration.y = -msg.linear_acceleration.y #* 0.981
        imu_msg.linear_acceleration.z = -msg.linear_acceleration.z #* 0.981
        
        self._imu_pub.publish(imu_msg)

if __name__ == "__main__":
    rospy.init_node("imu_conversion")
    imu_conversion = IMUConversion()
    rospy.spin()
