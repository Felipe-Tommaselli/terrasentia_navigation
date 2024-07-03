#!/usr/bin/env python3
import rospy
import numpy as np

from collections import deque
from sensor_msgs.msg import Imu

class IMUConversion(object):
    '''
    Converting signals in IMU acceleration and angular velocity 
    ''' 
    def __init__(self):
        self._imu_sub = rospy.Subscriber("/controller/imu", Imu, self.receiveIMU)
        self._imu_pub = rospy.Publisher("/terrasentia/imu", Imu, queue_size=1)

        window_size = 5
        self.accel_x = deque(maxlen=window_size)
        self.accel_y = deque(maxlen=window_size)
        self.accel_z = deque(maxlen=window_size)
        self.gyro_x = deque(maxlen=window_size)
        self.gyro_y = deque(maxlen=window_size)
        self.gyro_z = deque(maxlen=window_size)
        self.rot_x = deque(maxlen=window_size)
        self.rot_y = deque(maxlen=window_size)
        self.rot_z = deque(maxlen=window_size)
        self.rot_w = deque(maxlen=window_size)

    def receiveIMU(self, msg):
        imu_msg = Imu()
        imu_msg.header = msg.header
        imu_msg.header.frame_id = "imu_link"

        # Store new data in sliding window
        self.accel_x.append(msg.linear_acceleration.x)
        self.accel_y.append(msg.linear_acceleration.y)
        self.accel_z.append(msg.linear_acceleration.z)
        self.gyro_x.append(msg.angular_velocity.x)
        self.gyro_y.append(msg.angular_velocity.y)
        self.gyro_z.append(msg.angular_velocity.z)
        self.rot_x.append(msg.orientation.x)
        self.rot_y.append(msg.orientation.y)
        self.rot_z.append(msg.orientation.z)
        self.rot_w.append(msg.orientation.w)

        # Publish the new filtered IMU msg
        imu_msg.angular_velocity.x = np.median(self.gyro_x)
        imu_msg.angular_velocity.y = np.median(self.gyro_y)
        imu_msg.angular_velocity.z = np.median(self.gyro_z)
        imu_msg.linear_acceleration.x = -np.median(self.accel_x)
        imu_msg.linear_acceleration.y = -np.median(self.accel_y)
        imu_msg.linear_acceleration.z = -np.median(self.accel_z)
        new_quat = np.array([
            np.median(self.rot_x),
            np.median(self.rot_y),
            np.median(self.rot_z),
            np.median(self.rot_w)
        ])

        new_quat /= np.linalg.norm(new_quat)
        imu_msg.orientation.x = new_quat[0]
        imu_msg.orientation.y = new_quat[1]
        imu_msg.orientation.z = new_quat[2]
        imu_msg.orientation.w = new_quat[3]
        
        self._imu_pub.publish(imu_msg)

if __name__ == "__main__":
    rospy.init_node("imu_conversion")
    imu_conversion = IMUConversion()
    rospy.spin()
