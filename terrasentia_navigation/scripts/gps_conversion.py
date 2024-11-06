#!/usr/bin/env python3
from std_msgs.msg import Float64MultiArray

import rospy
import numpy as np

from sensor_msgs.msg import NavSatFix, NavSatStatus

class GPSPublisher():
    def __init__(self):
        # Publish terra-controller sensors  
        gps_topic = rospy.get_param('~gps_topic', 'fix')

        # Publisher
        self.pub_gps = rospy.Publisher(gps_topic, NavSatFix, queue_size=1)

        # Subscriber
        self.sub_controller_gps = rospy.Subscriber('/controller/gps_array', Float64MultiArray, self.gps_callback, queue_size=10)

        rospy.spin()
            
    def gps_callback(self, msg):
        # Create a new NavSatFix message
        gps_data = NavSatFix()
        
        # Set the header timestamp to the current time
        gps_data.header.stamp = rospy.Time.now()

        # Assume data is good unless proven otherwise
        data_is_good = True

        # Fill in the NavSatFix fields if data is valid (finite)
        if np.isfinite(msg.data[8]):  # Latitude
            gps_data.latitude = msg.data[8]
        else:
            data_is_good = False

        if np.isfinite(msg.data[9]):  # Longitude
            gps_data.longitude = msg.data[9]
        else:
            data_is_good = False

        if np.isfinite(msg.data[0]):  # Altitude
            gps_data.altitude = msg.data[0]
        else:
            data_is_good = False

        if np.isfinite(msg.data[7]):  # Horizontal accuracy (position_covariance)
            gps_data.position_covariance[0] = msg.data[7] ** 2  # Variance (square of accuracy)
            gps_data.position_covariance[4] = msg.data[7] ** 2  # Variance (square of accuracy)
            gps_data.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            data_is_good = False

        # Set status to STATUS_FIX (indicating a valid fix)
        gps_data.status.status = NavSatStatus.STATUS_FIX

        # Assuming this is using GPS (not differential GPS)
        gps_data.status.service = NavSatStatus.SERVICE_GPS

        if data_is_good:
            # Publish the valid GPS data
            self.pub_gps.publish(gps_data)
        else:
            rospy.logwarn('Not publishing GPS data because some fields are not finite')

if __name__ == '__main__':
    rospy.init_node('gps_publisher')
    node = GPSPublisher()