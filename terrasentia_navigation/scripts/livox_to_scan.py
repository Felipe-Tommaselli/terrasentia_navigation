#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan, PointCloud2
import numpy as np
import math

class LivoxToScan:
    def __init__(self):
        rospy.init_node('livox_to_scan', anonymous=True)

        # Parameters
        self.points_per_scan = rospy.get_param('~points_per_scan', 1081)
        self.scan_topic = rospy.get_param('~scan_topic', '/terrasentia/scan')
        self.livox_topic = rospy.get_param('~livox_topic', '/terrasentia/livox/lidar')
        self.min_height = rospy.get_param('~min_height', -0.1) # Slice thickness around 0
        self.max_height = rospy.get_param('~max_height', 0.1)
        self.range_min = rospy.get_param('~range_min', 0.1)
        self.range_max = rospy.get_param('~range_max', 30.0)
        self.fov = rospy.get_param('~fov', 360.0) # Degrees

        self.scan_pub = rospy.Publisher(self.scan_topic, LaserScan, queue_size=1)
        self.lidar_sub = rospy.Subscriber(self.livox_topic, PointCloud2, self.callback)
        
        rospy.loginfo(f"LivoxToScan initialized. Publishing {self.points_per_scan} points to {self.scan_topic}")

    def callback(self, msg):
        # Create LaserScan message
        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = "livox_frame" # Force frame if needed, or use msg.header.frame_id
        
        scan.angle_min = -math.radians(self.fov / 2.0)
        scan.angle_max = math.radians(self.fov / 2.0)
        scan.angle_increment = math.radians(self.fov) / (self.points_per_scan - 1)
        scan.time_increment = 0.0 # Instantaneous scan assumption
        scan.scan_time = 0.1 # 10Hz assumption
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Initialize ranges with Infinity
        ranges = [float('inf')] * self.points_per_scan
        
        # Read points
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        for x, y, z in points:
            # Fliter by height (Z slice)
            if z < self.min_height or z > self.max_height:
                continue

            dist = math.sqrt(x*x + y*y)
            if dist < self.range_min or dist > self.range_max:
                continue

            angle = math.atan2(y, x)
            
            # Normalize angle to [-PI, PI]
            # atan2 returns in [-PI, PI] already. 
            
            # Check FOV
            if angle < scan.angle_min or angle > scan.angle_max:
                 # Try wrapping if close to boundary? 
                 # For 360 scan, angle_min is -PI, angle_max is PI. 
                 # This should cover everything.
                 continue

            # Calculate index
            # angle = min + index * inc
            # index = (angle - min) / inc
            index = int((angle - scan.angle_min) / scan.angle_increment)

            if 0 <= index < self.points_per_scan:
                # Keep the closest point in that bin
                if dist < ranges[index]:
                    ranges[index] = dist

        scan.ranges = ranges
        self.scan_pub.publish(scan)

if __name__ == '__main__':
    try:
        LivoxToScan()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
