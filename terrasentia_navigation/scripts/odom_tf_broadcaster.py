#!/usr/bin/env python3
"""
Odom to TF Broadcaster

Subscribes to odometry messages and broadcasts the odom -> base_link transform.
This is needed because DLIO publishes odometry but does not broadcast TF.
"""

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class OdomTFBroadcaster:
    def __init__(self):
        rospy.init_node('odom_tf_broadcaster', anonymous=False)
        
        # Parameters
        self.odom_topic = rospy.get_param('~odom_topic', 'dlio/odom_node/odom')
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Subscribe to odometry
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, 
            Odometry, 
            self.odom_callback,
            queue_size=10
        )
        
        rospy.loginfo(f"OdomTFBroadcaster: Listening to {self.odom_topic}")
        rospy.loginfo("OdomTFBroadcaster: Broadcasting odom -> base_link transform")

    def odom_callback(self, msg):
        """Convert odometry message to TF and broadcast."""
        t = TransformStamped()
        
        # Header
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id  # odom
        t.child_frame_id = msg.child_frame_id    # base_link
        
        # Translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Rotation
        t.transform.rotation = msg.pose.pose.orientation
        
        # Broadcast
        self.tf_broadcaster.sendTransform(t)


# if __name__ == '__main__':
#     try:
#         node = OdomTFBroadcaster()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
