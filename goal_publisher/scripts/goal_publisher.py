#!/usr/bin/env python3
import tf
import copy
import rospy
import threading
import numpy as np
import message_filters

from nav_msgs.msg import Odometry
from geodesy.utm import fromLatLong
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

from gps_optimization import GPSOptimization

class GPSOdomTransformNode:
    def __init__(self):
        # Initialize node
        rospy.init_node('goal_publisher_node', anonymous=True)

        # Parameters
        self.min_distance = rospy.get_param('~min_distance', 5.0)               # Minimum distance for new GPS node
        self.min_num_nodes = rospy.get_param('~min_nodes', 3)                   # Number of nodes to collect before optimization
        self.max_num_nodes = rospy.get_param('~max_nodes', 100)                 # Maximum number of nodes to store
        self.goal_file = rospy.get_param('~goal_file', '/path/to/goal.txt')     # Path to goal file
        odom_topic = rospy.get_param('~odom_topic', '/terrasentia/odom')
        gps_topic = rospy.get_param('~gps_topic', '/terrasentia/fix')
        goal_topic = rospy.get_param('~goal_topic', '/move_base_simple/goal')

        # Initialize variables
        self.odom_gps_pairs = []
        self.tf_frame = 'odom'
        self.optimization_done = False
        self.transformation = np.eye(4)

        # Load goal from file in lat/lon format
        self.goal_lat, self.goal_lon = self.load_goal(self.goal_file)

        # Lock for thread-safe access to shared data (odom_gps_pairs)
        self.data_lock = threading.Lock()

        # Publisher for goal in odom frame
        self.goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=10)

        # Subscribers for odometry and GPS data using message_filters for synchronization
        odom_sub = message_filters.Subscriber(odom_topic, Odometry)
        gps_sub = message_filters.Subscriber(gps_topic, NavSatFix)

        # ApproximateTime synchronizer with a queue size of 10 and slop of 0.1 seconds
        sync = message_filters.ApproximateTimeSynchronizer([odom_sub, gps_sub], queue_size=10, slop=0.1)
        
        # Register synchronized callback
        sync.registerCallback(self.sync_callback)

        # Start optimization in a separate thread so it doesn't block callbacks
        optimization_thread = threading.Thread(target=self.optimize_transformation)
        optimization_thread.daemon = True  # Daemon thread will exit when main program exits
        optimization_thread.start()

    def load_goal(self, filepath):
        """Load the goal coordinates from a text file."""
        with open(filepath, 'r') as f:
            lat, lon = map(float, f.readline().split(','))
        return lat, lon

    def sync_callback(self, odom_data, gps_data):
        """
        Callback function to handle synchronized odometry and GPS data.
        """
        self.tf_frame = odom_data.header.frame_id

        odom_point = [odom_data.pose.pose.position.x, odom_data.pose.pose.position.y, odom_data.pose.pose.position.z]
        
        # Convert GPS data to UTM coordinates
        gps_utm = fromLatLong(gps_data.latitude, gps_data.longitude)
        gps_point = [gps_utm.easting, gps_utm.northing, gps_data.altitude]

        # Extract GPS accuracy from covariance matrix (assuming diagonal covariance)
        gps_accuracy = np.sqrt(np.mean(np.diag(gps_data.position_covariance)))

        new_data_point = odom_point + gps_point + [gps_accuracy]
        
        with self.data_lock:
            if len(self.odom_gps_pairs) == 0:
                self.odom_gps_pairs.append(new_data_point)

            else:
                prev_odom_point = self.odom_gps_pairs[-1][:3]
                prev_gps_point = self.odom_gps_pairs[-1][3:6]

                odom_dist = np.linalg.norm(np.array(odom_point) - np.array(prev_odom_point))
                gps_dist = np.linalg.norm(np.array(gps_point) - np.array(prev_gps_point))
                
                # Add new pair if distance threshold is exceeded
                if odom_dist > self.min_distance and gps_dist > self.min_distance:
                    self.odom_gps_pairs.append(new_data_point)

                # Remove oldest pair if maximum number of nodes is reached
                if len(self.odom_gps_pairs) > self.max_num_nodes:
                    self.odom_gps_pairs.pop(0)

    def optimize_transformation(self):
        """
        Optimization thread function that uses CasADi to minimize the squared distance between odometry and GPS measurements.
        Optimizes for 3 translation offsets and 3 rotation angles.
        """

        # Initialize GPSOptimization class
        gps_optimization = GPSOptimization(self.transformation, self.max_num_nodes)

        while not rospy.is_shutdown():
            if len(self.odom_gps_pairs) < self.min_num_nodes:
                rospy.logwarn("{} points collected for optimization. Walk with the robot to collect more data.".format(len(self.odom_gps_pairs)))
                rospy.sleep(1)
                continue

            rospy.loginfo("Starting optimization...")
            start_time = rospy.Time.now()

            with self.data_lock:
                data_buffer = copy.deepcopy(self.odom_gps_pairs)

            # Optimize transformation
            data_buffer = np.array(data_buffer).T
            transformation = gps_optimization.run(data_buffer)

            with self.data_lock:
                self.transformation = transformation
                self.optimization_done = True

            rospy.loginfo("Optimization complete with {} nodes".format(len(self.odom_gps_pairs)))
            rospy.loginfo("Optimized transformation: \n%s", transformation)
            rospy.loginfo("Optimization finished in %s seconds", (rospy.Time.now() - start_time).to_sec())

    def publish_goal(self, goal_lat, goal_lon):
        """
        Convert lat/lon goal into odometry frame using the optimized transformation and publish it.
        """
        goal_utm = fromLatLong(goal_lat, goal_lon)
        goal_x, goal_y = goal_utm.easting, goal_utm.northing

        # Apply transformation to goal coordinates
        transformed_goal = self.transformation[:3,:3] @ np.array([goal_x, goal_y, 0])
        transformed_goal = transformed_goal + self.transformation[:3,3]

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = self.tf_frame
        goal_msg.pose.position.x = transformed_goal[0]
        goal_msg.pose.position.y = transformed_goal[1]

        self.goal_pub.publish(goal_msg)

    def run(self):
        """Main loop to run optimization and publish goals."""
        
        rate = rospy.Rate(10)  # Run at 10 Hz
        
        while not rospy.is_shutdown():
            if self.optimization_done:
                self.publish_goal(self.goal_lat, self.goal_lon)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = GPSOdomTransformNode()
        node.run()

    except rospy.ROSInterruptException:
        pass