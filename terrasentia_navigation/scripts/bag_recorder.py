#!/usr/bin/env python3
import subprocess
import signal
import os

import rospy
import rospkg
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

class BagRecorder:
    def __init__(self):
        rospy.init_node('bag_recorder_trigger')
        self.bag_process = None
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('terrasentia_navigation')

        self.launch_file_path = os.path.join(package_path, 'launch', 'rosbag_record.launch')

        # Create a service
        self.toggle_service = rospy.Service('toggle_recording', Trigger, self.toggle_recording)

        # Create publisher
        self.status_pub = rospy.Publisher('recording_status', String)

    def toggle_recording(self, req):
        if self.bag_process is None:
            # Start recording using roslaunch
            rospy.loginfo("Starting to record rosbag via roslaunch...")
            self.bag_process = subprocess.Popen(['roslaunch', self.launch_file_path], preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
            
            status = String()
            status.data = "Recording rosbag..."
            self.status_pub.publish(status)
            
            return TriggerResponse(success=True)
        else:
            # Stop recording
            rospy.loginfo("Stopping rosbag recording...")
            self.bag_process.send_signal(subprocess.signal.SIGINT)
            self.bag_process = None

            status = String()
            status.data = "Stopped recording rosbag."
            self.status_pub.publish(status)

            return TriggerResponse(success=True)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    recorder = BagRecorder()
    recorder.run()
