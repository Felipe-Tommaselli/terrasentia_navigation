import rospy
import tf

class Transform:
    def __init__(self):
        rospy.init_node('tf_listener_node', anonymous=True)
        self.tf = tf.TransformListener()
        
    def get_transform(self):
        try:
            self.tf.waitForTransform("/base_link", "/zed2_imu_link", rospy.Time(), rospy.Duration(4.0))
            position, quaternion = self.tf.lookupTransform("/base_link", "/zed2_imu_link", rospy.Time(0))
            return position, quaternion
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Exception: {}".format(e))
            return None, None

if __name__ == '__main__':
    transform = Transform()
    rospy.sleep(1)
    pos, quat = transform.get_transform()
    if pos and quat:
        rospy.loginfo("Position: {}".format(pos))
        rospy.loginfo("Quaternion: {}".format(quat))
    else:
        rospy.loginfo("Could not get transform")
