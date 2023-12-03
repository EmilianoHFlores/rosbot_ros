#!/usr/bin/python3

import rospy
import tf
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelState, LinkState

class TransformTransformer:
    def __init__(self):
        rospy.init_node('transform_transformer', anonymous=True)
        # listen to pose estimate from rviz
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.pose_callback)
        # publish the transform
        self.br = tf.TransformBroadcaster()
        # publish to gazebo
        self.gazebo_pub = rospy.Publisher("/gazebo/set_link_state", LinkState, queue_size=10)

        rospy.logdebug("Transform Transformer node started")
        self.currentTransform = PoseWithCovarianceStamped()

        while True:
            # publish the transform
            self.br.sendTransform((self.currentTransform.pose.pose.position.x, self.currentTransform.pose.pose.position.y, self.currentTransform.pose.pose.position.z),
                                    (self.currentTransform.pose.pose.orientation.x, self.currentTransform.pose.pose.orientation.y, self.currentTransform.pose.pose.orientation.z, self.currentTransform.pose.pose.orientation.w),
                                    rospy.Time.now(),
                                    "base_link",
                                    "map")
            # publish to gazebo
            link_state = LinkState()
            link_state.link_name = "base_link"
            link_state.pose = self.currentTransform.pose.pose
            self.gazebo_pub.publish(link_state)
            rospy.sleep(0.01)



    def pose_callback(self, msg):
        # publish the transform
        self.currentTransform = msg
        
        rospy.loginfo("Transform received")


if __name__ == '__main__':
    try:
        transform_transformer = TransformTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass