#!/usr/bin/env python

from __future__ import print_function
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped

class TFToPosePublisher:
    def __init__(self):
        rospy.init_node('tf_to_pose_publisher')
        
        # Get parameters
        self.from_frame = rospy.get_param('~from_frame', 'map').lstrip('/')
        self.to_frame = rospy.get_param('~to_frame', 'base_footprint').lstrip('/')
        self.output_topic = rospy.get_param('~output_topic', '/slam_pose')
        self.rate = rospy.get_param('~rate', 10.0)
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publisher
        self.pose_pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=10)
        
        rospy.loginfo("TF to Pose: Publishing {}->{} to {} at {}Hz".format(
            self.from_frame, self.to_frame, self.output_topic, self.rate))
        
    def run(self):
        rate = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            try:
                # Look up the transform
                transform = self.tf_buffer.lookup_transform(
                    self.from_frame,
                    self.to_frame,
                    rospy.Time(0),  # Get latest available
                    rospy.Duration(0.1)
                )
                
                # Convert to PoseStamped
                pose_msg = PoseStamped()
                pose_msg.header = transform.header
                pose_msg.pose.position.x = transform.transform.translation.x
                pose_msg.pose.position.y = transform.transform.translation.y
                pose_msg.pose.position.z = transform.transform.translation.z
                pose_msg.pose.orientation = transform.transform.rotation
                
                # Publish
                self.pose_pub.publish(pose_msg)
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn_throttle(5.0, "TF lookup failed: {}".format(str(e)))
                
            rate.sleep()

if __name__ == '__main__':
    try:
        node = TFToPosePublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
