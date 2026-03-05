#!/usr/bin/env python

from __future__ import print_function
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class TrajectoryPublisher:
    
    def __init__(self):
        rospy.init_node('trajectory_publisher')
        
        # Parameters
        self.min_distance = rospy.get_param('~min_distance', 0.05)  # meters
        self.max_points = rospy.get_param('~max_points', 10000)  # Maximum trajectory points
        
        # Path to store trajectory
        self.path = Path()
        self.path.header.frame_id = "map"
        
        # Last recorded position
        self.last_pose = None
        
        # TF buffer for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publisher
        self.path_pub = rospy.Publisher('/trajectory', Path, queue_size=1, latch=True)
        
        # Subscriber to odometry
        self.odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.odom_callback)
        
        rospy.loginfo("Trajectory publisher started. Recording robot path...")
        rospy.loginfo("Min distance between points: %.2f m", self.min_distance)
        rospy.loginfo("Max trajectory points: %d", self.max_points)
    
    def odom_callback(self, msg):
        """Process odometry and add to trajectory if robot moved enough"""
        try:
            # Get current pose in map frame
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose
            
            try:
                source_frame = msg.header.frame_id.lstrip('/')
                
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    source_frame,
                    rospy.Time(0),
                    rospy.Duration(1.0)
                )
                pose_in_map = do_transform_pose(pose_stamped, transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                return
            
            if self.should_add_point(pose_in_map):
                pose_in_map.header.stamp = rospy.Time.now()
                self.path.poses.append(pose_in_map)
                self.last_pose = pose_in_map
                
                # Limit number of points
                if len(self.path.poses) > self.max_points:
                    self.path.poses.pop(0)
                
                # Update and publish path
                self.path.header.stamp = rospy.Time.now()
                self.path_pub.publish(self.path)
                
        except Exception as e:
            rospy.logwarn_throttle(5.0, "Error processing odometry: %s", str(e))
    
    def should_add_point(self, pose):
        """Check if new pose is far enough from last recorded pose"""
        if self.last_pose is None:
            return True
        
        # Calculate distance from last pose
        dx = pose.pose.position.x - self.last_pose.pose.position.x
        dy = pose.pose.position.y - self.last_pose.pose.position.y
        distance = (dx**2 + dy**2)**0.5
        
        return distance >= self.min_distance
    
    def run(self):
        """Keep node running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        node = TrajectoryPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
