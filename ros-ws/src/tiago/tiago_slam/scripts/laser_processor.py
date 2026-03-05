#!/usr/bin/env python

from __future__ import division
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_point
import math


class LaserProcessor(object):
    """
    Enhanced laser processor for SLAM applications.
    Processes laser scan data and provides utilities for mapping and navigation.
    """
    
    def __init__(self):
        rospy.loginfo("Initializing TIAGo SLAM Laser Processor...")
        
        # Initialize variables
        self.last_scan = None
        self.processed_scan = None
        self.obstacles = []
        self.free_space = []
        
        # Configuration parameters
        self.min_range = rospy.get_param('~min_range', 0.1)  # meters
        self.max_range = rospy.get_param('~max_range', 10.0)  # meters
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 2.0)  # meters
        self.angular_resolution = rospy.get_param('~angular_resolution', 1.0)  # degrees
        
        # TF2 buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publishers
        self.processed_scan_pub = rospy.Publisher(
            '/scan_processed', LaserScan, queue_size=1
        )
        self.obstacle_markers_pub = rospy.Publisher(
            '/obstacle_markers', MarkerArray, queue_size=1
        )
        self.cmd_vel_pub = rospy.Publisher(
            '/mobile_base_controller/cmd_vel', Twist, queue_size=1
        )
        
        # Subscribers
        self.laser_sub = rospy.Subscriber(
            '/scan', LaserScan, self.scan_callback, queue_size=1
        )
        
        rospy.loginfo("Subscribed to: '" + str(self.laser_sub.resolved_name) + "' topic.")
        rospy.loginfo("TIAGo SLAM Laser Processor initialized successfully!")
        
    def scan_callback(self, msg):
        """
        Callback for laser scan messages.
        :type msg: LaserScan
        """
        self.last_scan = msg
        self.process_scan(msg)
        
    def process_scan(self, scan_msg):
        """
        Process the laser scan for SLAM applications.
        Filters noise, detects obstacles, and identifies free space.
        """
        if scan_msg is None:
            return
            
        # Create a copy for processing
        processed_scan = LaserScan()
        processed_scan.header = scan_msg.header
        processed_scan.angle_min = scan_msg.angle_min
        processed_scan.angle_max = scan_msg.angle_max
        processed_scan.angle_increment = scan_msg.angle_increment
        processed_scan.time_increment = scan_msg.time_increment
        processed_scan.scan_time = scan_msg.scan_time
        processed_scan.range_min = scan_msg.range_min
        processed_scan.range_max = scan_msg.range_max
        
        # Process ranges
        filtered_ranges = []
        obstacles = []
        
        for i, range_val in enumerate(scan_msg.ranges):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            
            # Filter invalid readings
            if math.isnan(range_val) or math.isinf(range_val):
                filtered_ranges.append(scan_msg.range_max)
                continue
                
            # Apply range limits
            if range_val < self.min_range:
                filtered_ranges.append(self.min_range)
            elif range_val > self.max_range:
                filtered_ranges.append(self.max_range)
            else:
                filtered_ranges.append(range_val)
                
            # Detect obstacles
            if range_val <= self.obstacle_threshold and range_val >= self.min_range:
                # Convert to Cartesian coordinates
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                obstacles.append((x, y, angle, range_val))
        
        processed_scan.ranges = filtered_ranges
        self.processed_scan = processed_scan
        self.obstacles = obstacles
        
        # Publish processed scan
        self.processed_scan_pub.publish(processed_scan)
        
        # Publish obstacle markers for visualization
        self.publish_obstacle_markers(obstacles, scan_msg.header)
        
    def publish_obstacle_markers(self, obstacles, header):
        """
        Publish visualization markers for detected obstacles.
        """
        marker_array = MarkerArray()
        
        for i, (x, y, angle, range_val) in enumerate(obstacles):
            marker = Marker()
            marker.header = header
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker.lifetime = rospy.Duration(1.0)
            marker_array.markers.append(marker)
            
        self.obstacle_markers_pub.publish(marker_array)
        
    def get_obstacle_density(self):
        """
        Calculate the density of obstacles in different sectors.
        Returns a dictionary with sector information.
        """
        if not self.obstacles:
            return {}
            
        sectors = {
            'front': 0,
            'left': 0,
            'back': 0,
            'right': 0
        }
        
        for x, y, angle, range_val in self.obstacles:
            angle_deg = math.degrees(angle)
            
            if -30 <= angle_deg <= 30:
                sectors['front'] += 1
            elif 30 <= angle_deg <= 150:
                sectors['left'] += 1
            elif angle_deg >= 150 or angle_deg <= -150:
                sectors['back'] += 1
            elif -150 <= angle_deg <= -30:
                sectors['right'] += 1
                
        return sectors
        
    def get_navigation_recommendation(self):
        """
        Provide navigation recommendations based on obstacle detection.
        Returns a Twist message for safe navigation.
        """
        cmd_vel = Twist()
        
        if not self.obstacles:
            # No obstacles detected, safe to move forward
            cmd_vel.linear.x = 0.3
            return cmd_vel
            
        sectors = self.get_obstacle_density()
        
        # Simple obstacle avoidance logic
        if sectors['front'] > 5:  # Too many obstacles in front
            if sectors['left'] < sectors['right']:
                cmd_vel.angular.z = 0.5  # Turn left
            else:
                cmd_vel.angular.z = -0.5  # Turn right
        else:
            cmd_vel.linear.x = 0.2  # Move forward slowly
            
        return cmd_vel
        
    def run(self):
        """
        Main processing loop for SLAM laser processing.
        """
        rospy.loginfo("Starting TIAGo SLAM Laser Processor main loop...")
        rospy.loginfo("Waiting for first LaserScan message...")
        
        while not rospy.is_shutdown() and self.last_scan is None:
            rospy.sleep(0.1)
            
        rospy.loginfo("First laser scan received. Processing started!")
        
        # Process at 10Hz
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.last_scan is not None:
                # Analyze current situation
                sectors = self.get_obstacle_density()
                
                # Log analysis results
                if len(self.obstacles) > 0:
                    rospy.loginfo_throttle(
                        2.0, 
                        "Obstacles detected: {} total. "
                        "Sectors - Front: {}, "
                        "Left: {}, "
                        "Right: {}, "
                        "Back: {}".format(
                            len(self.obstacles),
                            sectors.get('front', 0),
                            sectors.get('left', 0),
                            sectors.get('right', 0),
                            sectors.get('back', 0)
                        )
                    )
                else:
                    rospy.loginfo_throttle(5.0, "Clear path detected - no obstacles in range")
                    
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('tiago_slam_laser_processor')
        processor = LaserProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("TIAGo SLAM Laser Processor node terminated.")