#!/usr/bin/env python

import rospy
import csv
import os
from datetime import datetime
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry


class PositionLogger:
    def __init__(self):
        rospy.init_node('position_logger', anonymous=True)
        
        # Parameters
        self.vicon_topic = rospy.get_param('~vicon_topic', '/vicon/tiago/tiago')
        self.robot_pose_topic = rospy.get_param('~robot_pose_topic', '/tiago_orb_slam2_rgbd/pose')
        self.robot_odom_topic = rospy.get_param('~robot_odom_topic', '/mobile_base_controller/odom')
        self.use_odom = rospy.get_param('~use_odom', False)  # Use odom instead of SLAM pose
        self.vicon_msg_type = rospy.get_param('~vicon_msg_type', 'pose')  # 'pose' or 'transform'
        self.output_dir = rospy.get_param('~output_dir', os.path.expanduser('~/position_logs'))
        self.log_rate = rospy.get_param('~log_rate', 10.0)  # Hz
        
        # Create output directory
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        # Generate filename with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.base_filename = os.path.join(self.output_dir, 'position_log_{}'.format(timestamp))
        
        # Data storage
        self.data = {
            'vicon': [],
            'robot': [],
            'timestamps': [],
            'metadata': {
                'start_time': rospy.Time.now().to_sec(),
                'vicon_topic': self.vicon_topic,
                'robot_topic': self.robot_pose_topic if not self.use_odom else self.robot_odom_topic,
                'vicon_msg_type': self.vicon_msg_type,
                'log_rate': self.log_rate,
                'use_odom': self.use_odom
            }
        }
        
        # Latest data from subscribers
        self.latest_vicon = None
        self.latest_robot = None
        
        # Subscribers: subscribe to Vicon topic with configured message type
        if str(self.vicon_msg_type).lower() == 'transform':
            rospy.Subscriber(self.vicon_topic, TransformStamped, self.vicon_transform_callback)
        else:
            rospy.Subscriber(self.vicon_topic, PoseStamped, self.vicon_pose_callback)

        if self.use_odom:
            rospy.Subscriber(self.robot_odom_topic, Odometry, self.robot_odom_callback)
        else:
            rospy.Subscriber(self.robot_pose_topic, PoseStamped, self.robot_pose_callback)
        
        # Timer for periodic logging
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.log_rate), self.log_callback)
        
        rospy.loginfo("Position Logger started")
        rospy.loginfo("  Vicon topic: {}".format(self.vicon_topic))
        rospy.loginfo("  Robot topic: {}".format(self.robot_pose_topic if not self.use_odom else self.robot_odom_topic))
        rospy.loginfo("  Output directory: {}".format(self.output_dir))
        rospy.loginfo("  Log rate: {} Hz".format(self.log_rate))
        rospy.loginfo("  Base filename: {}".format(self.base_filename))
        
        # Register shutdown hook
        rospy.on_shutdown(self.save_data)
    
    def vicon_transform_callback(self, msg):
        """Handle Vicon data as TransformStamped"""
        self.latest_vicon = {
            'timestamp': msg.header.stamp.to_sec(),
            'x': msg.transform.translation.x,
            'y': msg.transform.translation.y,
            'z': msg.transform.translation.z,
            'qx': msg.transform.rotation.x,
            'qy': msg.transform.rotation.y,
            'qz': msg.transform.rotation.z,
            'qw': msg.transform.rotation.w
        }
    
    def vicon_pose_callback(self, msg):
        """Handle Vicon data as PoseStamped"""
        self.latest_vicon = {
            'timestamp': msg.header.stamp.to_sec(),
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'qx': msg.pose.orientation.x,
            'qy': msg.pose.orientation.y,
            'qz': msg.pose.orientation.z,
            'qw': msg.pose.orientation.w
        }
    
    def robot_pose_callback(self, msg):
        """Handle robot SLAM pose data"""
        self.latest_robot = {
            'timestamp': msg.header.stamp.to_sec(),
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'qx': msg.pose.orientation.x,
            'qy': msg.pose.orientation.y,
            'qz': msg.pose.orientation.z,
            'qw': msg.pose.orientation.w
        }
    
    def robot_odom_callback(self, msg):
        """Handle robot odometry data"""
        self.latest_robot = {
            'timestamp': msg.header.stamp.to_sec(),
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'qx': msg.pose.pose.orientation.x,
            'qy': msg.pose.pose.orientation.y,
            'qz': msg.pose.pose.orientation.z,
            'qw': msg.pose.pose.orientation.w
        }
    
    def log_callback(self, event):
        """Periodic logging callback"""
        if self.latest_robot is not None:
            current_time = rospy.Time.now().to_sec()
            
            # If no Vicon data, use None values
            if self.latest_vicon is not None:
                vicon_data = self.latest_vicon.copy()
            else:
                vicon_data = {
                    'timestamp': 0.0,
                    'x': 0.0, 'y': 0.0, 'z': 0.0,
                    'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0
                }
                rospy.logwarn_throttle(10.0, "No Vicon data available - logging robot data only")
            
            self.data['vicon'].append(vicon_data)
            self.data['robot'].append(self.latest_robot.copy())
            self.data['timestamps'].append(current_time)
            
            # Print status every 50 samples
            if len(self.data['timestamps']) % 50 == 0:
                rospy.loginfo("Logged {} samples".format(len(self.data['timestamps'])))
        else:
            rospy.logwarn_throttle(5.0, "No robot pose data received yet")
    
    def save_data(self):
        """Save collected data to files"""
        if len(self.data['timestamps']) == 0:
            rospy.logwarn("No data collected, skipping save")
            return
        
        rospy.loginfo("Saving {} samples...".format(len(self.data['timestamps'])))
        
        # Add end time to metadata
        self.data['metadata']['end_time'] = rospy.Time.now().to_sec()
        self.data['metadata']['duration'] = self.data['metadata']['end_time'] - self.data['metadata']['start_time']
        self.data['metadata']['num_samples'] = len(self.data['timestamps'])
        
        # Save as CSV file
        csv_file = '{}.csv'.format(self.base_filename)
        with open(csv_file, 'w') as f:
            writer = csv.writer(f)
            
            # Header
            writer.writerow([
                'log_timestamp',
                'vicon_timestamp', 'vicon_x', 'vicon_y', 'vicon_z', 
                'vicon_qx', 'vicon_qy', 'vicon_qz', 'vicon_qw',
                'robot_timestamp', 'robot_x', 'robot_y', 'robot_z',
                'robot_qx', 'robot_qy', 'robot_qz', 'robot_qw'
            ])
            
            # Data rows
            for i, timestamp in enumerate(self.data['timestamps']):
                vicon = self.data['vicon'][i]
                robot = self.data['robot'][i]
                
                writer.writerow([
                    timestamp,
                    vicon['timestamp'], vicon['x'], vicon['y'], vicon['z'],
                    vicon['qx'], vicon['qy'], vicon['qz'], vicon['qw'],
                    robot['timestamp'], robot['x'], robot['y'], robot['z'],
                    robot['qx'], robot['qy'], robot['qz'], robot['qw']
                ])
        
        rospy.loginfo("Saved CSV file: {}".format(csv_file))
        
        # Save metadata as separate text file
        metadata_file = '{}_metadata.txt'.format(self.base_filename)
        with open(metadata_file, 'w') as f:
            for key, value in self.data['metadata'].items():
                f.write("{}: {}\n".format(key, value))
        rospy.loginfo("Saved metadata: {}".format(metadata_file))
        
        rospy.loginfo("Data logging complete! Files saved to: {}".format(self.output_dir))


def main():
    try:
        logger = PositionLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
