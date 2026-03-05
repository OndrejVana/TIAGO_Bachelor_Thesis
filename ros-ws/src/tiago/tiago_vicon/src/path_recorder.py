#!/usr/bin/env python

import rospy
import yaml
import os
from datetime import datetime
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse


class PathRecorder:
    def __init__(self):
        rospy.init_node('path_recorder', anonymous=False)
        
        # Parameters
        self.pose_topic = rospy.get_param('~pose_topic', '/mobile_base_controller/odom')
        self.use_odom = rospy.get_param('~use_odom', False)
        self.slam_pose_topic = rospy.get_param('~slam_pose_topic', '/slam_pose')
        self.output_dir = rospy.get_param('~output_dir', os.path.expanduser('./src/custom/tiago/recorded_paths'))
        self.recording_rate = rospy.get_param('~recording_rate', 2.0)
        self.min_distance = rospy.get_param('~min_distance', 0.1)
        
        # Create output directory
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            rospy.loginfo("Created output directory: {}".format(self.output_dir))
        
        # Recording state
        self.is_recording = False
        self.waypoints = []
        self.last_recorded_pose = None
        self.latest_pose = None
        
        # Subscribers
        if self.use_odom:
            rospy.Subscriber(self.pose_topic, Odometry, self.odom_callback)
        else:
            rospy.Subscriber(self.slam_pose_topic, PoseStamped, self.pose_callback)
        
        # Services
        self.start_service = rospy.Service('~start_recording', Trigger, self.start_recording)
        self.stop_service = rospy.Service('~stop_recording', Trigger, self.stop_recording)
        self.save_service = rospy.Service('~save_path', Trigger, self.save_path)
        
        # Timer for periodic recording
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.recording_rate), self.record_callback)
    
    def odom_callback(self, msg):
        """Handle odometry messages"""
        self.latest_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'qx': msg.pose.pose.orientation.x,
            'qy': msg.pose.pose.orientation.y,
            'qz': msg.pose.pose.orientation.z,
            'qw': msg.pose.pose.orientation.w,
            'frame_id': msg.header.frame_id,
            'timestamp': msg.header.stamp.to_sec()
        }
    
    def pose_callback(self, msg):
        """Handle PoseStamped messages"""
        self.latest_pose = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'qx': msg.pose.orientation.x,
            'qy': msg.pose.orientation.y,
            'qz': msg.pose.orientation.z,
            'qw': msg.pose.orientation.w,
            'frame_id': msg.header.frame_id,
            'timestamp': msg.header.stamp.to_sec()
        }
    
    def start_recording(self, req):
        """Service callback to start recording"""
        if self.is_recording:
            return TriggerResponse(success=False, message="Already recording!")
        
        self.is_recording = True
        self.waypoints = []
        self.last_recorded_pose = None
        
        rospy.loginfo("="*50)
        rospy.loginfo("Started recording path")
        rospy.loginfo("="*50)
        
        return TriggerResponse(success=True, message="Recording started. Move the robot to record waypoints.")
    def stop_recording(self, req):
        """Service callback to stop recording and save"""
        if not self.is_recording:
            return TriggerResponse(success=False, message="Not currently recording!")
        
        self.is_recording = False
        
        rospy.loginfo("="*50)
        rospy.loginfo("Stopped recording")
        rospy.loginfo("Recorded {} waypoints".format(len(self.waypoints)))
        rospy.loginfo("="*50)
        
        save_result = self.save_path(None)
        
        return TriggerResponse(
            success=True, 
            message="Recording stopped. Recorded {} waypoints. {}".format(
                len(self.waypoints), save_result.message
            )
        )
    
    def save_path(self, req):
        """Save recorded path to YAML file"""
        if len(self.waypoints) == 0:
            rospy.logwarn("No waypoints to save!")
            return TriggerResponse(success=False, message="No waypoints to save!")
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.output_dir, 'path_{}.yaml'.format(timestamp))
        
        path_data = {
            'metadata': {
                'timestamp': timestamp,
                'num_waypoints': len(self.waypoints),
                'frame_id': self.waypoints[0].get('frame_id', 'odom') if self.waypoints else 'odom',
                'source': 'odometry' if self.use_odom else 'slam_pose'
            },
            'waypoints': self.waypoints
        }
        
        try:
            with open(filename, 'w') as f:
                yaml.dump(path_data, f, default_flow_style=False)
            
            rospy.loginfo("="*50)
            rospy.loginfo("Path saved successfully!")
            rospy.loginfo("  File: {}".format(filename))
            rospy.loginfo("  Waypoints: {}".format(len(self.waypoints)))
            rospy.loginfo("="*50)
            
            return TriggerResponse(
                success=True, 
                message="Path saved to {}".format(filename)
            )
        except Exception as e:
            error_msg = "Failed to save path: {}".format(str(e))
            rospy.logerr(error_msg)
            return TriggerResponse(success=False, message=error_msg)
    
    def record_callback(self, event):
        """Periodic callback to record waypoints"""
        if not self.is_recording or self.latest_pose is None:
            return
        
        if self.should_record_waypoint():
            waypoint = {
                'x': self.latest_pose['x'],
                'y': self.latest_pose['y'],
                'z': self.latest_pose['z'],
                'qx': self.latest_pose['qx'],
                'qy': self.latest_pose['qy'],
                'qz': self.latest_pose['qz'],
                'qw': self.latest_pose['qw'],
                'frame_id': self.latest_pose['frame_id'],
                'timestamp': self.latest_pose['timestamp']
            }
            
            self.waypoints.append(waypoint)
            self.last_recorded_pose = self.latest_pose.copy()
            
            rospy.loginfo("Recorded waypoint #{}: ({:.2f}, {:.2f}, {:.2f})".format(
                len(self.waypoints),
                waypoint['x'],
                waypoint['y'],
                waypoint['z']
            ))
    
    def should_record_waypoint(self):
        """Check if current pose should be recorded as a waypoint"""
        if self.last_recorded_pose is None:
            return True

        dx = self.latest_pose['x'] - self.last_recorded_pose['x']
        dy = self.latest_pose['y'] - self.last_recorded_pose['y']
        dz = self.latest_pose['z'] - self.last_recorded_pose['z']
        distance = (dx**2 + dy**2 + dz**2)**0.5
        
        return distance >= self.min_distance
    
    def run(self):
        """Keep the node running"""
        rospy.spin()


if __name__ == '__main__':
    try:
        recorder = PathRecorder()
        recorder.run()
    except rospy.ROSInterruptException:
        pass
