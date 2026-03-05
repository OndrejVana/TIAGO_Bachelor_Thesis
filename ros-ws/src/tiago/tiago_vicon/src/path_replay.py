#!/usr/bin/env python

import rospy
import yaml
import actionlib
import tf
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Quaternion
from std_srvs.srv import Trigger, TriggerResponse
from actionlib_msgs.msg import GoalStatus
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry


class PathReplay:
    def __init__(self):
        rospy.init_node('path_replay', anonymous=False)
        
        # Parameters
        self.path_file = rospy.get_param('~path_file', '')
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.5)
        self.loop_replay = rospy.get_param('~loop_replay', False)
        self.publish_path = rospy.get_param('~publish_path', True)
        
        # State
        self.waypoints = []
        self.original_waypoints = []  # Store original waypoints before offset
        self.current_waypoint_index = 0
        self.is_replaying = False
        self.move_base_client = None
        self.current_pose = None
        self.pose_received = False
        
        # Subscribers
        self.odom_topic = rospy.get_param('~odom_topic', '/mobile_base_controller/odom')
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.pose_callback)
        
        # Publishers
        if self.publish_path:
            from nav_msgs.msg import Path
            self.path_pub = rospy.Publisher('~recorded_path', Path, queue_size=1, latch=True)
        
        # Services
        self.start_service = rospy.Service('~start_replay', Trigger, self.start_replay)
        self.stop_service = rospy.Service('~stop_replay', Trigger, self.stop_replay)
        self.load_service = rospy.Service('~load_path', Trigger, self.load_path_service)
        
        # Initialize move_base action client
        self.move_base_action_name = rospy.get_param('~move_base_action', 'tiago_move_base/move_base')
        self.move_base_client = actionlib.SimpleActionClient(self.move_base_action_name, MoveBaseAction)
        if not self.move_base_client.wait_for_server(rospy.Duration(30.0)):
            rospy.logwarn("%s action server not available after 30s. Start it and then call ~start_replay. Will retry when starting replay.", self.move_base_action_name)
        else:
            rospy.loginfo("Connected to %s action server", self.move_base_action_name)
           
        if self.path_file:
            self.load_path(self.path_file)

        self.abort_retry_count = rospy.get_param('~abort_retry_count', 3)
        self.abort_retry_delay = rospy.get_param('~abort_retry_delay', 2.0)
        self._retry_counters = {}
        self.last_goal = None
    
    def pose_callback(self, msg):
        """Callback to store current robot pose"""
        self.current_pose = msg.pose.pose
        self.pose_received = True
    
    def load_path(self, filename):
        """Load waypoints from YAML file"""
        try:
            with open(filename, 'r') as f:
                path_data = yaml.safe_load(f)
            
            self.waypoints = path_data.get('waypoints', [])
            self.original_waypoints = list(self.waypoints)
            metadata = path_data.get('metadata', {})
            
            rospy.loginfo("="*50)
            rospy.loginfo("Loaded path from: {}".format(filename))
            rospy.loginfo("  Waypoints: {}".format(len(self.waypoints)))
            rospy.loginfo("  Frame: {}".format(metadata.get('frame_id', 'unknown')))
            rospy.loginfo("  Source: {}".format(metadata.get('source', 'unknown')))
            rospy.loginfo("="*50)
            
            if self.publish_path and len(self.waypoints) > 0:
                self.publish_path_visualization()
            
            return True
        except Exception as e:
            rospy.logerr("Failed to load path from {}: {}".format(filename, str(e)))
            return False
    
    def load_path_service(self, req):
        """Service callback to reload path file"""
        if not self.path_file:
            return TriggerResponse(
                success=False, 
                message="No path file specified. Set ~path_file parameter."
            )
        
        success = self.load_path(self.path_file)
        return TriggerResponse(
            success=success,
            message="Path loaded successfully" if success else "Failed to load path"
        )
    
    def publish_path_visualization(self):
        """Publish path for RViz visualization"""
        from nav_msgs.msg import Path
        
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.waypoints[0].get('frame_id', 'odom')
        
        for wp in self.waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = wp['x']
            pose_stamped.pose.position.y = wp['y']
            pose_stamped.pose.position.z = wp['z']
            pose_stamped.pose.orientation.x = wp['qx']
            pose_stamped.pose.orientation.y = wp['qy']
            pose_stamped.pose.orientation.z = wp['qz']
            pose_stamped.pose.orientation.w = wp['qw']
            path_msg.poses.append(pose_stamped)
        
        self.path_pub.publish(path_msg)
        rospy.loginfo("Published path for visualization ({} waypoints)".format(len(self.waypoints)))
    
    def transform_waypoints_to_current_pose(self):
        """Transform waypoints to start from robot's current pose"""
        if not self.pose_received:
            rospy.logwarn("No current pose received yet, waiting...")
            timeout = rospy.Time.now() + rospy.Duration(5.0)
            while not self.pose_received and rospy.Time.now() < timeout:
                rospy.sleep(0.1)
            if not self.pose_received:
                rospy.logerr("Failed to receive current pose from {}".format(self.odom_topic))
                return False
        
        # Get current robot position and orientation
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_z = self.current_pose.position.z
        
        # Get current robot yaw
        quaternion = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        current_yaw = euler[2]
        
        # Get first waypoint as reference (origin of recorded path)
        first_wp = self.original_waypoints[0]
        first_wp_x = first_wp['x']
        first_wp_y = first_wp['y']
        first_wp_z = first_wp['z']
        
        # Get first waypoint yaw
        first_quat = (
            first_wp['qx'],
            first_wp['qy'],
            first_wp['qz'],
            first_wp['qw']
        )
        first_euler = tf.transformations.euler_from_quaternion(first_quat)
        first_yaw = first_euler[2]
        
        # Calculate transformation: rotation and translation
        yaw_offset = current_yaw - first_yaw
        
        rospy.loginfo("Transforming path to current pose:")
        rospy.loginfo("  Current pose: ({:.2f}, {:.2f}, {:.2f} deg)".format(
            current_x, current_y, math.degrees(current_yaw)))
        rospy.loginfo("  First waypoint: ({:.2f}, {:.2f}, {:.2f} deg)".format(
            first_wp_x, first_wp_y, math.degrees(first_yaw)))
        rospy.loginfo("  Offset: ({:.2f}, {:.2f}, {:.2f} deg)".format(
            current_x - first_wp_x, current_y - first_wp_y, math.degrees(yaw_offset)))
        
        # Transform all waypoints
        self.waypoints = []
        for wp in self.original_waypoints:
            # Translate to origin
            rel_x = wp['x'] - first_wp_x
            rel_y = wp['y'] - first_wp_y
            rel_z = wp['z'] - first_wp_z
            
            # Rotate around Z axis
            rot_x = rel_x * math.cos(yaw_offset) - rel_y * math.sin(yaw_offset)
            rot_y = rel_x * math.sin(yaw_offset) + rel_y * math.cos(yaw_offset)
            
            # Translate to current position
            new_x = rot_x + current_x
            new_y = rot_y + current_y
            new_z = rel_z + current_z
            
            # Transform orientation
            wp_quat = (
                wp['qx'],
                wp['qy'],
                wp['qz'],
                wp['qw']
            )
            wp_euler = tf.transformations.euler_from_quaternion(wp_quat)
            new_yaw = wp_euler[2] + yaw_offset
            new_quat = tf.transformations.quaternion_from_euler(wp_euler[0], wp_euler[1], new_yaw)
            
            # Create transformed waypoint
            new_wp = {
                'x': new_x,
                'y': new_y,
                'z': new_z,
                'qx': new_quat[0],
                'qy': new_quat[1],
                'qz': new_quat[2],
                'qw': new_quat[3],
                'frame_id': wp.get('frame_id', 'map')
            }
            self.waypoints.append(new_wp)
        
        return True
    
    def start_replay(self, req):
        """Service callback to start replaying the path"""
        if len(self.original_waypoints) == 0:
            return TriggerResponse(
                success=False, 
                message="No waypoints loaded! Load a path file first."
            )
        
        if self.is_replaying:
            return TriggerResponse(
                success=False, 
                message="Already replaying a path!"
            )
        
        if not self.move_base_client.wait_for_server(rospy.Duration(10.0)):
            return TriggerResponse(
                success=False,
                message="%s action server not available. Ensure it is running (check 'rosnode list' and 'rostopic list | grep %s')." % (self.move_base_action_name, self.move_base_action_name)
            )
        
        if not self.transform_waypoints_to_current_pose():
            return TriggerResponse(
                success=False,
                message="Failed to get current robot pose for path transformation"
            )
        
        self.is_replaying = True
        self.current_waypoint_index = 0
        
        rospy.loginfo("="*50)
        rospy.loginfo("Started path replay from current pose")
        rospy.loginfo("  Waypoints: {}".format(len(self.waypoints)))
        rospy.loginfo("  Loop mode: {}".format(self.loop_replay))
        rospy.loginfo("="*50)
        
        if self.publish_path:
            self.publish_path_visualization()
        
        self.send_next_goal()
        
        return TriggerResponse(
            success=True, 
            message="Started replaying path with {} waypoints".format(len(self.waypoints))
        )
    
    def stop_replay(self, req):
        """Service callback to stop replaying"""
        if not self.is_replaying:
            return TriggerResponse(
                success=False, 
                message="Not currently replaying!"
            )
        
        self.is_replaying = False
        self.move_base_client.cancel_all_goals()
        
        rospy.loginfo("="*50)
        rospy.loginfo("Stopped path replay")
        rospy.loginfo("  Completed {} / {} waypoints".format(
            self.current_waypoint_index, len(self.waypoints)
        ))
        rospy.loginfo("="*50)
        
        return TriggerResponse(
            success=True, 
            message="Stopped replay at waypoint {}/{}".format(
                self.current_waypoint_index, len(self.waypoints)
            )
        )
    
    def send_next_goal(self):
        """Send the next waypoint as a navigation goal"""
        if not self.is_replaying:
            return
        
        if self.current_waypoint_index >= len(self.waypoints):
            if self.loop_replay:
                rospy.loginfo("Completed path. Looping back to start...")
                self.current_waypoint_index = 0
            else:
                rospy.loginfo("="*50)
                rospy.loginfo("Path replay completed!")
                rospy.loginfo("  Total waypoints: {}".format(len(self.waypoints)))
                rospy.loginfo("="*50)
                self.is_replaying = False
                return
        
        wp = self.waypoints[self.current_waypoint_index]
        
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'  
        goal.target_pose.pose.position.x = wp['x']
        goal.target_pose.pose.position.y = wp['y']
        goal.target_pose.pose.position.z = wp['z']
        goal.target_pose.pose.orientation.x = wp['qx']
        goal.target_pose.pose.orientation.y = wp['qy']
        goal.target_pose.pose.orientation.z = wp['qz']
        goal.target_pose.pose.orientation.w = wp['qw']
        
        rospy.loginfo("Sending goal {}/{}: ({:.2f}, {:.2f}, {:.2f})".format(
            self.current_waypoint_index + 1,
            len(self.waypoints),
            wp['x'], wp['y'], wp['z']
        ))
        
        self.last_goal = goal

        self.move_base_client.send_goal(goal, done_cb=self.goal_done_callback)
    
    def goal_done_callback(self, status, result):
        """Callback when a navigation goal completes"""
        if not self.is_replaying:
            return
        
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached waypoint {}/{}".format(
                self.current_waypoint_index + 1, len(self.waypoints)
            ))
            self.current_waypoint_index += 1
            
            rospy.sleep(0.5)
            self.send_next_goal()
        
        elif status == GoalStatus.ABORTED:
            idx = self.current_waypoint_index
            retries = self._retry_counters.get(idx, 0)

            rospy.logwarn("Goal {}/{} was aborted (retry {}/{}).".format(
                idx + 1, len(self.waypoints), retries, self.abort_retry_count
            ))

            if retries < self.abort_retry_count:
                try:
                    clear_costmaps_service = '/' + self.move_base_action_name + '/clear_costmaps'
                    rospy.wait_for_service(clear_costmaps_service, timeout=2.0)
                    clear_costmaps = rospy.ServiceProxy(clear_costmaps_service, Empty)
                    clear_costmaps()
                    rospy.loginfo('Called %s to attempt recovery', clear_costmaps_service)
                except Exception as e:
                    rospy.logwarn('Could not call clear_costmaps service: {}'.format(e))

                self._retry_counters[idx] = retries + 1
                rospy.sleep(self.abort_retry_delay)
                if self.last_goal is not None:
                    rospy.loginfo('Retrying goal {}/{} (attempt {})'.format(idx+1, len(self.waypoints), retries+1))
                    self.move_base_client.send_goal(self.last_goal, done_cb=self.goal_done_callback)
                    return
                else:
                    rospy.logwarn('No last goal available to retry; skipping to next waypoint')

            rospy.logwarn('Skipping waypoint {}/{} after {} retries'.format(idx+1, len(self.waypoints), self._retry_counters.get(idx, 0)))

            if idx in self._retry_counters:
                del self._retry_counters[idx]
            self.current_waypoint_index += 1
            rospy.sleep(1.0)
            self.send_next_goal()
        
        elif status == GoalStatus.PREEMPTED:
            rospy.logwarn("Goal was preempted (cancelled)")
            self.is_replaying = False
        
        else:
            rospy.logwarn("Goal finished with status: {}".format(status))
            self.current_waypoint_index += 1
            rospy.sleep(1.0)
            self.send_next_goal()
    
    def run(self):
        """Keep the node running"""
        rospy.spin()


if __name__ == '__main__':
    try:
        replay = PathReplay()
        replay.run()
    except rospy.ROSInterruptException:
        pass
