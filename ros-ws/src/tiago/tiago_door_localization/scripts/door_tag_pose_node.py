#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import rospy

from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped

import tf2_ros
import tf2_geometry_msgs

from apriltag_ros.msg import AprilTagDetectionArray


class DoorTagPoseNode(object):
    def __init__(self):
        # Input
        self.in_topic = rospy.get_param("~topics/tag_detections", "/tag_detections")
        self.camera_frame_fallback = rospy.get_param("~frames/camera_frame_fallback", "camera_color_optical_frame")

        # Frames
        self.target_frame_base = rospy.get_param("~frames/base_link", "base_link")
        self.target_frame_map = rospy.get_param("~frames/map", "map")

        # Publishing
        self.publish_base = bool(rospy.get_param("~publish_base", True))
        self.publish_map = bool(rospy.get_param("~tf/publish_map", True))

        self.tf_timeout = float(rospy.get_param("~tf/tf_timeout", 0.1))
        self.max_age_s = float(rospy.get_param("~max_age_s", 0.2))

        # Multi-tag config
        self.tag_ids = rospy.get_param("~apriltag/tag_ids", [int(rospy.get_param("~tag_id", 0))])
        self.tag_ids = [int(x) for x in self.tag_ids]

        raw_side_by_id = rospy.get_param("~apriltag/side_by_id", {})
        self.side_by_id = {}
        for k, v in raw_side_by_id.items():
            try:
                self.side_by_id[int(k)] = str(v)
            except Exception:
                pass

        raw_hinge_side_by_id = rospy.get_param("~apriltag/hinge_side_by_id", {})
        self.hinge_side_by_id = {}
        for k, v in raw_hinge_side_by_id.items():
            try:
                self.hinge_side_by_id[int(k)] = str(v).strip().lower()
            except Exception:
                pass

        self.preferred_side = rospy.get_param("~apriltag/preferred_side", "")  # optional
        self.switch_hysteresis_s = float(rospy.get_param("~apriltag/switch_hysteresis_s", 0.5))

        # Side -> interaction label (push/pull)
        self.interaction_by_side = rospy.get_param("~apriltag/interaction_by_side", {})
        raw_interaction_by_id = rospy.get_param("~apriltag/interaction_by_id", {})
        self.interaction_by_id = {}
        for k, v in raw_interaction_by_id.items():
            try:
                self.interaction_by_id[int(k)] = str(v).strip().lower()
            except Exception:
                pass

        # State
        self._active_tag_id = None
        self._active_side = ""
        self._last_switch_time = rospy.Time(0)
        
        # Sliding average for stable pose
        self._map_pose_samples = []
        self._base_pose_samples = []
        self._max_samples = 12
        self._min_samples = 3
        
        self._map_frame_available = False
        self._map_frame_check_count = 0

        # Publishers
        self.pub_cam = rospy.Publisher("/door/tag_pose_camera", PoseStamped, queue_size=10)
        self.pub_base = rospy.Publisher("/door/tag_pose_base", PoseStamped, queue_size=10)
        self.pub_map = rospy.Publisher("/door/tag_pose_map", PoseStamped, queue_size=10)

        self.pub_tag_id = rospy.Publisher("/door/tag_id", Int32, queue_size=10)
        self.pub_side = rospy.Publisher("/door/door_side", String, queue_size=10)
        self.pub_interaction = rospy.Publisher("/door/interaction", String, queue_size=10)
        self.pub_hinge_side = rospy.Publisher("/door/hinge_side", String, queue_size=10)

        self.pub_dbg = rospy.Publisher("~debug", String, queue_size=10)

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub = rospy.Subscriber(self.in_topic, AprilTagDetectionArray, self.detection_callback, queue_size=1)

        rospy.loginfo("door_tag_pose_node: listening %s, tag_ids=%s", self.in_topic, str(self.tag_ids))
        rospy.loginfo("door_tag_pose_node: side_by_id=%s", str(self.side_by_id))
        rospy.loginfo("door_tag_pose_node: hinge_side_by_id=%s", str(self.hinge_side_by_id))
        rospy.loginfo("door_tag_pose_node: interaction_by_side=%s", str(self.interaction_by_side))
        rospy.loginfo("door_tag_pose_node: interaction_by_id=%s", str(self.interaction_by_id))
        
        if self.publish_map:
            rospy.sleep(1.0)
            if not self._check_frame_exists(self.target_frame_map):
                rospy.logwarn("=" * 80)
                rospy.logwarn("Map frame '%s' not yet available!", self.target_frame_map)
                rospy.logwarn("This is normal if SLAM (RTAB-Map) is still initializing.")
                rospy.logwarn("Door poses will be published in base_link until map frame becomes available.")
                rospy.logwarn("=" * 80)
            else:
                rospy.loginfo("Map frame '%s' is available. Door mapping enabled.", self.target_frame_map)

    def _check_frame_exists(self, frame_id):
        """Check if a frame exists in the TF tree"""
        try:
            all_frames = self.tf_buffer.all_frames_as_string()
            return frame_id in all_frames
        except Exception:
            return False

    def _average_pose(self, samples):
        """Compute average of pose samples (position and orientation)"""
        from geometry_msgs.msg import Pose
        import numpy as np
        
        if not samples:
            return None
        
        avg_pose = Pose()
        
        # Average position
        avg_pose.position.x = sum(p.position.x for p in samples) / len(samples)
        avg_pose.position.y = sum(p.position.y for p in samples) / len(samples)
        avg_pose.position.z = sum(p.position.z for p in samples) / len(samples)
        
        # Average orientation (simple mean, normalized)
        avg_pose.orientation.x = sum(p.orientation.x for p in samples) / len(samples)
        avg_pose.orientation.y = sum(p.orientation.y for p in samples) / len(samples)
        avg_pose.orientation.z = sum(p.orientation.z for p in samples) / len(samples)
        avg_pose.orientation.w = sum(p.orientation.w for p in samples) / len(samples)
        
        # Normalize quaternion
        qlen = (avg_pose.orientation.x**2 + avg_pose.orientation.y**2 + 
                avg_pose.orientation.z**2 + avg_pose.orientation.w**2) ** 0.5
        if qlen > 0:
            avg_pose.orientation.x /= qlen
            avg_pose.orientation.y /= qlen
            avg_pose.orientation.z /= qlen
            avg_pose.orientation.w /= qlen
        
        return avg_pose

    def _transform_pose(self, pose_in, target_frame):
        src = pose_in.header.frame_id
        if not src:
            raise RuntimeError("Input pose has empty header.frame_id")
        if not self._check_frame_exists(target_frame):
            raise RuntimeError("Target frame '%s' does not exist in TF tree (SLAM may not be ready)" % target_frame)

        stamp = pose_in.header.stamp
        if stamp == rospy.Time():
            stamp = rospy.Time.now()
            
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, src, stamp, rospy.Duration(2.0)
            )
        except Exception as e:
            rospy.logwarn_throttle(5.0, "TF lookup at stamp %.3f failed (%s). Falling back to latest." % (stamp.to_sec(), str(e)))
            try:
                tf = self.tf_buffer.lookup_transform(
                    target_frame, src, rospy.Time(0), rospy.Duration(self.tf_timeout)
                )
            except Exception as e2:
                rospy.logerr("Both timestamped and latest TF failed: %s" % str(e2))
                raise

        pose_out = tf2_geometry_msgs.do_transform_pose(pose_in, tf)
        pose_out.header.frame_id = target_frame
        pose_out.header.stamp = stamp
        
        rospy.loginfo_throttle(2.0, "Transform %s->%s: in=(%.3f,%.3f,%.3f) out=(%.3f,%.3f,%.3f)" % 
                               (src, target_frame,
                                pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z,
                                pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z))
        
        return pose_out

    def _choose_detection(self, msg):
        """
        Choose which tag detection to use.
        Strategy:
          1) collect all detections for tag_ids
          2) if both sides visible and preferred_side set, choose that side
          3) else keep current active side unless hysteresis elapsed
          4) else choose first in priority order
        Returns: (det, tag_id) or (None, None)
        """
        det_by_id = {}
        for det in msg.detections:
            if len(det.id) == 0:
                continue
            tid = int(det.id[0])
            if tid in self.tag_ids:
                det_by_id[tid] = det

        if not det_by_id:
            return None, None

        if self.preferred_side:
            for tid, det in det_by_id.items():
                side = self.side_by_id.get(tid, "")
                if side == self.preferred_side:
                    return det, tid

        now = rospy.Time.now()
        can_switch = (now - self._last_switch_time).to_sec() >= self.switch_hysteresis_s

        if self._active_tag_id in det_by_id and not can_switch:
            return det_by_id[self._active_tag_id], self._active_tag_id

        if self._active_tag_id in det_by_id:
            return det_by_id[self._active_tag_id], self._active_tag_id

        for tid in self.tag_ids:
            if tid in det_by_id:
                return det_by_id[tid], tid

        any_tid = list(det_by_id.keys())[0]
        return det_by_id[any_tid], any_tid

    def detection_callback(self, msg):
        time_now = rospy.Time.now()

        chosen, tid = self._choose_detection(msg)
        if chosen is None:
            self.pub_dbg.publish(String(data="no_tag_any_of_%s" % str(self.tag_ids)))
            return

        p = PoseStamped()
        p.header = chosen.pose.header
        if not p.header.frame_id:
            p.header.frame_id = self.camera_frame_fallback
        
        age = (time_now - p.header.stamp).to_sec() if p.header.stamp != rospy.Time() else 0.0
        p.header.stamp = time_now
        
        p.pose = chosen.pose.pose.pose
        if age > self.max_age_s:
            self.pub_dbg.publish(String(data="stale age=%.3fs" % age))
            return

        if tid != self._active_tag_id:
            self._active_tag_id = tid
            self._active_side = self.side_by_id.get(tid, "")
            self._last_switch_time = time_now

        self.pub_tag_id.publish(Int32(data=int(tid)))
        self.pub_side.publish(String(data=self._active_side))
        hinge_side = str(self.hinge_side_by_id.get(tid, "")).strip().lower()
        self.pub_hinge_side.publish(String(data=hinge_side))

        interaction = ""
        if tid in self.interaction_by_id:
            interaction = self.interaction_by_id[tid]
        elif self._active_side and isinstance(self.interaction_by_side, dict):
            interaction = str(self.interaction_by_side.get(self._active_side, ""))
        self.pub_interaction.publish(String(data=interaction))

        self.pub_cam.publish(p)

        ok = ["cam"]

        if self.publish_base:
            try:
                pb = self._transform_pose(p, self.target_frame_base)
                
                self._base_pose_samples.append(pb.pose)
                if len(self._base_pose_samples) > self._max_samples:
                    self._base_pose_samples.pop(0)
                
                if len(self._base_pose_samples) >= self._min_samples:
                    pb.pose = self._average_pose(self._base_pose_samples)
                    ok.append("base(avg=%d)" % len(self._base_pose_samples))
                else:
                    ok.append("base(raw)")
                
                self.pub_base.publish(pb)
            except Exception as e:
                self.pub_dbg.publish(String(data="base_tf_failed: %s" % str(e)))

        if self.publish_map:
            try:
                pm = self._transform_pose(p, self.target_frame_map)
                
                self._map_pose_samples.append(pm.pose)
                if len(self._map_pose_samples) > self._max_samples:
                    self._map_pose_samples.pop(0)
                
                if len(self._map_pose_samples) >= self._min_samples:
                    pm.pose = self._average_pose(self._map_pose_samples)
                    ok.append("map(avg=%d)" % len(self._map_pose_samples))
                else:
                    ok.append("map(raw)")
                
                self.pub_map.publish(pm)
                    
            except Exception as e:
                if "does not exist" in str(e):
                    self._map_frame_check_count += 1
                    if self._map_frame_check_count == 1:
                        rospy.logwarn("Map frame not yet available - waiting for SLAM to initialize...")
                    rospy.logwarn_throttle(10.0, "Still waiting for map frame (SLAM initializing). Door poses only in base_link for now.")
                else:
                    rospy.logwarn_throttle(5.0, "Map TF transform failed: %s" % str(e))
                self.pub_dbg.publish(String(data="map_tf_failed: %s" % str(e)))

        self.pub_dbg.publish(String(
            data="ok tid=%d side=%s hinge_side=%s interaction=%s " % (tid, self._active_side, hinge_side, interaction) + "+".join(ok)
        ))


def main():
    rospy.init_node("door_tag_pose_node", anonymous=False)
    _ = DoorTagPoseNode()
    rospy.spin()


if __name__ == "__main__":
    main()
