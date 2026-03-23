#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function, division
import numpy as np

from geometry_msgs.msg import PoseStamped

from .utils import pose_stamped, yaw_from_quat


class DoorModel(object):
    """Simple planar door model around hinge (map frame).

    Assumptions:
      - hinge_pose defines hinge position and yaw (hinge frame x-axis).
      - door rotates around hinge about +Z.
      - handle is at distance handle_offset from hinge along door plane.
      - handle_height is the absolute height of the handle in the map frame.
    """

    def __init__(self, door_width=None, handle_offset_from_hinge=None, handle_height=None):
        self.door_width = door_width
        self.handle_offset_from_hinge = handle_offset_from_hinge
        self.handle_height = handle_height if handle_height is not None else 1.0

    def _hinge_xy(self, hinge_pose):
        """
        Extract hinge planar position from PoseStamped.
        """
        hx = hinge_pose.pose.position.x
        hy = hinge_pose.pose.position.y
        return hx, hy

    def _hinge_yaw(self, hinge_pose):
        """
        Extract hinge yaw from PoseStamped orientation.
        """
        return yaw_from_quat(hinge_pose.pose.orientation)

    def _door_yaw(self, hinge_yaw, door_angle_rad, opening_sign):
        """
        Compute current door yaw from hinge yaw and opening direction.
        """
        return hinge_yaw + float(opening_sign) * door_angle_rad

    def _handle_radius(self):
        """
        Return configured handle offset from hinge.
        """
        return self.handle_offset_from_hinge

    def _handle_xy_from_door_yaw(self, hx, hy, door_yaw):
        """
        Compute handle planar position for a given door yaw.
        """
        r = self._handle_radius()
        x = hx + r * np.cos(door_yaw)
        y = hy + r * np.sin(door_yaw)
        return x, y

    def _build_handle_pose(self, frame_id, x, y, door_yaw):
        """
        Build PoseStamped for handle target pose.
        """
        return pose_stamped(
            frame_id=frame_id,
            x=x,
            y=y,
            yaw=door_yaw,
            z=self.handle_height
        )

    def handle_pose_from_hinge(self, hinge_pose, door_angle_rad, frame_id, opening_sign=1.0):
        """
        Compute handle pose from hinge pose and current door angle.
        """
        hx, hy = self._hinge_xy(hinge_pose)
        hinge_yaw = self._hinge_yaw(hinge_pose)
        door_yaw = self._door_yaw(hinge_yaw, door_angle_rad, opening_sign)

        x, y = self._handle_xy_from_door_yaw(hx, hy, door_yaw)

        return self._build_handle_pose(frame_id, x, y, door_yaw)