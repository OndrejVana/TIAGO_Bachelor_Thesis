#!/usr/bin/env bash
# record_bag.sh — record rosbag for door localization & planning tests
# Usage: ./record_bag.sh [label] [--door-loc]

LABEL="door_test"
DOOR_LOC=false

for arg in "$@"; do
  case "$arg" in
    --door-loc) DOOR_LOC=true ;;
    *) LABEL="$arg" ;;
  esac
done

TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)
BAG_DIR="$(dirname "$0")/bags"
BAG_NAME="${LABEL}_${TIMESTAMP}"

mkdir -p "$BAG_DIR"

TOPICS="/tf /tf_static \
  /xtion/rgb/image_rect_color /xtion/rgb/camera_info \
  /xtion/depth_registered/image_raw /xtion/depth_registered/camera_info \
  /tag_detections \
  /mobile_base_controller/odom /mobile_base_controller/cmd_vel \
  /scan /map \
  /joint_states \
  /arm_right_controller/follow_joint_trajectory/goal \
  /arm_right_controller/follow_joint_trajectory/result \
  /arm_left_controller/follow_joint_trajectory/goal \
  /arm_left_controller/follow_joint_trajectory/result \
  /torso_controller/follow_joint_trajectory/goal"

if [ "$DOOR_LOC" = true ]; then
  TOPICS="$TOPICS \
    /door/tag_pose_camera /door/tag_pose_base /door/tag_pose_map \
    /door/hinge_pose_map /door/handle_pose_map \
    /door/hinge_axis_map /door/plane_map \
    /door/door_side /door/hinge_side /door/interaction /door/tag_id"
  echo "[record_bag] Door localization topics: ON"
else
  echo "[record_bag] Door localization topics: OFF (pass --door-loc to enable)"
fi

echo "[record_bag] Recording to: $BAG_DIR/$BAG_NAME.bag"
echo "[record_bag] Press Ctrl+C to stop."

rosbag record -O "$BAG_DIR/$BAG_NAME" $TOPICS

echo "[record_bag] Saved: $BAG_DIR/$BAG_NAME.bag ($(du -h "$BAG_DIR/$BAG_NAME.bag" | cut -f1))"
