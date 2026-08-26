#!/bin/bash
DEPTH_PROFILE="$1"
COLOR_PROFILE="$2"
LABEL="$3"
TRIAL="$4"
DURATION="${5:-8}"
OUTDIR="/tmp/rawhz"
mkdir -p "$OUTDIR"

source /opt/ros/jazzy/setup.bash
source /home/auto/ros2_ws/install/setup.bash

pkill -f realsense2_camera_node 2>/dev/null
sleep 2

LAUNCHLOG="$OUTDIR/${LABEL}_trial${TRIAL}_launch.log"
nohup ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true \
  depth_module.depth_profile:="$DEPTH_PROFILE" \
  rgb_camera.color_profile:="$COLOR_PROFILE" \
  > "$LAUNCHLOG" 2>&1 &

for i in $(seq 1 25); do
  grep -q "RealSense Node Is Up" "$LAUNCHLOG" 2>/dev/null && break
  sleep 1
done

if ! grep -q "RealSense Node Is Up" "$LAUNCHLOG" 2>/dev/null; then
  echo "NODE FAILED TO START for $LABEL trial $TRIAL" >&2
  exit 1
fi

sleep 3

date -u +"%Y-%m-%dT%H:%M:%SZ" > "$OUTDIR/${LABEL}_trial${TRIAL}_color_hz.txt"
echo "COMMAND: timeout ${DURATION} ros2 topic hz /camera/camera/color/image_raw" >> "$OUTDIR/${LABEL}_trial${TRIAL}_color_hz.txt"
echo "-----RAW OUTPUT BELOW-----" >> "$OUTDIR/${LABEL}_trial${TRIAL}_color_hz.txt"
timeout "$DURATION" ros2 topic hz /camera/camera/color/image_raw >> "$OUTDIR/${LABEL}_trial${TRIAL}_color_hz.txt" 2>&1

date -u +"%Y-%m-%dT%H:%M:%SZ" > "$OUTDIR/${LABEL}_trial${TRIAL}_depth_hz.txt"
echo "COMMAND: timeout ${DURATION} ros2 topic hz /camera/camera/depth/image_rect_raw" >> "$OUTDIR/${LABEL}_trial${TRIAL}_depth_hz.txt"
echo "-----RAW OUTPUT BELOW-----" >> "$OUTDIR/${LABEL}_trial${TRIAL}_depth_hz.txt"
timeout "$DURATION" ros2 topic hz /camera/camera/depth/image_rect_raw >> "$OUTDIR/${LABEL}_trial${TRIAL}_depth_hz.txt" 2>&1

pkill -f realsense2_camera_node 2>/dev/null
sleep 2
