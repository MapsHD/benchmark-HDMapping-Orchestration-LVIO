#!/bin/bash
# Run r3live on one common bag (see NOTES.md).
# Usage: run.sh <common_bag> <output_dir>     (shows RViz; LVIO_RVIZ=0 to run headless)
HERE="$(cd "$(dirname "$0")" && pwd)"

export LVIO_RUN_SCRIPT="$0"
export LVIO_SETUP=/root/catkin_ws/devel/setup.bash
export LVIO_MODE=play
export LVIO_LAUNCH=config/r3live.launch
export LVIO_PLAY_TOPICS="/os_cloud_node/points /os_cloud_node/imu /lvio/cam0_rect/compressed"
export LVIO_RECORD_TOPICS="/aft_mapped_to_init /cloud_registered"
# r3live always opens an OpenCV "Control panel" window: headless runs get a virtual X server
export LVIO_NEEDS_DISPLAY=1
# r3live ignores SIGINT (roslaunch would wait the whole default 180 s) and saves nothing on
# shutdown (its RGB map is saved only by pressing 's' in the control panel), so stop it quickly.
export LVIO_SHUTDOWN_TIMEOUT=20

exec "$HERE/../../common/docker_ros1_session.sh" "$HERE" "$@"
