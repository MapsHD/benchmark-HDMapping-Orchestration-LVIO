#!/bin/bash
# Run FAST-LIVO2 on one common bag (see NOTES.md).
# Usage: run.sh <common_bag> <output_dir>     (LVIO_RVIZ=1 to show RViz)
HERE="$(cd "$(dirname "$0")" && pwd)"

export LVIO_RUN_SCRIPT="$0"
export LVIO_SETUP=/root/catkin_ws/devel/setup.bash
export LVIO_MODE=play
export LVIO_LAUNCH=config/fast_livo2.launch
export LVIO_PLAY_TOPICS="/os_cloud_node/points /alphasense_driver_ros/imu /alphasense_driver_ros/cam0/compressed"
export LVIO_RECORD_TOPICS="/aft_mapped_to_init /cloud_registered"
export LVIO_OUTPUT_SUBDIRS="result"                # Log/result/ in the image links here (Dockerfile)
export LVIO_RESULT_GLOB="result/trajectory.txt"   # native TUM trajectory (config/coco.yaml, evo:)

exec "$HERE/../../common/docker_ros1_session.sh" "$HERE" "$@"
