#!/bin/bash
# Run COIN-LIO on one common bag (see NOTES.md).
# Usage: run.sh <common_bag> <output_dir>     (LVIO_RVIZ=1 to show RViz)
HERE="$(cd "$(dirname "$0")" && pwd)"

export LVIO_RUN_SCRIPT="$0"
export LVIO_SETUP=/root/catkin_ws/devel/setup.bash
export LVIO_MODE=play
export LVIO_LAUNCH=config/coin_lio.launch
export LVIO_PLAY_TOPICS="/os_cloud_node/points /os_cloud_node/imu"
export LVIO_RECORD_TOPICS="/Odometry /cloud_registered"

exec "$HERE/../../common/docker_ros1_session.sh" "$HERE" "$@"
