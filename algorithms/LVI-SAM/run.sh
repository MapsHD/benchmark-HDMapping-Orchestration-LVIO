#!/bin/bash
# Run LVI-SAM on one common bag (see NOTES.md).
# Usage: [LVIO_DATASET=COCO|BUNKER] run.sh <common_bag> <output_dir>   (shows RViz; LVIO_RVIZ=0 to run headless)
HERE="$(cd "$(dirname "$0")" && pwd)"

export LVIO_RUN_SCRIPT="$0"
export LVIO_SETUP=/root/catkin_ws/devel/setup.bash
export LVIO_MODE=play
case "${LVIO_DATASET:-COCO}" in
    COCO)   export LVIO_LAUNCH=config/lvi_sam.launch
            export LVIO_PLAY_TOPICS="/lvio/points_xyzirt /alphasense_driver_ros/imu /alphasense_driver_ros/cam0/compressed" ;;
    BUNKER) export LVIO_LAUNCH=config/bunker/lvi_sam.launch
            export LVIO_PLAY_TOPICS="/lvio/points_xyzirt /livox/imu /lvio/d455_rect/compressed" ;;
    *)      echo "ERROR: LVI-SAM has no configuration for dataset '$LVIO_DATASET'" >&2; exit 1 ;;
esac
export LVIO_RECORD_TOPICS="/lvi_sam/lidar/mapping/odometry /lvi_sam/lidar/mapping/path /lvi_sam/lidar/mapping/cloud_registered"
# savePCD writes to $HOME/lvi_sam_map/ on shutdown: HOME = output dir
export LVIO_CONTAINER_HOME=/lvio/out

exec "$HERE/../../common/docker_ros1_session.sh" "$HERE" "$@"
