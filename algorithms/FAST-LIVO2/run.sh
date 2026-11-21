#!/bin/bash
# Run FAST-LIVO2 on one common bag (see NOTES.md).
# Usage: [LVIO_DATASET=COCO|BUNKER] run.sh <common_bag> <output_dir>   (shows RViz; LVIO_RVIZ=0 to run headless)
HERE="$(cd "$(dirname "$0")" && pwd)"

export LVIO_RUN_SCRIPT="$0"
export LVIO_SETUP=/root/catkin_ws/devel/setup.bash
export LVIO_MODE=play
case "${LVIO_DATASET:-COCO}" in
    COCO)   export LVIO_LAUNCH=config/fast_livo2.launch
            export LVIO_PLAY_TOPICS="/os_cloud_node/points /alphasense_driver_ros/imu /alphasense_driver_ros/cam0/compressed" ;;
    BUNKER) export LVIO_LAUNCH=config/bunker/fast_livo2.launch
            export LVIO_PLAY_TOPICS="/lvio/livox/lidar /livox/imu /lvio/d455_rect/compressed" ;;
    *)      echo "ERROR: FAST-LIVO2 has no configuration for dataset '$LVIO_DATASET'" >&2; exit 1 ;;
esac
export LVIO_RECORD_TOPICS="/aft_mapped_to_init /cloud_registered"
export LVIO_OUTPUT_SUBDIRS="result"                # Log/result/ in the image links here (Dockerfile)
export LVIO_RESULT_GLOB="result/trajectory.txt"   # native TUM trajectory (config/coco.yaml, evo:)

exec "$HERE/../../common/docker_ros1_session.sh" "$HERE" "$@"
