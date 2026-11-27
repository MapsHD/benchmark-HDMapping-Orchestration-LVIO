#!/bin/bash
# Run Coco-LIC on one common bag (see NOTES.md).
# Usage: [LVIO_DATASET=COCO|BUNKER] run.sh <common_bag> <output_dir>   (shows RViz; LVIO_RVIZ=0 to run headless)
# Coco-LIC reads the bag file itself; the TUM trajectory lands in <output_dir>/data/.
HERE="$(cd "$(dirname "$0")" && pwd)"

export LVIO_RUN_SCRIPT="$0"
export LVIO_SETUP=/root/catkin_coco/devel/setup.bash
export LVIO_MODE=offline
case "${LVIO_DATASET:-COCO}" in
    COCO)   export LVIO_LAUNCH=config/coco_lic.launch ;;
    BUNKER) export LVIO_LAUNCH=config/bunker/coco_lic.launch ;;
    *)      echo "ERROR: Coco-LIC has no configuration for dataset '$LVIO_DATASET'" >&2; exit 1 ;;
esac
export LVIO_OUTPUT_SUBDIRS="data"
export LVIO_RESULT_GLOB="data/*.txt"   # the run fails if Coco-LIC wrote no trajectory

exec "$HERE/../../common/docker_ros1_session.sh" "$HERE" "$@"
