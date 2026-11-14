#!/bin/bash
# Host side of one algorithm run: starts the algorithm's image with common/ros1_session.sh.
# Called by algorithms/<dir>/run.sh, which exports the per-algorithm settings and passes
# its own directory:
#
#   docker_ros1_session.sh <algorithm-dir> <input_bag> <output_dir>
#
# Per-algorithm settings (exported by run.sh):
#   LVIO_SETUP           workspace setup.bash inside the image
#   LVIO_LAUNCH          launch file, relative to the algorithm dir (its config/ is copied into the output)
#   LVIO_MODE            play    = rosbag play the bag into the running algorithm and record its outputs
#                        offline = the algorithm reads the bag file itself and exits when done
#   LVIO_PLAY_TOPICS     topics to play (play mode)
#   LVIO_RECORD_TOPICS   topics to record (play mode)
#   LVIO_OUTPUT_SUBDIRS  folders to create in the output dir before the run (optional)
#   LVIO_RESULT_GLOB     offline mode: result file(s) that must exist afterwards, relative to the output dir
#   LVIO_NEEDS_DISPLAY=1 the algorithm opens GUI windows: without LVIO_RVIZ=1 the session starts Xvfb
#                        (the image must contain xvfb)
#   LVIO_SHUTDOWN_TIMEOUT seconds allowed for a graceful stop / map saving (default 180)
#   LVIO_CONTAINER_HOME  HOME inside the container (optional, default /tmp)
# User settings:
#   LVIO_RVIZ=1          show RViz (needs an X session; runs 'xhost +local:docker')
#   LVIO_PLAY_RATE       rosbag play rate (default 1.0)
#   LVIO_IMAGE           use this image instead of <id>_<distro> from algorithms.conf
set -euo pipefail

usage() {
    echo "Usage: $(basename "${LVIO_RUN_SCRIPT:-$0}") <input_bag> <output_dir>" >&2
    echo "  input_bag   common bag from Step 1 (datasets/COCO/common-ros1/<sequence>.bag)" >&2
    echo "  output_dir  new or empty folder for the results" >&2
    echo "  LVIO_RVIZ=1 to show RViz, LVIO_PLAY_RATE=<rate> to change the playback rate." >&2
    exit 1
}

[[ $# -eq 3 ]] || usage
ALGO_DIR="$(realpath "$1")"
INPUT="$2"
OUTPUT="$3"

# shellcheck source=../algos_lib.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../algos_lib.sh"

row="$(algo_row_for_dir "$(basename "$ALGO_DIR")")"
if [ -z "$row" ]; then
    echo "ERROR: algorithms/$(basename "$ALGO_DIR") is not listed in $ALGOS_CONF" >&2
    exit 1
fi
IFS=$'\t' read -r distro _dir id _input <<< "$row"
IMAGE="${LVIO_IMAGE:-$(image_tag "$id" "$distro")}"

: "${LVIO_SETUP:?run.sh must set LVIO_SETUP}" "${LVIO_LAUNCH:?run.sh must set LVIO_LAUNCH}"
LVIO_MODE="${LVIO_MODE:-play}"
if [ "$LVIO_MODE" = play ]; then
    : "${LVIO_PLAY_TOPICS:?run.sh must set LVIO_PLAY_TOPICS}" "${LVIO_RECORD_TOPICS:?run.sh must set LVIO_RECORD_TOPICS}"
fi

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "ERROR: Docker image '$IMAGE' not found. Build it with build_images_step2/build_images_step2.sh" >&2
    exit 1
fi
if [ ! -f "$INPUT" ]; then
    echo "ERROR: input bag not found: $INPUT" >&2
    exit 1
fi
INPUT="$(realpath "$INPUT")"
if [ -d "$OUTPUT" ] && [ -n "$(ls -A "$OUTPUT")" ]; then
    echo "ERROR: output dir is not empty: $OUTPUT" >&2
    exit 1
fi
mkdir -p "$OUTPUT"
OUTPUT="$(realpath "$OUTPUT")"

# The config used for this run is kept next to its results.
cp -r "$ALGO_DIR/config" "$OUTPUT/config"
for sub in ${LVIO_OUTPUT_SUBDIRS:-}; do
    mkdir -p "$OUTPUT/$sub"
done

# No TTY on purpose: without -t, docker run proxies Ctrl-C (SIGINT) to the container, where
# --init forwards it to ros1_session.sh, which stops the run gracefully. With -it, Ctrl-C would
# only reach whichever command is in the container's foreground at that moment.
DOCKER_ARGS=(--rm --init)
if [ "${LVIO_RVIZ:-0}" = 1 ]; then
    xhost +local:docker >/dev/null
    DOCKER_ARGS+=(-e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

echo "=== $id ($IMAGE) on $(basename "$INPUT") -> $OUTPUT"
docker run "${DOCKER_ARGS[@]}" \
    -e LVIO_ID="$id" \
    -e LVIO_SETUP="$LVIO_SETUP" \
    -e LVIO_LAUNCH="$LVIO_LAUNCH" \
    -e LVIO_MODE="$LVIO_MODE" \
    -e LVIO_PLAY_TOPICS="${LVIO_PLAY_TOPICS:-}" \
    -e LVIO_RECORD_TOPICS="${LVIO_RECORD_TOPICS:-}" \
    -e LVIO_PLAY_RATE="${LVIO_PLAY_RATE:-1.0}" \
    -e LVIO_RVIZ="${LVIO_RVIZ:-0}" \
    -e LVIO_RESULT_GLOB="${LVIO_RESULT_GLOB:-}" \
    -e LVIO_NEEDS_DISPLAY="${LVIO_NEEDS_DISPLAY:-0}" \
    -e LVIO_SHUTDOWN_TIMEOUT="${LVIO_SHUTDOWN_TIMEOUT:-180}" \
    -e LVIO_INPUT_BAG="/lvio/in/$(basename "$INPUT")" \
    -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
    -e HOME="${LVIO_CONTAINER_HOME:-/tmp}" \
    -v "$(dirname "$INPUT")":/lvio/in:ro \
    -v "$OUTPUT":/lvio/out \
    -v "$LVIO_ROOT/common/ros1_session.sh":/lvio/ros1_session.sh:ro \
    "$IMAGE" bash /lvio/ros1_session.sh
