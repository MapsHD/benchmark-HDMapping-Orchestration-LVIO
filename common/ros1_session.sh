#!/bin/bash
# Container side of one algorithm run (mounted at /lvio/ros1_session.sh by docker_ros1_session.sh).
#
#   /lvio/in   input bag directory (read-only)
#   /lvio/out  output directory: config/ (copied by the host), logs/, recorded-<id>.bag, algorithm files
#
# play mode:    roslaunch -> rosbag record (outputs) -> rosbag play --clock (inputs)
#               -> drain -> stop recorder -> SIGINT roslaunch (algorithms save maps on shutdown)
# offline mode: roslaunch only; the algorithm reads the bag itself and exits when done.
# Every launch file takes the args rviz, input_bag and out_dir.
#
# Exit status: 0 = ok; 1 = failed (see the [lvio] lines and logs/); 130 = interrupted.
# A run fails when the bag cannot be played, the algorithm exits early, nothing was recorded
# (play mode) or the expected result file is missing (offline mode, LVIO_RESULT_GLOB).
#
# Ctrl-C / docker stop: the container gets SIGINT/SIGTERM (docker run without a TTY proxies
# signals; --init forwards them to this script). Stops the bag, closes the recorded bag, lets the
# algorithm save its map, then exits 130. A second Ctrl-C aborts without waiting.
set -u
# Job control: without it, background jobs of a non-interactive shell start with SIGINT
# ignored, and the graceful stops below (kill -INT) would never reach roslaunch or the recorder.
set -m

OUT="${LVIO_OUT:-/lvio/out}"
LOGS="$OUT/logs"
mkdir -p "$LOGS"

export ROS_HOME=/tmp/.ros
export ROS_LOG_DIR="$LOGS/ros"
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

MODE="${LVIO_MODE:-play}"
RATE="${LVIO_PLAY_RATE:-1.0}"
STARTUP_DELAY="${LVIO_STARTUP_DELAY:-5}"        # s between launch and recording
DRAIN_SECONDS="${LVIO_DRAIN_SECONDS:-10}"       # s to let the algorithm finish after the bag ends
SHUTDOWN_TIMEOUT="${LVIO_SHUTDOWN_TIMEOUT:-180}" # s allowed for map saving on shutdown
RVIZ=false
[ "${LVIO_RVIZ:-0}" = 1 ] && RVIZ=true

LAUNCH_FILE="$OUT/$LVIO_LAUNCH"
LAUNCH_ARGS=("rviz:=$RVIZ" "input_bag:=$LVIO_INPUT_BAG" "out_dir:=$OUT")
RECORD_BAG="$OUT/recorded-$LVIO_ID.bag"

log() { echo "[lvio] $*"; }

# nap <seconds>: sleep in the background and wait, so INT/TERM traps fire at once
# (bash defers traps until a foreground command ends).
nap() { sleep "$1" & wait $!; }

# shellcheck disable=SC1090
source "$LVIO_SETUP"

# roslaunch escalates SIGINT to SIGTERM after 15 s by default, which can cut map saving
# short (LVI-SAM, r3live save on shutdown). Raise it where roslaunch supports the option.
if roslaunch --help 2>/dev/null | grep -q -- '--sigint-timeout'; then
    LAUNCH_ARGS=(--sigint-timeout "$SHUTDOWN_TIMEOUT" "${LAUNCH_ARGS[@]}")
fi

LAUNCH_PID=""
REC_PID=""
PLAY_PID=""
XVFB_PID=""

# stop_pid <pid> <signal> <timeout-s>
stop_pid() {
    local pid="$1" sig="$2" timeout="$3" i
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null || return 0
    kill -"$sig" "$pid" 2>/dev/null
    for ((i = 0; i < timeout; i++)); do
        kill -0 "$pid" 2>/dev/null || { wait "$pid" 2>/dev/null; return 0; }
        sleep 1
    done
    log "process $pid did not stop after ${timeout}s, killing it"
    kill -9 "$pid" 2>/dev/null
    wait "$pid" 2>/dev/null
}

# shellcheck disable=SC2329  # invoked by the EXIT trap
cleanup() {
    trap - EXIT INT TERM
    stop_pid "$PLAY_PID" INT 10
    stop_pid "$REC_PID" INT 30
    stop_pid "$LAUNCH_PID" INT $((SHUTDOWN_TIMEOUT + 30))  # after roslaunch's own SIGINT->SIGTERM escalation
    stop_pid "$XVFB_PID" TERM 5
    chown -R "${HOST_UID:-0}:${HOST_GID:-0}" "$OUT" 2>/dev/null || true
}
trap cleanup EXIT
trap 'log "interrupted: stopping gracefully (Ctrl-C again to abort)"; exit 130' INT TERM

if [ ! -f "$LAUNCH_FILE" ]; then
    log "ERROR: launch file not found: $LAUNCH_FILE"
    exit 1
fi

# Algorithms that open GUI windows (LVIO_NEEDS_DISPLAY=1, e.g. r3live's OpenCV control panel)
# crash without a display. Without a real one (--rviz / LVIO_RVIZ=1), use a virtual X server.
if [ "${LVIO_NEEDS_DISPLAY:-0}" = 1 ] && [ -z "${DISPLAY:-}" ]; then
    if ! command -v Xvfb >/dev/null; then
        log "FAILED: this algorithm needs a display but the image has no Xvfb (rebuild it, or use --rviz)"
        exit 1
    fi
    Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp < /dev/null > "$LOGS/xvfb.log" 2>&1 &
    XVFB_PID=$!
    for ((i = 0; i < 20; i++)); do
        [ -e /tmp/.X11-unix/X99 ] && break
        nap 0.5
    done
    export DISPLAY=:99
    log "started virtual X server (Xvfb $DISPLAY) for the algorithm's GUI windows"
fi

# ---------------------------------------------------------------- offline mode
if [ "$MODE" = offline ]; then
    log "offline run: $LAUNCH_FILE"
    roslaunch "$LAUNCH_FILE" "${LAUNCH_ARGS[@]}" < /dev/null > >(tee "$LOGS/launch.log") 2>&1 &
    LAUNCH_PID=$!
    wait "$LAUNCH_PID"
    status=$?
    LAUNCH_PID=""
    log "algorithm finished with status $status"
    if [ -n "${LVIO_RESULT_GLOB:-}" ]; then
        results=()
        for f in "$OUT"/$LVIO_RESULT_GLOB; do
            [ -s "$f" ] && results+=("$f")
        done
        if [ ${#results[@]} -eq 0 ]; then
            log "FAILED: no non-empty result file matches $LVIO_RESULT_GLOB"
            exit 1
        fi
        for f in "${results[@]}"; do
            log "result: ${f#"$OUT"/} ($(wc -l < "$f") lines)"
        done
    fi
    [ "$status" -eq 0 ] || { log "FAILED: algorithm exit status $status, see logs/launch.log"; exit 1; }
    exit 0
fi

# ---------------------------------------------------------------- play mode
log "starting $LAUNCH_FILE (log: logs/launch.log)"
roslaunch "$LAUNCH_FILE" "${LAUNCH_ARGS[@]}" < /dev/null > "$LOGS/launch.log" 2>&1 &
LAUNCH_PID=$!

for ((i = 0; i < 60; i++)); do
    rosparam get /run_id >/dev/null 2>&1 && break
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        log "FAILED: roslaunch exited during startup, see logs/launch.log"
        tail -n 30 "$LOGS/launch.log"
        exit 1
    fi
    nap 1
done
nap "$STARTUP_DELAY"

read -r -a RECORD_TOPICS <<< "$LVIO_RECORD_TOPICS"
read -r -a PLAY_TOPICS <<< "$LVIO_PLAY_TOPICS"

log "recording ${RECORD_TOPICS[*]} -> $(basename "$RECORD_BAG") (log: logs/record.log)"
rosrun rosbag record -O "$RECORD_BAG" "${RECORD_TOPICS[@]}" __name:=lvio_recorder < /dev/null > "$LOGS/record.log" 2>&1 &
REC_PID=$!
nap 3

log "playing $(basename "$LVIO_INPUT_BAG") at rate $RATE (log: logs/play.log): ${PLAY_TOPICS[*]}"
# The rosbag play binary (rosrun) returns non-zero on errors; the 'rosbag play' Python wrapper
# returns 0 even after a [FATAL]. Background + wait, so Ctrl-C / docker stop act at once.
# stdin from /dev/null: a background rosbag play would otherwise read the terminal.
rosrun rosbag play --clock -r "$RATE" "$LVIO_INPUT_BAG" --topics "${PLAY_TOPICS[@]}" \
    < /dev/null > >(tee "$LOGS/play.log") 2>&1 &
PLAY_PID=$!
wait "$PLAY_PID"
play_status=$?
PLAY_PID=""

log "bag finished, draining for ${DRAIN_SECONDS}s"
nap "$DRAIN_SECONDS"   # also lets tee finish writing logs/play.log

failures=()
if [ "$play_status" -ne 0 ] || grep -q -E '\[FATAL\]|ROSLZ4_|BagException|BagIOException|BagFormatException' "$LOGS/play.log"; then
    failures+=("rosbag play failed (exit $play_status), see logs/play.log")
fi
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    failures+=("the algorithm exited before the end of the bag, see logs/launch.log")
fi

log "stopping recorder"
rosnode kill /lvio_recorder >/dev/null 2>&1
stop_pid "$REC_PID" INT 30
REC_PID=""

log "stopping algorithm (up to ${SHUTDOWN_TIMEOUT}s for map saving)"
stop_pid "$LAUNCH_PID" INT $((SHUTDOWN_TIMEOUT + 30))  # after roslaunch's own SIGINT->SIGTERM escalation
LAUNCH_PID=""

recorded=0
if [ -f "$RECORD_BAG" ]; then
    recorded="$(rosbag info --yaml "$RECORD_BAG" 2>/dev/null | awk '/^messages:/ { print $2 }')"
    recorded="${recorded:-0}"
    log "recorded $recorded messages ($(du -h "$RECORD_BAG" | cut -f1)) in $(basename "$RECORD_BAG")"
    rosbag info "$RECORD_BAG" 2>/dev/null | sed -n '/^topics:/,$p' | tee "$LOGS/recorded_topics.txt"
fi
if [ "$recorded" -eq 0 ]; then
    failures+=("no output messages were recorded")
fi

if [ ${#failures[@]} -gt 0 ]; then
    for f in "${failures[@]}"; do
        log "FAILED: $f"
    done
    exit 1
fi
log "OK"
exit 0
