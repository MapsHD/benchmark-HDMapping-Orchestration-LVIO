#!/bin/bash
# Step 4: export every finished run's trajectory (TUM) and map (PLY) (see README.md, Step 4).
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=../../algos_lib.sh
source "$SCRIPT_DIR/../../algos_lib.sh"

IMAGE_NAME="lvio-prepare"   # Python + rosbags + numpy + OpenCV, built from steps/1_prepare_data/
DATA_DIR="$LVIO_ROOT/datasets/COCO"
COMMON_DIR=""
RESULTS_DIR="$LVIO_ROOT/results"
VOXEL_SIZE="0.05"
JOBS=1
FORCE=0

usage() {
    cat <<EOF
Usage: $0 [--results-dir DIR] [--data-dir DIR] [--common-dir DIR] [--voxel-size METERS] [--jobs N] [--force]

Exports each finished run as described by algorithms/<dir>/export.yaml:
  <results-dir>/<sequence>/<algorithm-id>/  ->  .../<algorithm-id>/export/
      <id>_trajectory_tum.txt, <id>_map.ply, <id>_preview.png, export.json

  --results-dir DIR    results of Step 3                          (default: results)
  --data-dir DIR       dataset directory                          (default: datasets/COCO)
  --common-dir DIR     common ROS 1 bags (maps rebuilt from scans) (default: <data-dir>/common-ros1)
  --voxel-size METERS  map voxel size, 0 = keep every point       (default: 0.05)
  --jobs N             exports run at the same time               (default: 1)
  --force              redo existing exports                      (default: skip them)

Set ONLY_ALGOS="r3live lvi-sam" and/or ONLY_SEQUENCES="stairs" to restrict the exports.
Runs whose latest status in run_summary.csv is not 'ok' are skipped.
Every export appends a line to <results-dir>/export_summary.csv (status ok | failed | skipped).
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --results-dir) RESULTS_DIR="${2:?--results-dir needs a value}"; shift 2 ;;
        --data-dir)    DATA_DIR="${2:?--data-dir needs a value}"; shift 2 ;;
        --common-dir)  COMMON_DIR="${2:?--common-dir needs a value}"; shift 2 ;;
        --voxel-size)  VOXEL_SIZE="${2:?--voxel-size needs a value}"; shift 2 ;;
        --jobs)        JOBS="${2:?--jobs needs a value}"; shift 2 ;;
        --force)       FORCE=1; shift ;;
        -h|--help)     usage; exit 0 ;;
        *)             echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 1 ;;
    esac
done

if ! [[ "$VOXEL_SIZE" =~ ^[0-9]*\.?[0-9]+$ ]]; then
    echo "ERROR: --voxel-size must be a non-negative number of meters, got '$VOXEL_SIZE'" >&2; exit 1
fi
if ! [[ "$JOBS" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: --jobs must be a positive integer, got '$JOBS'" >&2; exit 1
fi
check_only_algos
check_only_sequences

DATA_DIR="$(realpath -m "$DATA_DIR")"
COMMON_DIR="$(realpath -m "${COMMON_DIR:-$DATA_DIR/common-ros1}")"
RESULTS_DIR="$(realpath -m "$RESULTS_DIR")"
if [ ! -d "$RESULTS_DIR" ]; then
    echo "ERROR: no results directory $RESULTS_DIR (run Step 3)" >&2; exit 1
fi
RUN_SUMMARY="$RESULTS_DIR/run_summary.csv"
SUMMARY="$RESULTS_DIR/export_summary.csv"
[ -f "$SUMMARY" ] || echo "finished_at,sequence,algorithm,status,seconds,output" > "$SUMMARY"

echo "Building helper image '$IMAGE_NAME'..."
docker build -q -t "$IMAGE_NAME" "$LVIO_ROOT/steps/1_prepare_data" >/dev/null || exit 1

# last_run_status <sequence> <id>: status of the latest Step 3 run (empty if none is listed)
last_run_status() {
    [ -f "$RUN_SUMMARY" ] || return 0
    awk -F, -v s="$1" -v a="$2" '$2 == s && $3 == a { st = $4 } END { print st }' "$RUN_SUMMARY"
}

record() {  # record <sequence> <id> <status> <seconds> <output>
    echo "$(date -Iseconds),$1,$2,$3,$4,$5" >> "$SUMMARY"
}

# export_one <sequence> <dir> <id>: one export in the helper image; output in <run>/logs/export.log
export_one() {
    local seq="$1" dir="$2" id="$3"
    local run="$RESULTS_DIR/$seq/$id" start=$SECONDS rc status
    local cmd=(docker run --rm --init --user "$(id -u):$(id -g)"
               -v "$run":/lvio/run
               -v "$LVIO_ROOT/algorithms/$dir/export.yaml":/lvio/export.yaml:ro
               -v "$SCRIPT_DIR":/app:ro
               -v "$LVIO_ROOT/common/lvio_py":/lvio_py:ro -e PYTHONPATH=/lvio_py)
    if [ -f "$COMMON_DIR/$seq.bag" ]; then
        cmd+=(-v "$COMMON_DIR/$seq.bag":/lvio/common.bag:ro)
    fi
    cmd+=("$IMAGE_NAME" python3 /app/export_results.py --run /lvio/run --spec /lvio/export.yaml
          --id "$id" --sequence "$seq" --common-bag /lvio/common.bag --voxel-size "$VOXEL_SIZE")
    mkdir -p "$run/logs"
    if [ "$JOBS" -eq 1 ]; then
        "${cmd[@]}" 2>&1 | tee "$run/logs/export.log"
        rc=${PIPESTATUS[0]}
    else
        "${cmd[@]}" > "$run/logs/export.log" 2>&1
        rc=$?
    fi
    case "$rc" in
        0)   status=ok ;;
        130) status=interrupted ;;
        *)   status=failed ;;
    esac
    record "$seq" "$id" "$status" $((SECONDS - start)) "$run/export"
    echo "=== $seq/$id: export $status after $((SECONDS - start)) s (log: $run/logs/export.log)"
    [ "$status" = ok ]
}

INTERRUPTED=0
trap 'INTERRUPTED=1' INT TERM
declare -A PIDS=()   # pid -> sequence/id
failed=()

# reap: wait for one export to finish and note a failure
reap() {
    local pid
    wait -n -p pid
    local rc=$?
    [ -n "${pid:-}" ] || return 0
    [ "$rc" -eq 0 ] || failed+=("${PIDS[$pid]}")
    unset "PIDS[$pid]"
}

while IFS=$'\t' read -r -u 4 seq _seq_bag; do
    while IFS=$'\t' read -r -u 3 _distro dir id _input; do
        [ "$INTERRUPTED" -eq 1 ] && break
        run="$RESULTS_DIR/$seq/$id"
        spec="$LVIO_ROOT/algorithms/$dir/export.yaml"
        if [ ! -f "$spec" ]; then
            echo "Skipping $seq/$id: no $spec"; continue
        fi
        if [ ! -d "$run" ]; then
            echo "Skipping $seq/$id: no results in $run"; continue
        fi
        run_status="$(last_run_status "$seq" "$id")"
        if [ -n "$run_status" ] && [ "$run_status" != ok ]; then
            echo "Skipping $seq/$id: its latest run is '$run_status' (run_summary.csv)"
            record "$seq" "$id" skipped 0 "run $run_status"
            continue
        fi
        if [ -d "$run/export" ] && [ "$FORCE" -eq 0 ]; then
            echo "Skipping $seq/$id: $run/export exists (use --force to redo)"; continue
        fi
        while [ ${#PIDS[@]} -ge "$JOBS" ]; do reap; done
        [ "$INTERRUPTED" -eq 1 ] && break
        echo "==========================================="
        echo "Exporting $seq/$id"
        export_one "$seq" "$dir" "$id" &
        PIDS[$!]="$seq/$id"
    done 3< <(algo_rows)
    [ "$INTERRUPTED" -eq 1 ] && break
done 4< <(seq_rows)
while [ ${#PIDS[@]} -gt 0 ]; do reap; done

echo "==========================================="
echo "Summary: $SUMMARY"
if [ "$INTERRUPTED" -eq 1 ]; then
    echo "INTERRUPTED: remaining exports were not started" >&2
    exit 130
fi
if [ ${#failed[@]} -gt 0 ]; then
    echo "FAILED: ${failed[*]}" >&2
    exit 1
fi
echo "DONE"
