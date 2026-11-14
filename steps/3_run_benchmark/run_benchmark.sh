#!/bin/bash
# Step 3: run every selected algorithm on every selected sequence (see README.md, Step 3).
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=../../algos_lib.sh
source "$SCRIPT_DIR/../../algos_lib.sh"

DATA_DIR="$LVIO_ROOT/datasets/COCO"
COMMON_DIR=""
OUTPUT_DIR="$LVIO_ROOT/results"
FORCE=0

usage() {
    cat <<EOF
Usage: $0 [--data-dir DIR] [--common-dir DIR] [--output-dir DIR] [--force] [--no-rviz] [--rate RATE]

Runs algorithms/<dir>/run.sh for each (sequence, algorithm) pair:
  <common-dir>/<sequence>.bag  ->  <output-dir>/<sequence>/<algorithm-id>/

  --data-dir DIR    dataset directory                       (default: datasets/COCO)
  --common-dir DIR  common ROS 1 bags from Step 1           (default: <data-dir>/common-ros1)
  --output-dir DIR  results                                 (default: results)
  --force           delete and redo existing results        (default: skip them)
  --no-rviz         run without RViz (headless); by default every run shows RViz (X session needed)
  --rate RATE       rosbag play rate                        (default: 1.0)

Set ONLY_ALGOS="r3live lvi-sam" and/or ONLY_SEQUENCES="stairs" to restrict the runs.
Every run appends a line to <output-dir>/run_summary.csv (status ok | failed | interrupted).
Ctrl-C stops the current run gracefully (bag closed, map saved) and ends the loop.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --data-dir)   DATA_DIR="${2:?--data-dir needs a value}"; shift 2 ;;
        --common-dir) COMMON_DIR="${2:?--common-dir needs a value}"; shift 2 ;;
        --output-dir) OUTPUT_DIR="${2:?--output-dir needs a value}"; shift 2 ;;
        --force)      FORCE=1; shift ;;
        --no-rviz)    export LVIO_RVIZ=0; shift ;;
        --rate)       export LVIO_PLAY_RATE="${2:?--rate needs a value}"; shift 2 ;;
        -h|--help)    usage; exit 0 ;;
        *)            echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 1 ;;
    esac
done

check_only_algos
check_only_sequences
export LVIO_RVIZ="${LVIO_RVIZ:-1}"   # RViz is on by default
check_rviz_display || exit 1

DATA_DIR="$(realpath -m "$DATA_DIR")"
COMMON_DIR="$(realpath -m "${COMMON_DIR:-$DATA_DIR/common-ros1}")"
OUTPUT_DIR="$(realpath -m "$OUTPUT_DIR")"
mkdir -p "$OUTPUT_DIR"
SUMMARY="$OUTPUT_DIR/run_summary.csv"
[ -f "$SUMMARY" ] || echo "finished_at,sequence,algorithm,status,seconds,output" > "$SUMMARY"

# resolve_input <kind> <sequence> <original bag>
resolve_input() {
    case "$1" in
        common-ros1) echo "$COMMON_DIR/$2.bag" ;;
        *)      echo "" ;;
    esac
}

failed=()
# Ctrl-C: the running container gets the signal from docker run (proxied) and stops gracefully;
# this flag then ends the loop instead of starting the next run.
INTERRUPTED=0
trap 'INTERRUPTED=1' INT TERM

# Tables are read on file descriptors 3 and 4, not stdin, so nothing started inside the
# loop can consume them.
while IFS=$'\t' read -r -u 4 seq seq_bag; do
    while IFS=$'\t' read -r -u 3 _distro dir id input; do
        [ "$INTERRUPTED" -eq 1 ] && break
        run_script="$LVIO_ROOT/algorithms/$dir/run.sh"
        out="$OUTPUT_DIR/$seq/$id"
        echo "==========================================="
        echo "Sequence $seq, algorithm $id"

        if [ ! -x "$run_script" ]; then
            echo "Skipping: $run_script missing or not executable"
            continue
        fi
        bag="$(resolve_input "$input" "$seq" "$seq_bag")"
        if [ -z "$bag" ]; then
            echo "ERROR: unknown input kind '$input' for $id in algorithms.conf" >&2
            failed+=("$seq/$id"); continue
        fi
        if [ ! -f "$bag" ]; then
            echo "ERROR: missing input $bag (run Step 1)" >&2
            failed+=("$seq/$id"); continue
        fi
        if [ -d "$out" ] && [ -n "$(ls -A "$out")" ]; then
            if [ "$FORCE" -eq 0 ]; then
                echo "Skipping: $out exists (use --force to redo)"
                continue
            fi
            echo "Removing previous results in $out"
            rm -rf "$out"
        fi

        start=$SECONDS
        "$run_script" "$bag" "$out"
        rc=$?
        if [ "$INTERRUPTED" -eq 1 ] || [ "$rc" -eq 130 ]; then
            INTERRUPTED=1
            status=interrupted
        elif [ "$rc" -eq 0 ]; then
            status=ok
        else
            status=failed
            failed+=("$seq/$id")
        fi
        echo "$(date -Iseconds),$seq,$id,$status,$((SECONDS - start)),$out" >> "$SUMMARY"
        echo "=== $seq/$id: $status after $((SECONDS - start)) s"
        [ "$INTERRUPTED" -eq 1 ] && break
        echo "Waiting 5 seconds before the next run"
        sleep 5
    done 3< <(algo_rows)
    [ "$INTERRUPTED" -eq 1 ] && break
done 4< <(seq_rows)

echo "==========================================="
echo "Summary: $SUMMARY"
if [ "$INTERRUPTED" -eq 1 ]; then
    echo "INTERRUPTED: remaining runs were not started" >&2
    exit 130
fi
if [ ${#failed[@]} -gt 0 ]; then
    echo "FAILED: ${failed[*]}" >&2
    exit 1
fi
echo "DONE"
