#!/bin/bash
# Step 1: build the common ROS 1 bag of every selected sequence (see README.md, Step 1).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=../../algos_lib.sh
source "$SCRIPT_DIR/../../algos_lib.sh"

IMAGE_NAME="lvio-prepare"
DATA_DIR="$LVIO_DATASETS_DIR"
OUTPUT_DIR=""
DURATION=""
FORCE=0
USE_DOCKER=1

usage() {
    cat <<EOF
Usage: $0 [--data-dir DIR] [--output-dir DIR] [--duration SECONDS] [--force] [--no-docker]

Converts each sequence of sequences.conf into its common ROS 1 bag (uncompressed), with the
recipe and calibration of its dataset (steps/1_prepare_data/<dataset>_calibration.yaml):
  <data-dir>/<dataset>/<bag>  ->  <output-dir>/<sequence>.bag

  --data-dir DIR      folder holding the dataset folders          (default: datasets)
  --output-dir DIR    where common bags are written       (default: <data-dir>/<dataset>/common-ros1)
  --duration SECONDS  convert only the first SECONDS of each bag (quick tests)
  --force             replace existing common bags (default: skip them)
  --no-docker         run the converter with the local python3 instead of the
                      lvio-prepare image (needs rosbags, numpy, opencv-python-headless, PyYAML)

Set ONLY_SEQUENCES="quad-easy stairs" to convert only those sequences.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --data-dir)   DATA_DIR="${2:?--data-dir needs a value}"; shift 2 ;;
        --output-dir) OUTPUT_DIR="${2:?--output-dir needs a value}"; shift 2 ;;
        --duration)   DURATION="${2:?--duration needs a value}"; shift 2 ;;
        --force)      FORCE=1; shift ;;
        --no-docker)  USE_DOCKER=0; shift ;;
        -h|--help)    usage; exit 0 ;;
        *)            echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 1 ;;
    esac
done

check_only_sequences

DATA_DIR="$(realpath "$DATA_DIR")"
[ -n "$OUTPUT_DIR" ] && OUTPUT_DIR="$(realpath -m "$OUTPUT_DIR")"

EXTRA_ARGS=()
[ -n "$DURATION" ] && EXTRA_ARGS+=(--duration "$DURATION")

if [ "$USE_DOCKER" -eq 1 ]; then
    echo "Building helper image '$IMAGE_NAME'..."
    docker build -t "$IMAGE_NAME" "$SCRIPT_DIR"
fi

failed=()
while IFS=$'\t' read -r -u 3 seq dataset bag; do
    data="$DATA_DIR/$dataset"
    out="${OUTPUT_DIR:-$data/common-ros1}"
    calib="$(basename "$(dataset_calibration "$dataset")")"
    src="$data/$bag"
    dst="$out/$seq.bag"
    mkdir -p "$out"
    echo "==========================================="
    echo "Sequence $seq: $src -> $dst"
    if [ ! -f "$src" ]; then
        echo "ERROR: missing source bag $src" >&2
        failed+=("$seq"); continue
    fi
    if [ -e "$dst" ]; then
        if [ "$FORCE" -eq 0 ]; then
            echo "Skipping: $dst exists (use --force to replace)"
            continue
        fi
        rm -f "$dst"
    fi

    if [ "$USE_DOCKER" -eq 1 ]; then
        cmd=(docker run --rm --user "$(id -u):$(id -g)"
             -v "$data":/data:ro -v "$out":/out -v "$SCRIPT_DIR":/app:ro
             "$IMAGE_NAME" python3 /app/make_common_bag.py
             "/data/$bag" "/out/$seq.bag" --dataset "$dataset" --calib "/app/$calib")
    else
        cmd=("${PYTHON:-python3}" "$SCRIPT_DIR/make_common_bag.py"
             "$src" "$dst" --dataset "$dataset" --calib "$SCRIPT_DIR/$calib")
    fi
    if ! "${cmd[@]}" "${EXTRA_ARGS[@]}"; then
        echo "ERROR: conversion of $seq failed" >&2
        failed+=("$seq")
    fi
done 3< <(seq_rows)

echo "==========================================="
if [ ${#failed[@]} -gt 0 ]; then
    echo "FAILED: ${failed[*]}" >&2
    exit 1
fi
echo "DONE. Common bags in ${OUTPUT_DIR:-$DATA_DIR/<dataset>/common-ros1}"
