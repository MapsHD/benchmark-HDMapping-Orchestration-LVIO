#!/bin/bash
# Checks the benchmark's prerequisites (see README.md, "Validate and clean").
# Read-only: it builds no image, runs no container and changes no file.
set -uo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=algos_lib.sh
source "$ROOT/algos_lib.sh"

DATA_DIR="$LVIO_DATASETS_DIR"

usage() {
    cat <<EOF
Usage: $0 [--data-dir DIR]

Checks:
  Docker  the docker command, a running daemon, and access without sudo
  Data    for every sequence of sequences.conf: <data-dir>/<dataset>/<bag> is a readable ROS 1 bag,
          and its dataset's ground truth (Steps 5 and 6) is there:
            COCO    <data-dir>/COCO/ground_truth/gt-nc-<sequence>.csv
            BUNKER  <data-dir>/BUNKER/HDMappingGroundTruth/lio_result_0/lio_result_0_trajectory_tum.txt

  --data-dir DIR   folder holding the dataset folders (default: datasets)

Set ONLY_SEQUENCES="stairs" to check only those sequences.
Exits 1 if any check fails. start_benchmark.sh runs it before Step 1.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --data-dir) DATA_DIR="${2:?--data-dir needs a value}"; shift 2 ;;
        -h|--help)  usage; exit 0 ;;
        *)          echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 1 ;;
    esac
done

check_only_sequences
DATA_DIR="$(realpath -m "$DATA_DIR")"

PASSED=0
FAILED=0
pass() { echo "PASS: $*"; PASSED=$((PASSED + 1)); }
fail() { echo "FAIL: $*" >&2; FAILED=$((FAILED + 1)); }

echo "=== Docker"
if command -v docker >/dev/null; then
    pass "docker command: $(docker --version 2>/dev/null)"
    if info="$(timeout 30 docker info --format '{{.ServerVersion}}' 2>&1)"; then
        pass "Docker daemon $info is running and usable without sudo"
    elif grep -qi 'permission denied' <<< "$info"; then
        fail "no permission to use the Docker daemon. Add your user to the docker group" \
             "(sudo usermod -aG docker \$USER) and log in again"
    else
        fail "Docker daemon not reachable: $(head -n 1 <<< "$info")" \
             "(start it, e.g. sudo systemctl start docker)"
    fi
else
    fail "docker command not found: install Docker Engine (https://docs.docker.com/engine/install/)"
fi

echo "=== Datasets ($DATA_DIR)"
while IFS=$'\t' read -r -u 3 seq dataset bag; do
    path="$DATA_DIR/$dataset/$bag"
    if [ ! -f "$path" ] || [ ! -r "$path" ] || [ ! -s "$path" ]; then
        fail "$seq: missing, empty or unreadable bag $path (see datasets/README.md)"
    elif [ "$(head -c 13 "$path")" != "#ROSBAG V2.0" ]; then
        fail "$seq: $path is not a ROS 1 bag (no '#ROSBAG V2.0' header)"
    else
        pass "$seq: $dataset/$bag ($(du -h "$path" | cut -f1))"
    fi
    case "$dataset" in
        COCO)
            gt="$DATA_DIR/COCO/ground_truth/gt-nc-$seq.csv"
            if [ ! -f "$gt" ] || [ ! -r "$gt" ] || [ ! -s "$gt" ]; then
                fail "$seq: missing, empty or unreadable ground truth $gt"
            elif [ "$(head -n 1 "$gt" | tr -d '\r')" != "#sec,nsec,x,y,z,qx,qy,qz,qw" ]; then
                fail "$seq: $gt does not start with the header '#sec,nsec,x,y,z,qx,qy,qz,qw'"
            else
                pass "$seq: ground truth $(basename "$gt") ($(($(wc -l < "$gt") - 1)) poses)"
            fi ;;
        BUNKER)
            gt="$DATA_DIR/BUNKER/HDMappingGroundTruth/lio_result_0/lio_result_0_trajectory_tum.txt"
            if [ ! -f "$gt" ] || [ ! -r "$gt" ] || [ ! -s "$gt" ]; then
                fail "$seq: missing, empty or unreadable ground truth $gt"
            else
                pass "$seq: ground truth $(basename "$gt") ($(wc -l < "$gt") poses)"
            fi ;;
        *)
            fail "$seq: unknown dataset '$dataset' (sequences.conf)" ;;
    esac
done 3< <(seq_rows)

echo "=== $PASSED passed, $FAILED failed"
[ "$FAILED" -eq 0 ]
