#!/bin/bash
# Checks the benchmark's prerequisites (see README.md, "Validate and clean").
# Read-only: it builds no image, runs no container and changes no file.
set -uo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=algos_lib.sh
source "$ROOT/algos_lib.sh"

DATA_DIR="$LVIO_ROOT/datasets/COCO"

usage() {
    cat <<EOF
Usage: $0 [--data-dir DIR]

Checks:
  Docker  the docker command, a running daemon, and access without sudo
  COCO    <data-dir> holds every original bag of sequences.conf as a readable ROS 1 bag

  --data-dir DIR   dataset directory (default: datasets/COCO)

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

echo "=== COCO dataset ($DATA_DIR)"
if [ -d "$DATA_DIR" ] && [ -r "$DATA_DIR" ] && [ -x "$DATA_DIR" ]; then
    pass "dataset directory accessible: $DATA_DIR"
else
    fail "missing or inaccessible dataset directory: $DATA_DIR (see datasets/README.md)"
fi
while IFS=$'\t' read -r -u 3 seq bag; do
    path="$DATA_DIR/$bag"
    if [ ! -f "$path" ] || [ ! -r "$path" ] || [ ! -s "$path" ]; then
        fail "$seq: missing, empty or unreadable bag $path"
    elif [ "$(head -c 13 "$path")" != "#ROSBAG V2.0" ]; then
        fail "$seq: $path is not a ROS 1 bag (no '#ROSBAG V2.0' header)"
    else
        pass "$seq: $bag ($(du -h "$path" | cut -f1))"
    fi
done 3< <(seq_rows)

echo "=== $PASSED passed, $FAILED failed"
[ "$FAILED" -eq 0 ]
