#!/bin/bash
# Runs Steps 1-3 in order (see README.md): prepare common bags, build images, run algorithms.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"

usage() {
    cat <<EOF
Usage: $0 [--force] [--rviz] [--rate RATE] [algorithm-id ...]

  Step 1  prepare_data_step1/prepare_data_step1.sh    (existing common bags are kept)
  Step 2  build_images_step2/build_images_step2.sh
  Step 3  run_benchmark_step3/run_benchmark_step3.sh   (--force/--rviz/--rate are passed here)

Algorithm ids (the id column of algorithms.conf) restrict Steps 2 and 3, e.g.:
  $0 fast-livo2 r3live
ONLY_SEQUENCES="stairs" restricts Steps 1 and 3.
EOF
}

STEP3_ARGS=()
ALGOS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --force)   STEP3_ARGS+=(--force); shift ;;
        --rviz)    STEP3_ARGS+=(--rviz); shift ;;
        --rate)    STEP3_ARGS+=(--rate "${2:?--rate needs a value}"); shift 2 ;;
        -h|--help) usage; exit 0 ;;
        -*)        echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 1 ;;
        *)         ALGOS+=("$1"); shift ;;
    esac
done
if [ ${#ALGOS[@]} -gt 0 ]; then
    export ONLY_ALGOS="${ALGOS[*]}"
fi

echo "########## Step 1: prepare common bags"
"$ROOT/prepare_data_step1/prepare_data_step1.sh"
echo "########## Step 2: build images"
"$ROOT/build_images_step2/build_images_step2.sh"
echo "########## Step 3: run algorithms"
"$ROOT/run_benchmark_step3/run_benchmark_step3.sh" "${STEP3_ARGS[@]}"
