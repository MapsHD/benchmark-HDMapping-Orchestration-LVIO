#!/bin/bash
# Runs Steps 1-4 in order (see README.md): prepare common bags, build images, run algorithms,
# export trajectories (TUM) and maps (PLY).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"

usage() {
    cat <<EOF
Usage: $0 [--force] [--rviz] [--rate RATE] [algorithm-id ...]

  Step 1  prepare_data_step1/prepare_data_step1.sh    (existing common bags are kept)
  Step 2  build_images_step2/build_images_step2.sh
  Step 3  run_benchmark_step3/run_benchmark_step3.sh   (--force/--rviz/--rate are passed here)
  Step 4  export_step4/export_step4.sh                 (existing exports are kept; runs even if a
                                                        Step 3 run failed, not after Ctrl-C)

Algorithm ids (the id column of algorithms.conf) restrict Steps 2, 3 and 4, e.g.:
  $0 fast-livo2 r3live
ONLY_SEQUENCES="stairs" restricts Steps 1, 3 and 4.
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
step3_status=0
"$ROOT/run_benchmark_step3/run_benchmark_step3.sh" "${STEP3_ARGS[@]}" || step3_status=$?
if [ "$step3_status" -eq 130 ]; then
    exit 130  # Ctrl-C: stop here
fi
# A failed run is skipped by Step 4; the other runs are still exported.
echo "########## Step 4: export trajectories and maps"
step4_status=0
"$ROOT/export_step4/export_step4.sh" || step4_status=$?
if [ "$step3_status" -ne 0 ] || [ "$step4_status" -ne 0 ]; then
    echo "Finished with failures: Step 3 exit $step3_status, Step 4 exit $step4_status" >&2
    exit 1
fi
