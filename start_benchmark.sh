#!/bin/bash
# Runs validate.sh, then Steps 1-4 in order (see README.md): prepare common bags, build images,
# run algorithms, export trajectories (TUM) and maps (PLY).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=algos_lib.sh
source "$ROOT/algos_lib.sh"

usage() {
    cat <<EOF
Usage: $0 [--force] [--no-rviz] [--rate RATE] [algorithm-id ...]

          validate.sh                               (Docker and the COCO dataset; stops on failure)
  Step 1  steps/1_prepare_data/prepare_data.sh      (existing common bags are kept)
  Step 2  steps/2_build_images/build_images.sh
  Step 3  steps/3_run_benchmark/run_benchmark.sh    (--force/--no-rviz/--rate are passed here)
  Step 4  steps/4_export/export.sh                  (existing exports are kept; runs even if a
                                                     Step 3 run failed, not after Ctrl-C)

RViz is shown for every run by default (X session needed); --no-rviz runs headless.

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
        --no-rviz) STEP3_ARGS+=(--no-rviz); export LVIO_RVIZ=0; shift ;;
        --rate)    STEP3_ARGS+=(--rate "${2:?--rate needs a value}"); shift 2 ;;
        -h|--help) usage; exit 0 ;;
        -*)        echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 1 ;;
        *)         ALGOS+=("$1"); shift ;;
    esac
done
if [ ${#ALGOS[@]} -gt 0 ]; then
    export ONLY_ALGOS="${ALGOS[*]}"
fi

# Fail now, not after Steps 1-2, if RViz (the default) has no display.
check_rviz_display
echo "########## Validate"
"$ROOT/validate.sh"
echo "########## Step 1: prepare common bags"
"$ROOT/steps/1_prepare_data/prepare_data.sh"
echo "########## Step 2: build images"
"$ROOT/steps/2_build_images/build_images.sh"
echo "########## Step 3: run algorithms"
step3_status=0
"$ROOT/steps/3_run_benchmark/run_benchmark.sh" "${STEP3_ARGS[@]}" || step3_status=$?
if [ "$step3_status" -eq 130 ]; then
    exit 130  # Ctrl-C: stop here
fi
# A failed run is skipped by Step 4; the other runs are still exported.
echo "########## Step 4: export trajectories and maps"
step4_status=0
"$ROOT/steps/4_export/export.sh" || step4_status=$?
if [ "$step3_status" -ne 0 ] || [ "$step4_status" -ne 0 ]; then
    echo "Finished with failures: Step 3 exit $step3_status, Step 4 exit $step4_status" >&2
    exit 1
fi
