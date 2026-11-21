#!/bin/bash
# Step 5: trajectory errors of every exported run against the ground truth, with evo (see README.md, Step 5).
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=../../algos_lib.sh
source "$SCRIPT_DIR/../../algos_lib.sh"

IMAGE_NAME="lvio-eval"   # Python + evo, built from this folder
DATA_DIR="$LVIO_ROOT/datasets/COCO"
RESULTS_DIR="$LVIO_ROOT/results"
CALIBRATION="$LVIO_ROOT/steps/1_prepare_data/coco_calibration.yaml"
MAX_GAP="0.5"
RPE_DELTA="10"

usage() {
    cat <<EOF
Usage: $0 [--results-dir DIR] [--data-dir DIR] [--max-gap SECONDS] [--rpe-delta METERS]

Evaluates each exported trajectory (Step 4) against <data-dir>/ground_truth/gt-nc-<sequence>.csv:
converted to the ground-truth frame, resampled at the ground-truth stamps, then evo APE (after
SE(3) alignment) and RPE. Writes results/<sequence>/evaluation/trajectories/ and the summary
results/evaluation/trajectory_summary.{csv,md,tex}.

  --results-dir DIR    results of Steps 3 and 4                     (default: results)
  --data-dir DIR       dataset directory (ground_truth/ inside)     (default: datasets/COCO)
  --max-gap SECONDS    longest own gap a trajectory is interpolated across (default: 0.5)
  --rpe-delta METERS   RPE segment length along the ground truth    (default: 10)

Set ONLY_ALGOS / ONLY_SEQUENCES to re-evaluate only those runs; the summary always covers every
run evaluated so far. Evaluations are always redone (they take seconds).
EOF
}

positive_number() { [[ "$1" =~ ^[0-9]*\.?[0-9]+$ ]] && awk -v v="$1" 'BEGIN { exit !(v > 0) }'; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --results-dir) RESULTS_DIR="${2:?--results-dir needs a value}"; shift 2 ;;
        --data-dir)    DATA_DIR="${2:?--data-dir needs a value}"; shift 2 ;;
        --max-gap)     MAX_GAP="${2:?--max-gap needs a value}"; shift 2 ;;
        --rpe-delta)   RPE_DELTA="${2:?--rpe-delta needs a value}"; shift 2 ;;
        -h|--help)     usage; exit 0 ;;
        *)             echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 1 ;;
    esac
done

for value in "$MAX_GAP" "$RPE_DELTA"; do
    if ! positive_number "$value"; then
        echo "ERROR: --max-gap and --rpe-delta must be positive numbers, got '$value'" >&2; exit 1
    fi
done
check_only_algos
check_only_sequences

RESULTS_DIR="$(realpath -m "$RESULTS_DIR")"
GT_DIR="$(realpath -m "$DATA_DIR/ground_truth")"
if [ ! -d "$RESULTS_DIR" ]; then
    echo "ERROR: no results directory $RESULTS_DIR (run Steps 3 and 4)" >&2; exit 1
fi
if [ ! -d "$GT_DIR" ]; then
    echo "ERROR: no ground truth directory $GT_DIR (see datasets/README.md)" >&2; exit 1
fi

# Every sequence and algorithm in conf order (for the summary), and the selected ones.
mapfile -t ALL_SEQS < <(seq_ids)
mapfile -t SEL_SEQS < <(seq_rows | cut -f1)
ALL_ALGOS=()
while IFS=$'\t' read -r _distro dir id _input; do ALL_ALGOS+=("$id=$dir"); done < <(ONLY_ALGOS='' algo_rows)
mapfile -t SEL_ALGOS < <(algo_rows | cut -f3)
for seq in "${SEL_SEQS[@]}"; do
    if [ ! -f "$GT_DIR/gt-nc-$seq.csv" ]; then
        echo "ERROR: missing ground truth $GT_DIR/gt-nc-$seq.csv" >&2; exit 1
    fi
done

echo "Building helper image '$IMAGE_NAME'..."
docker build -q -t "$IMAGE_NAME" "$SCRIPT_DIR" >/dev/null || exit 1

docker run --rm --init --user "$(id -u):$(id -g)" \
    -v "$RESULTS_DIR":/results \
    -v "$GT_DIR":/gt:ro \
    -v "$LVIO_ROOT/algorithms":/algorithms:ro \
    -v "$CALIBRATION":/calibration/coco_calibration.yaml:ro \
    -v "$SCRIPT_DIR":/app:ro \
    -v "$LVIO_ROOT/common/lvio_py":/lvio_py:ro -e PYTHONPATH=/lvio_py \
    "$IMAGE_NAME" python3 /app/evaluate_trajectories.py \
        --results /results --ground-truth /gt --algorithms /algorithms \
        --calibration /calibration/coco_calibration.yaml \
        --sequences "${ALL_SEQS[@]}" --algos "${ALL_ALGOS[@]}" \
        --select-sequences "${SEL_SEQS[@]}" --select-algos "${SEL_ALGOS[@]}" \
        --max-gap "$MAX_GAP" --rpe-delta "$RPE_DELTA"
status=$?
echo "==========================================="
echo "Summary: $RESULTS_DIR/evaluation/trajectory_summary.md"
if [ "$status" -ne 0 ]; then
    echo "FAILED: some evaluations failed (see the [evaluate] FAILED lines)" >&2
    exit "$status"
fi
echo "DONE"
