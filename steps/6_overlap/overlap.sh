#!/bin/bash
# Step 6: how much of the ground truth every exported trajectory covers (see README.md, Step 6).
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=../../algos_lib.sh
source "$SCRIPT_DIR/../../algos_lib.sh"

IMAGE_NAME="lvio-eval"   # built from steps/5_evaluate_trajectories/
DATA_DIR="$LVIO_DATASETS_DIR"
RESULTS_DIR="$LVIO_ROOT/results"
MAX_GAP="0.5"

usage() {
    cat <<EOF
Usage: $0 [--results-dir DIR] [--data-dir DIR] [--max-gap SECONDS]

Compares each exported trajectory (Step 4) with the ground-truth time line of its sequence:
covered share of the ground-truth stamps, start delay, how early it ends, longest gap, rate.
Writes results/<sequence>/evaluation/overlap/ and results/evaluation/overlap_summary.{csv,md,tex}.

  --results-dir DIR    results of Steps 3 and 4                  (default: results)
  --data-dir DIR       folder holding the dataset folders        (default: datasets)
  --max-gap SECONDS    own gaps longer than this are not covered (default: 0.5, as Step 5)

Set ONLY_ALGOS / ONLY_SEQUENCES to redo only those runs; the summary always covers every run
computed so far.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --results-dir) RESULTS_DIR="${2:?--results-dir needs a value}"; shift 2 ;;
        --data-dir)    DATA_DIR="${2:?--data-dir needs a value}"; shift 2 ;;
        --max-gap)     MAX_GAP="${2:?--max-gap needs a value}"; shift 2 ;;
        -h|--help)     usage; exit 0 ;;
        *)             echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 1 ;;
    esac
done

if ! [[ "$MAX_GAP" =~ ^[0-9]*\.?[0-9]+$ ]] || ! awk -v v="$MAX_GAP" 'BEGIN { exit !(v > 0) }'; then
    echo "ERROR: --max-gap must be a positive number, got '$MAX_GAP'" >&2; exit 1
fi
check_only_algos
check_only_sequences

RESULTS_DIR="$(realpath -m "$RESULTS_DIR")"
DATA_DIR="$(realpath -m "$DATA_DIR")"
if [ ! -d "$RESULTS_DIR" ]; then
    echo "ERROR: no results directory $RESULTS_DIR (run Steps 3 and 4)" >&2; exit 1
fi
ALL_SEQS=()
while IFS=$'\t' read -r seq dataset _bag; do ALL_SEQS+=("$seq=$dataset"); done < <(ONLY_SEQUENCES='' seq_rows)
mapfile -t SEL_SEQS < <(seq_rows | cut -f1)
ALL_ALGOS=()
while IFS=$'\t' read -r _distro _dir id _input datasets; do ALL_ALGOS+=("$id=$datasets"); done < <(ONLY_ALGOS='' algo_rows)
mapfile -t SEL_ALGOS < <(algo_rows | cut -f3)

echo "Building helper image '$IMAGE_NAME'..."
docker build -q -t "$IMAGE_NAME" "$LVIO_ROOT/steps/5_evaluate_trajectories" >/dev/null || exit 1

docker run --rm --init --user "$(id -u):$(id -g)" \
    -v "$RESULTS_DIR":/results \
    -v "$DATA_DIR":/datasets:ro \
    -v "$LVIO_ROOT/steps/1_prepare_data":/calibration:ro \
    -v "$SCRIPT_DIR":/app:ro \
    -v "$LVIO_ROOT/common/lvio_py":/lvio_py:ro -e PYTHONPATH=/lvio_py \
    "$IMAGE_NAME" python3 /app/overlap.py \
        --results /results --datasets /datasets --calibration-dir /calibration \
        --sequences "${ALL_SEQS[@]}" --algos "${ALL_ALGOS[@]}" \
        --select-sequences "${SEL_SEQS[@]}" --select-algos "${SEL_ALGOS[@]}" \
        --max-gap "$MAX_GAP"
status=$?
echo "==========================================="
echo "Summary: $RESULTS_DIR/evaluation/overlap_summary.md"
if [ "$status" -ne 0 ]; then
    echo "FAILED: some runs could not be read (see the [overlap] lines)" >&2
    exit "$status"
fi
echo "DONE"
