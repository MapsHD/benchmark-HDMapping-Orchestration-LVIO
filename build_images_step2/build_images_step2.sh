#!/bin/bash
# Step 2: build the Docker image of every selected algorithm (see README.md, Step 2).
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=../algos_lib.sh
source "$SCRIPT_DIR/../algos_lib.sh"

NO_CACHE=()

usage() {
    cat <<EOF
Usage: $0 [--no-cache]

Builds algorithms/<dir>/Dockerfile as image <id>_<distro> for each row of algorithms.conf.

  --no-cache   build without Docker's layer cache (fresh clones of the algorithm repositories)

Set ONLY_ALGOS="fast-livo2 r3live" to build only those algorithms.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-cache) NO_CACHE=(--no-cache); shift ;;
        -h|--help)  usage; exit 0 ;;
        *)          echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 1 ;;
    esac
done

check_only_algos

built=() failed=()
while IFS=$'\t' read -r -u 3 distro dir id _input; do
    context="$LVIO_ROOT/algorithms/$dir"
    tag="$(image_tag "$id" "$distro")"
    echo "==========================================="
    echo "Building $id: $context -> $tag"
    if [ ! -f "$context/Dockerfile" ]; then
        echo "ERROR: $context/Dockerfile not found" >&2
        failed+=("$id"); continue
    fi
    start=$SECONDS
    if docker build "${NO_CACHE[@]}" -t "$tag" "$context"; then
        built+=("$tag ($((SECONDS - start)) s)")
    else
        echo "ERROR: build of $id failed" >&2
        failed+=("$id")
    fi
done 3< <(algo_rows)

echo "==========================================="
for b in "${built[@]}"; do echo "Built:  $b"; done
if [ ${#failed[@]} -gt 0 ]; then
    echo "FAILED: ${failed[*]}" >&2
    exit 1
fi
echo "DONE"
