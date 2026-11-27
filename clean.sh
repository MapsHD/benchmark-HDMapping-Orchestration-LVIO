#!/bin/bash
# Removes everything the benchmark steps produced (see README.md, "Validate and clean"):
#   Docker images  lvio-prepare (Steps 1 and 4), lvio-eval (Steps 5 and 6) and <id>_<distro> of every
#                  algorithms.conf row (Step 2)
#   Step 1         datasets/<dataset>/common-ros1/ of every dataset (and the obsolete datasets/COCO/common/)
#   Steps 3 to 6   everything in results/ except its .gitignore (runs, exports, evaluation)
# Kept: the original data (bags, calibration, ground truth, reference maps), the repository and notes/.
# Only this benchmark's images are removed; other images and Docker's build cache are not touched.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=algos_lib.sh
source "$ROOT/algos_lib.sh"
unset ONLY_ALGOS ONLY_SEQUENCES   # clean always covers every algorithm and sequence

HELPER_IMAGES=("lvio-prepare" "lvio-eval")
DATA_DIR="$LVIO_DATASETS_DIR"
RESULTS_DIR="$LVIO_ROOT/results"
DRY_RUN=0

usage() {
    cat <<EOF
Usage: $0 [--dry-run]

Removes the outputs of all steps: this benchmark's Docker images, the common bags of Step 1
(datasets/<dataset>/common-ros1/) and the results of Steps 3 to 6 (results/). The original
dataset is kept. Lists what it will remove and asks you to type CLEAN to confirm.

  --dry-run   only list what would be removed
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run) DRY_RUN=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *)         echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 1 ;;
    esac
done

if ! command -v docker >/dev/null || ! timeout 30 docker info >/dev/null 2>&1; then
    echo "ERROR: Docker is not usable, so the images cannot be checked or removed (run ./validate.sh)." >&2
    exit 1
fi

# ---------------------------------------------------------------- what to remove
TAGS=("${HELPER_IMAGES[@]}")
while IFS=$'\t' read -r distro _dir id _input; do
    TAGS+=("$(image_tag "$id" "$distro")")
done < <(algo_rows)
IMAGES=()   # the tags that exist
for tag in "${TAGS[@]}"; do
    if docker image inspect "$tag" >/dev/null 2>&1; then
        IMAGES+=("$tag")
    fi
done

# in_repo <path>: true if <path> (not resolved through a final symlink) lies inside the repository
in_repo() {
    local path
    path="$(realpath -m -s -- "$1")"
    [[ "$path" == "$LVIO_ROOT"/* ]]
}

PATHS=()
for path in $(seq_rows | cut -f2 | sort -u | sed "s|^|$DATA_DIR/|; s|\$|/common-ros1|") "$DATA_DIR/COCO/common"; do
    if [ -e "$path" ] || [ -L "$path" ]; then
        PATHS+=("$path")
    fi
done
shopt -s nullglob dotglob
for path in "$RESULTS_DIR"/*; do
    [ "$(basename "$path")" = .gitignore ] && continue
    PATHS+=("$path")
done
shopt -u nullglob dotglob
for path in "${PATHS[@]}"; do
    if ! in_repo "$path"; then
        echo "ERROR: refusing to remove a path outside the repository: $path" >&2
        exit 1
    fi
done

# A running benchmark container still writes into results/ and uses its image.
running=()
for tag in "${IMAGES[@]}"; do
    while read -r name; do
        [ -n "$name" ] && running+=("$name ($tag)")
    done < <(docker ps --filter "ancestor=$tag" --format '{{.Names}}')
done
if [ ${#running[@]} -gt 0 ]; then
    echo "ERROR: benchmark containers are running: ${running[*]}. Stop them first." >&2
    exit 1
fi

# ---------------------------------------------------------------- confirm
if [ ${#IMAGES[@]} -eq 0 ] && [ ${#PATHS[@]} -eq 0 ]; then
    echo "Nothing to remove."
    exit 0
fi
echo "Docker images to remove:"
if [ ${#IMAGES[@]} -eq 0 ]; then
    echo "  (none)"
fi
for tag in "${IMAGES[@]}"; do
    size="$(docker image inspect -f '{{.Size}}' "$tag" | awk '{ printf "%.1f GB", $1 / 1e9 }')"
    printf '  %-24s %s\n' "$tag" "$size"
done
echo "Files and folders to remove:"
if [ ${#PATHS[@]} -eq 0 ]; then
    echo "  (none)"
fi
for path in "${PATHS[@]}"; do
    if [ -L "$path" ]; then
        printf '  %-8s %s (symlink only; its target is kept)\n' "link" "${path#"$LVIO_ROOT"/}"
    else
        printf '  %-8s %s\n' "$(du -sh -- "$path" 2>/dev/null | cut -f1)" "${path#"$LVIO_ROOT"/}"
    fi
done
echo "Kept: the original data in datasets/<dataset>/ (bags, calibration, ground truth, reference maps) and the repository."
if [ "$DRY_RUN" -eq 1 ]; then
    echo "Dry run: nothing removed."
    exit 0
fi
echo "WARNING: this is permanent. Getting it back means running Steps 1-4 again (a few hours)."
if ! read -r -p "Type CLEAN to confirm: " answer || [ "$answer" != CLEAN ]; then
    echo "Cancelled: nothing removed."
    exit 1
fi

# ---------------------------------------------------------------- remove
errors=0
for tag in "${IMAGES[@]}"; do
    echo "Removing image $tag"
    docker image rm "$tag" >/dev/null || { echo "ERROR: could not remove image $tag" >&2; errors=$((errors + 1)); }
done
for path in "${PATHS[@]}"; do
    echo "Removing ${path#"$LVIO_ROOT"/}"
    if [ -L "$path" ]; then
        rm -- "$path"
    else
        rm -rf --one-file-system -- "$path"
    fi
    if [ -e "$path" ] || [ -L "$path" ]; then
        echo "ERROR: still exists: $path" >&2
        errors=$((errors + 1))
    fi
done
echo "Docker's build cache was kept: 'docker builder prune' frees it (it affects every build on this machine)."
if [ "$errors" -gt 0 ]; then
    echo "FAILED: $errors item(s) could not be removed" >&2
    exit 1
fi
echo "Clean complete."
