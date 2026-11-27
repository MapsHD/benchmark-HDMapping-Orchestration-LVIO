#!/bin/bash
# Shared readers for algorithms.conf and sequences.conf. Every step sources this
# file, so an algorithm or sequence name is written exactly once.
#
# Environment:
#   ONLY_ALGOS="id1 id2"      restrict to these algorithm ids (as spelled in algorithms.conf)
#   ONLY_SEQUENCES="s1 s2"    restrict to these sequence ids (as spelled in sequences.conf)

LVIO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ALGOS_CONF="${ALGOS_CONF:-$LVIO_ROOT/algorithms.conf}"
SEQUENCES_CONF="${SEQUENCES_CONF:-$LVIO_ROOT/sequences.conf}"
LVIO_DATASETS_DIR="${LVIO_DATASETS_DIR:-$LVIO_ROOT/datasets}"   # datasets/<dataset>/ per sequences.conf

for _conf in "$ALGOS_CONF" "$SEQUENCES_CONF"; do
    if [ ! -f "$_conf" ]; then
        echo "ERROR: cannot find $_conf" >&2
        exit 1
    fi
done
unset _conf

# _conf_rows <file> <only-list> <id-column>
# Prints the '|'-separated rows of a conf file as tab-separated, trimmed fields,
# skipping comments and blank lines, and keeping only ids in <only-list> (if set).
_conf_rows() {
    awk -F'|' -v only=" $2 " -v idcol="$3" '
        /^[[:space:]]*#/ { next }
        /^[[:space:]]*$/ { next }
        {
            for (i = 1; i <= NF; i++) { gsub(/^[ \t]+|[ \t]+$/, "", $i) }
            if (only != "  " && index(only, " " $idcol " ") == 0) next
            line = $1
            for (i = 2; i <= NF; i++) line = line "\t" $i
            print line
        }
    ' "$1"
}

# algo_rows: "distro<TAB>dir<TAB>id<TAB>input<TAB>datasets" per algorithm, honouring ONLY_ALGOS.
algo_rows() { _conf_rows "$ALGOS_CONF" "${ONLY_ALGOS:-}" 3; }

# algo_ids: every algorithm id, ignoring ONLY_ALGOS.
algo_ids() { _conf_rows "$ALGOS_CONF" "" 3 | cut -f3; }

# seq_rows: "id<TAB>dataset<TAB>bag" per sequence, honouring ONLY_SEQUENCES.
seq_rows() { _conf_rows "$SEQUENCES_CONF" "${ONLY_SEQUENCES:-}" 1; }

# seq_ids: every sequence id, ignoring ONLY_SEQUENCES.
seq_ids() { _conf_rows "$SEQUENCES_CONF" "" 1 | cut -f1; }

# _check_only <var-name> <known-ids> <conf-file>
_check_only() {
    local value="${!1:-}" known="$2" unknown=()
    [ -n "$value" ] || return 0
    for name in $value; do
        grep -qx -- "$name" <<< "$known" || unknown+=("$name")
    done
    if [ ${#unknown[@]} -gt 0 ]; then
        echo "ERROR: unknown name(s) in $1: ${unknown[*]}" >&2
        echo "Known names (see $3): $(paste -sd' ' - <<< "$known")" >&2
        exit 1
    fi
    echo "$1 set: restricting to $value"
}

# algo_supports <datasets-column> <dataset>: true if the algorithm runs on that dataset.
algo_supports() { [[ ",$1," == *",$2,"* ]]; }

# seq_dataset <sequence>: the dataset of a sequence (ignoring ONLY_SEQUENCES).
seq_dataset() { _conf_rows "$SEQUENCES_CONF" "" 1 | awk -F'\t' -v s="$1" '$1 == s { print $2; exit }'; }

# dataset_calibration <dataset>: its calibration / ground-truth description file.
dataset_calibration() {
    printf '%s/steps/1_prepare_data/%s_calibration.yaml\n' "$LVIO_ROOT" "$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')"
}

# Fail early on a misspelled name instead of silently running nothing.
check_only_algos() { _check_only ONLY_ALGOS "$(algo_ids)" "$ALGOS_CONF"; }
check_only_sequences() { _check_only ONLY_SEQUENCES "$(seq_ids)" "$SEQUENCES_CONF"; }

# check_rviz_display: RViz is on unless LVIO_RVIZ=0 (--no-rviz). It needs an X display that
# containers may use (xhost). Prints why and returns 1 when that is missing.
check_rviz_display() {
    [ "${LVIO_RVIZ:-1}" = 1 ] || return 0
    if [ -z "${DISPLAY:-}" ]; then
        echo "ERROR: RViz is shown by default but DISPLAY is not set (no X session)." >&2
        echo "       Run with --no-rviz (or LVIO_RVIZ=0) for headless runs." >&2
        return 1
    fi
    if ! command -v xhost >/dev/null; then
        echo "ERROR: RViz needs 'xhost' to let containers use the display (apt install x11-xserver-utils)." >&2
        echo "       Or run with --no-rviz (or LVIO_RVIZ=0)." >&2
        return 1
    fi
}

# image_tag <id> <distro>: Docker forbids uppercase in image names.
image_tag() {
    printf '%s_%s\n' "$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')" "$2"
}

# algo_row_for_dir <dir>: the algorithms.conf row of algorithms/<dir>, ignoring ONLY_ALGOS.
algo_row_for_dir() {
    _conf_rows "$ALGOS_CONF" "" 3 | awk -F'\t' -v d="$1" '$2 == d { print; exit }'
}
