"""Helpers for the results/ layout shared by Steps 5-6 (standard library only)."""

import csv
from pathlib import Path


def last_run_status(results, seq, algo):
    """Status of the latest Step 3 run of (seq, algo) in results/run_summary.csv, or None."""
    path = Path(results) / 'run_summary.csv'
    status = None
    if path.is_file():
        with open(path) as f:
            for row in csv.DictReader(f):
                if row.get('sequence') == seq and row.get('algorithm') == algo:
                    status = row.get('status')
    return status


def missing_reason(results, seq, algo):
    """Why results/<seq>/<algo> has no Step 4 trajectory."""
    status = last_run_status(results, seq, algo)
    if status and status != 'ok':
        return f'run {status} (Step 3)'
    if not (Path(results) / seq / algo).is_dir():
        return 'no run (Step 3)'
    return 'no export (Step 4)'


def trajectory_path(results, seq, algo):
    """The Step 4 trajectory of a run."""
    return Path(results) / seq / algo / 'export' / f'{algo}_trajectory_tum.txt'
