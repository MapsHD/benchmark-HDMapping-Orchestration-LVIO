#!/usr/bin/env python3
"""Step 5: trajectory errors of every exported run against the Newer College ground truth (evo).

For each selected (sequence, algorithm):
  1. read the Step 4 trajectory  results/<seq>/<id>/export/<id>_trajectory_tum.txt  (pose of the
     algorithm's body frame in its own world frame);
  2. convert it to the ground-truth body frame 'Base':
         T_world_base = T_world_body * T_body_lidar * T_lidar_base
     T_body_lidar: the LiDAR extrinsic the algorithm was configured with (export.yaml lidar_extrinsic,
     read from the run's config copy); T_lidar_base: one value for all (coco_calibration.yaml);
  3. resample it at the ground-truth stamps (linear position, slerp rotation), only inside its own
     time span and never across its own gaps longer than --max-gap;
  4. evo: APE after SE(3) Umeyama alignment (translation and rotation), RPE over --rpe-delta metres
     of ground-truth path (same pairs for every algorithm), and the translation APE for time shifts
     of -0.2..+0.2 s (diagnostic, not applied).

Writes  results/<seq>/evaluation/trajectories/<id>/{metrics.json, <id>_base_tum.txt,
<id>_aligned_tum.txt, time_shift.csv}, per sequence ground_truth_tum.txt, trajectories.png and
ape_over_time.png, and rebuilds results/evaluation/trajectory_summary.{csv,md,tex} from every
metrics.json on disk. Needs numpy, PyYAML, matplotlib and evo (the lvio-eval image).

export.yaml key used here:
    lidar_extrinsic: {rotation: 9 values row-major | identity | {file, key},
                      translation: 3 values | {file, key}}     # LiDAR pose in the body frame
"""

import argparse
import copy
import csv
import datetime
import json
import shutil
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml
from evo.core import geometry, metrics
from evo.core.metrics import PoseRelation, Unit
from evo.core.trajectory import PoseTrajectory3D

# common/lvio_py, on PYTHONPATH (evaluate_trajectories.sh mounts it)
from lvio_trajectory import (NS, TrajectoryError, coverage, format_time_ns, interpolate, normalize_quats,
                             read_gt_csv, read_tum, right_multiply, rigid_from_spec, write_tum)
import lvio_ground_truth
from lvio_results import missing_reason, spec_for_dataset, trajectory_path

PARTIAL = 0.99          # covered fraction below this is marked as partial in the tables
TIME_SHIFTS = np.round(np.arange(-0.20, 0.2001, 0.01), 3)
MIN_POSES = 20
GT_QUAT_TOLERANCE = 0.02  # ground-truth quaternions are renormalised up to this norm error


def log(msg):
    print(f'[evaluate] {msg}', flush=True)


def stats(metric):
    return {k: round(float(v), 6) for k, v in metric.get_all_statistics().items()}


def evo_traj(t_ns, p, q_xyzw):
    return PoseTrajectory3D(positions_xyz=p, orientations_quat_wxyz=q_xyzw[:, [3, 0, 1, 2]],
                            timestamps=t_ns / NS)


def time_shift_sweep(t_gt, p_gt, t_est, p_est, q_est, max_gap_ns):
    """Translation APE RMSE (after Umeyama) of est(t + dt) against gt(t) for each dt."""
    rows = []
    for dt in TIME_SHIFTS:
        p, _, valid = interpolate(t_est, p_est, q_est, t_gt + int(round(dt * NS)), max_gap_ns)
        if valid.sum() < MIN_POSES:
            rows.append((dt, None, int(valid.sum())))
            continue
        x, y = p[valid].T, p_gt[valid].T
        r, t, _ = geometry.umeyama_alignment(x, y, with_scale=False)
        err = y - (r @ x + t[:, None])
        rows.append((dt, float(np.sqrt((err ** 2).sum(axis=0).mean())), int(valid.sum())))
    return rows


def evaluate_run(args, seq, dataset, algo, algo_dir, gt, out_dir):
    """Evaluate one run; returns the metrics document (status ok) or raises."""
    t_gt, p_gt, q_gt = gt
    run = args.results / seq / algo
    tum = trajectory_path(args.results, seq, algo)
    spec_path = args.algorithms / algo_dir / 'export.yaml'
    spec = spec_for_dataset(yaml.safe_load(spec_path.read_text()), dataset)
    if 'lidar_extrinsic' not in spec:
        raise TrajectoryError(f'{spec_path} has no lidar_extrinsic')
    T_body_lidar = rigid_from_spec(spec['lidar_extrinsic'], run)
    T_body_base = T_body_lidar @ args.T_lidar_base

    t_est, p_body, q_body = read_tum(tum)
    q_body = normalize_quats(q_body)
    p_est, q_est = right_multiply(p_body, q_body, T_body_base)
    max_gap_ns = int(args.max_gap * NS)
    cov = coverage(t_est, t_gt, max_gap_ns)
    p_rs, q_rs, valid = interpolate(t_est, p_est, q_est, t_gt, max_gap_ns)
    n = int(valid.sum())
    if n < MIN_POSES:
        raise TrajectoryError(f'only {n} ground-truth stamps are covered by the trajectory')

    ref = evo_traj(t_gt[valid], p_gt[valid], q_gt[valid])
    est = evo_traj(t_gt[valid], p_rs[valid], q_rs[valid])
    warnings = []
    aligned = copy.deepcopy(est)
    try:
        r_a, t_a, _ = aligned.align(ref, correct_scale=False)
        method = 'se3_umeyama'
    except geometry.GeometryException:
        aligned = copy.deepcopy(est)
        T = aligned.align_origin(ref)
        r_a, t_a = T[:3, :3], T[:3, 3]
        method = 'origin'
        warnings.append('the trajectory never moves, so SE(3) alignment is impossible: aligned at the first pose')

    ape_t = metrics.APE(PoseRelation.translation_part)
    ape_t.process_data((ref, aligned))
    ape_r = metrics.APE(PoseRelation.rotation_angle_deg)
    ape_r.process_data((ref, aligned))

    rpe = {'delta_m': args.rpe_delta, 'pairs_from': 'ground truth', 'all_pairs': True}
    try:
        rpe_t = metrics.RPE(PoseRelation.translation_part, delta=args.rpe_delta, delta_unit=Unit.meters,
                            all_pairs=True, pairs_from_reference=True)
        rpe_t.process_data((ref, est))
        rpe_r = metrics.RPE(PoseRelation.rotation_angle_deg, delta=args.rpe_delta, delta_unit=Unit.meters,
                            all_pairs=True, pairs_from_reference=True)
        rpe_r.process_data((ref, est))
        scale = 100.0 / args.rpe_delta
        rpe['pairs'] = int(len(rpe_t.error))
        rpe['translation_pct'] = {k: round(v * scale, 4) for k, v in stats(rpe_t).items()}
        rpe['rotation_deg_per_m'] = {k: round(v / args.rpe_delta, 6) for k, v in stats(rpe_r).items()}
    except Exception as err:  # evo raises when no pair spans delta (short or interrupted trajectory)
        warnings.append(f'no RPE: {err}')

    sweep = time_shift_sweep(t_gt, p_gt, t_est, p_est, q_est, max_gap_ns)
    usable = [(dt, e) for dt, e, _ in sweep if e is not None]
    best_dt, best_e = min(usable, key=lambda r: r[1]) if usable else (None, None)
    zero_e = next((e for dt, e, _ in sweep if dt == 0.0), None)
    if best_dt is not None and abs(best_dt) >= TIME_SHIFTS[-1] - 1e-9:
        warnings.append(f'the best time shift is at the edge of the sweep ({best_dt:+.2f} s)')
    if cov['covered_fraction'] < PARTIAL:
        warnings.append(f"covers {cov['covered_fraction']:.1%} of the ground truth: the errors describe that part only")

    # files
    a_pos = aligned.positions_xyz
    a_q = aligned.orientations_quat_wxyz[:, [1, 2, 3, 0]]
    write_tum(out_dir / f'{algo}_base_tum.txt', t_gt[valid], p_rs[valid], q_rs[valid])
    write_tum(out_dir / f'{algo}_aligned_tum.txt', t_gt[valid], a_pos, a_q)
    with open(out_dir / 'time_shift.csv', 'w') as f:
        f.write('time_shift_s,ape_translation_rmse_m,poses\n')
        for dt, e, k in sweep:
            f.write(f"{dt:+.2f},{'' if e is None else f'{e:.6f}'},{k}\n")
    return {
        'status': 'ok',
        'inputs': {'trajectory': str(tum.relative_to(args.results)), 'spec': f'algorithms/{algo_dir}/export.yaml'},
        'frames': spec.get('frames', {}),
        'T_body_lidar': np.round(T_body_lidar, 9).tolist(),
        'T_lidar_base': np.round(args.T_lidar_base, 9).tolist(),
        'max_gap_s': args.max_gap,
        'coverage': cov,
        'poses_evaluated': n,
        'alignment': {'method': method, 'rotation': np.round(r_a, 9).tolist(),
                      'translation': np.round(t_a, 6).tolist(), 'scale': 1.0,
                      'meaning': 'p_gt = R * p_estimate_in_Base + t'},
        'ape_translation_m': stats(ape_t),
        'ape_rotation_deg': stats(ape_r),
        'rpe': rpe,
        'time_shift': {'range_s': [float(TIME_SHIFTS[0]), float(TIME_SHIFTS[-1])], 'step_s': 0.01,
                       'best_s': best_dt, 'ape_rmse_at_best_m': None if best_e is None else round(best_e, 6),
                       'ape_rmse_at_zero_m': None if zero_e is None else round(zero_e, 6)},
        'ape_over_time': {'t_s': np.round((t_gt[valid] - t_gt[0]) / NS, 3).tolist(),
                          'error_m': np.round(ape_t.error, 4).tolist()},
        'warnings': warnings,
    }


# ------------------------------------------------------------------ plots

def colors(algos):
    cmap = plt.get_cmap('tab10')
    return {algo: cmap(i % 10) for i, (algo, _) in enumerate(algos)}


def plot_sequence(seq_dir, seq, gt, algos, docs):
    t_gt, p_gt, _ = gt
    col = colors(algos)
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    for ax, (i, j, name) in zip(axes, [(0, 1, 'top view (x, y)'), (0, 2, 'side view (x, z)')]):
        ax.plot(p_gt[:, i], p_gt[:, j], 'k--', lw=1.2, label='ground truth')
        for algo, _ in algos:
            path = seq_dir / algo / f'{algo}_aligned_tum.txt'
            if docs.get(algo, {}).get('status') == 'ok' and path.is_file():
                _, p, _ = read_tum(path)
                ax.plot(p[:, i], p[:, j], color=col[algo], lw=1.0, label=algo)
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]' if j == 1 else 'z [m]')
        ax.set_title(name)
        ax.grid(alpha=0.3)
    axes[0].legend(loc='best', fontsize=9)
    fig.suptitle(f'{seq}: trajectories in the ground-truth body frame, SE(3)-aligned to the ground truth')
    fig.tight_layout()
    fig.savefig(seq_dir / 'trajectories.png', dpi=120)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(14, 5))
    for algo, _ in algos:
        d = docs.get(algo, {})
        if d.get('status') == 'ok':
            ax.plot(d['ape_over_time']['t_s'], d['ape_over_time']['error_m'], color=col[algo], lw=1.0,
                    label=f"{algo} (RMSE {d['ape_translation_m']['rmse']:.3f} m)")
    ax.set_xlabel(f'time since the first ground-truth pose [s] ({format_time_ns(t_gt[0])})')
    ax.set_ylabel('APE translation [m]')
    ax.set_title(f'{seq}: translation error over time (SE(3)-aligned)')
    ax.grid(alpha=0.3)
    if ax.get_legend_handles_labels()[0]:
        ax.legend(loc='upper left', fontsize=9)
    fig.tight_layout()
    fig.savefig(seq_dir / 'ape_over_time.png', dpi=120)
    plt.close(fig)


# ------------------------------------------------------------------ summary

SUMMARY_FIELDS = ['sequence', 'algorithm', 'status', 'reason', 'covered_fraction', 'start_delay_s',
                  'end_shortfall_s', 'poses_evaluated', 'ape_rmse_m', 'ape_mean_m', 'ape_median_m', 'ape_max_m',
                  'ape_rot_rmse_deg', 'rpe_trans_mean_pct', 'rpe_rot_mean_deg_per_m', 'rpe_pairs',
                  'best_time_shift_s', 'alignment', 'warnings']


def summary_row(seq, algo, d):
    row = {'sequence': seq, 'algorithm': algo, 'status': d.get('status', 'not evaluated'),
           'reason': d.get('reason', '')}
    if d.get('status') == 'ok':
        cov, ape, rpe = d['coverage'], d['ape_translation_m'], d['rpe']
        row.update({
            'covered_fraction': cov['covered_fraction'], 'start_delay_s': cov['start_delay_s'],
            'end_shortfall_s': cov['end_shortfall_s'], 'poses_evaluated': d['poses_evaluated'],
            'ape_rmse_m': ape['rmse'], 'ape_mean_m': ape['mean'], 'ape_median_m': ape['median'],
            'ape_max_m': ape['max'], 'ape_rot_rmse_deg': d['ape_rotation_deg']['rmse'],
            'rpe_trans_mean_pct': rpe.get('translation_pct', {}).get('mean', ''),
            'rpe_rot_mean_deg_per_m': rpe.get('rotation_deg_per_m', {}).get('mean', ''),
            'rpe_pairs': rpe.get('pairs', ''), 'best_time_shift_s': d['time_shift']['best_s'],
            'alignment': d['alignment']['method'], 'warnings': ' | '.join(d.get('warnings', [])),
        })
    return row


def cell(row, key, fmt):
    if row['status'] == 'n/a':
        return 'n/a'
    if row['status'] != 'ok':
        return '–'
    value = row.get(key)
    if value in ('', None):
        return '–'
    text = fmt.format(value)
    return text + (' †' if row['covered_fraction'] < PARTIAL else '')


def pivot(rows, seqs, algos, key, fmt, best=min):
    """Markdown table algorithms x sequences; the best full-coverage value per sequence in bold."""
    by = {(r['sequence'], r['algorithm']): r for r in rows}
    lines = ['| Algorithm | ' + ' | '.join(seqs) + ' |', '| --- |' + ' ---: |' * len(seqs)]
    winners = {}
    for s in (seqs if best else []):
        cands = [(by[(s, a)][key], a) for a, _ in algos
                 if by[(s, a)]['status'] == 'ok' and by[(s, a)].get(key) not in ('', None)
                 and by[(s, a)]['covered_fraction'] >= PARTIAL]
        if cands:
            winners[s] = best(cands)[1]
    for a, _ in algos:
        cells = []
        for s in seqs:
            text = cell(by[(s, a)], key, fmt)
            cells.append(f'**{text}**' if winners.get(s) == a else text)
        lines.append(f'| {a} | ' + ' | '.join(cells) + ' |')
    return lines, winners


def latex_pivot(rows, seqs, algos, key, fmt, caption, label):
    by = {(r['sequence'], r['algorithm']): r for r in rows}
    _, winners = pivot(rows, seqs, algos, key, fmt)
    out = ['\\begin{table}[]', '\\centering', '\\begin{tabular}{l' + 'r' * len(seqs) + '}',
           '\\rowcolor[HTML]{C0C0C0}', 'Algorithm & ' + ' & '.join(seqs) + ' \\\\']
    for a, _ in algos:
        cells = []
        for s in seqs:
            text = cell(by[(s, a)], key, fmt).replace('†', '$^\\dagger$').replace('–', '-')
            cells.append(f'\\textbf{{{text}}}' if winners.get(s) == a else text)
        out.append(f'{a} & ' + ' & '.join(cells) + ' \\\\')
    out += ['\\end{tabular}', f'\\caption{{{caption}}}', f'\\label{{{label}}}', '\\end{table}', '']
    return out


def write_summary(results, seqs, algos, max_gap, rpe_delta):
    rows = []
    for seq in seqs:
        for algo, _ in algos:
            path = results / seq / 'evaluation' / 'trajectories' / algo / 'metrics.json'
            d = json.loads(path.read_text()) if path.is_file() else {'status': 'not evaluated',
                                                                    'reason': 'Step 5 not run for it'}
            rows.append(summary_row(seq, algo, d))
    out = results / 'evaluation'
    out.mkdir(parents=True, exist_ok=True)
    with open(out / 'trajectory_summary.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS)
        w.writeheader()
        w.writerows(rows)

    md = ['# Trajectory evaluation (Step 5)', '',
          'Ground truth per dataset (`ground_truth` and `base_frame` in `steps/1_prepare_data/<dataset>_calibration.yaml`): '
          'COCO = Newer College multi-cam, 10 Hz, one pose per LiDAR scan; its body frame is called `Base` in the '
          'dataset docs, the data shows it is the LiDAR (`os_sensor`) pose. BUNKER = the HDMapping ground-truth '
          'session of the LIO benchmark (~200 Hz, evaluated at 10 Hz), a LiDAR (Livox Mid-360) pose.',
          '',
          f'Every trajectory is converted to that frame with the LiDAR extrinsic its algorithm was configured with, '
          f'resampled at the ground-truth stamps (no interpolation across its own gaps > {max_gap} s) and '
          f'SE(3)-aligned (Umeyama) before the APE. The RPE uses the same {rpe_delta:g} m segments of the ground '
          f'truth for every algorithm.',
          '',
          f'Best value per sequence in **bold** (runs covering at least {PARTIAL:.0%} only); † = the run covers less '
          f'than {PARTIAL:.0%} of the ground truth, so its errors describe that part only.', '']
    sections = [('APE translation RMSE [m]', 'ape_rmse_m', '{:.3f}', min),
                ('APE rotation RMSE [deg]', 'ape_rot_rmse_deg', '{:.2f}', min),
                (f'RPE translation, mean over {rpe_delta:g} m segments [%]', 'rpe_trans_mean_pct', '{:.2f}', min),
                (f'RPE rotation, mean over {rpe_delta:g} m segments [deg/m]', 'rpe_rot_mean_deg_per_m', '{:.4f}', min),
                ('Ground-truth coverage [%]', 'covered_fraction', '{:.1%}', None),
                ('Time shift with the lowest APE [s] (diagnostic: -0.20..+0.20 s, not applied)',
                 'best_time_shift_s', '{:+.2f}', None)]
    for title, key, fmt, best in sections:
        lines, _ = pivot(rows, seqs, algos, key, fmt, best)
        if best is None:   # coverage and time shift: no partial-coverage mark, no winner
            lines = lines[:2] + [ln.replace(' †', '') for ln in lines[2:]]
        md += [f'## {title}', ''] + lines + ['']
    notes = [f"- {r['sequence']} / {r['algorithm']}: {r['status']}" + (f" ({r['reason']})" if r['reason'] else '')
             for r in rows if r['status'] not in ('ok', 'n/a')]
    if any(r['status'] == 'n/a' for r in rows):
        notes.append('- n/a: the algorithm does not run on that dataset (datasets column of algorithms.conf)')
    notes += [f"- {r['sequence']} / {r['algorithm']}: {r['warnings']}" for r in rows
              if r['status'] == 'ok' and r['warnings']]
    if notes:
        md += ['## Notes', ''] + notes + ['']
    md += ['Per-run details: `results/<sequence>/evaluation/trajectories/<id>/metrics.json`; plots: '
           '`results/<sequence>/evaluation/trajectories/{trajectories,ape_over_time}.png`.', '']
    (out / 'trajectory_summary.md').write_text('\n'.join(md))

    tex = ['% Generated by steps/5_evaluate_trajectories/evaluate_trajectories.py', '']
    tex += latex_pivot(rows, seqs, algos, 'ape_rmse_m', '{:.3f}',
                       'APE translation RMSE [m] after SE(3) alignment, ground truth in the Base frame. '
                       '$^\\dagger$ partial coverage of the ground truth.', 'tab:lvio_ape')
    tex += latex_pivot(rows, seqs, algos, 'rpe_trans_mean_pct', '{:.2f}',
                       f'RPE translation [\\%], mean over {rpe_delta:g} m segments of the ground truth. '
                       '$^\\dagger$ partial coverage of the ground truth.', 'tab:lvio_rpe')
    (out / 'trajectory_summary.tex').write_text('\n'.join(tex))
    return rows


# ------------------------------------------------------------------ main

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--results', type=Path, required=True)
    parser.add_argument('--datasets', type=Path, required=True, help='the datasets/ folder (ground truth inside)')
    parser.add_argument('--calibration-dir', type=Path, required=True,
                        help='folder with <dataset>_calibration.yaml (ground_truth, base_frame)')
    parser.add_argument('--algorithms', type=Path, required=True, help='the algorithms/ folder (export.yaml)')
    parser.add_argument('--sequences', nargs='+', required=True, help='every sequence as id=DATASET, in order')
    parser.add_argument('--algos', nargs='+', required=True,
                        help='every algorithm as id=dir=DATASET[,DATASET...], in order')
    parser.add_argument('--select-sequences', nargs='*', default=None)
    parser.add_argument('--select-algos', nargs='*', default=None)
    parser.add_argument('--max-gap', type=float, default=0.5, help='longest own gap to interpolate across [s]')
    parser.add_argument('--rpe-delta', type=float, default=10.0, help='RPE segment length along the GT [m]')
    args = parser.parse_args()

    seqs = [tuple(a.split('=', 1)) for a in args.sequences]
    algo_rows = [a.split('=', 2) for a in args.algos]
    algos = [(a, d) for a, d, _ in algo_rows]
    supported = {a: set(ds.split(',')) for a, _, ds in algo_rows}
    sel_seqs = set(args.select_sequences or [s for s, _ in seqs])
    sel_algos = set(args.select_algos or [a for a, _ in algos])

    failures = 0
    for seq, dataset in seqs:
        if seq not in sel_seqs:
            continue
        seq_dir = args.results / seq / 'evaluation' / 'trajectories'
        seq_dir.mkdir(parents=True, exist_ok=True)
        g = lvio_ground_truth.load(args.datasets, dataset, seq, args.calibration_dir)
        args.T_lidar_base = np.linalg.inv(g['T_gt_lidar'])
        gt = (g['t'], g['p'], g['q'])
        write_tum(seq_dir / 'ground_truth_tum.txt', *gt)
        rate = f", thinned to {g['max_rate_hz']} Hz from {g['raw_poses']}" if g['max_rate_hz'] else ''
        log(f"{seq} ({dataset}): ground truth {len(g['t'])} poses{rate}, {(g['t'][-1] - g['t'][0]) / NS:.1f} s "
            f"({g['file']}); LiDAR in the ground-truth frame at {g['T_gt_lidar'][:3, 3].tolist()} ({g['base_source']})")
        if g['renormalised']:
            log(f"{seq}: {g['renormalised']} ground-truth quaternion(s) renormalised "
                f"(norm off by up to {g['max_deviation']:.1e})")
        for algo, algo_dir in algos:
            if algo not in sel_algos:
                continue
            out_dir = seq_dir / algo
            if out_dir.exists():
                shutil.rmtree(out_dir)
            out_dir.mkdir()
            doc = {'sequence': seq, 'dataset': dataset, 'algorithm': algo,
                   'created': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds'),
                   'ground_truth': {'file': g['file'], 'poses': int(len(g['t'])), 'raw_poses': g['raw_poses'],
                                    'calibration': g['calibration']}}
            if dataset not in supported[algo]:
                doc.update(status='n/a', reason=f'not supported on {dataset} (algorithms.conf)')
                log(f'  {algo:11s} n/a: {doc["reason"]}')
            elif not trajectory_path(args.results, seq, algo).is_file():
                doc.update(status='missing', reason=missing_reason(args.results, seq, algo))
                log(f'  {algo:11s} {doc["status"]}: {doc["reason"]}')
            else:
                try:
                    doc.update(evaluate_run(args, seq, dataset, algo, algo_dir, gt, out_dir))
                    log(f"  {algo:11s} APE {doc['ape_translation_m']['rmse']:.3f} m, "
                        f"RPE {doc['rpe'].get('translation_pct', {}).get('mean', float('nan')):.2f} %, "
                        f"coverage {doc['coverage']['covered_fraction']:.1%}, "
                        f"best time shift {doc['time_shift']['best_s']:+.2f} s")
                    for w in doc['warnings']:
                        log(f'  {algo:11s} WARNING: {w}')
                except (TrajectoryError, OSError, ValueError, KeyError) as err:
                    doc.update(status='failed', reason=str(err))
                    failures += 1
                    log(f'  {algo:11s} FAILED: {err}')
            (out_dir / 'metrics.json').write_text(json.dumps(doc, indent=1) + '\n')
        docs = {}
        for algo, _ in algos:
            path = seq_dir / algo / 'metrics.json'
            if path.is_file():
                docs[algo] = json.loads(path.read_text())
        plot_sequence(seq_dir, seq, gt, algos, docs)

    write_summary(args.results, [s for s, _ in seqs], algos, args.max_gap, args.rpe_delta)
    log(f"summary: {args.results / 'evaluation' / 'trajectory_summary.md'}")
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
