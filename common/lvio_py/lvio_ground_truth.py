"""Ground truth of a sequence, as described by steps/1_prepare_data/<dataset>_calibration.yaml (Steps 5-6).

    ground_truth:
      format: nc_csv | tum          # '#sec,nsec,x,y,z,qx,qy,qz,qw' (Newer College) or TUM
      file: <path relative to datasets/<dataset>/, may contain {sequence}>
      max_rate_hz: <optional: keep at most this rate, e.g. 10 for a ~200 Hz ground truth>
    base_frame:
      T_gt_lidar: {rotation, translation}   # LiDAR pose in the ground-truth body frame
"""

from pathlib import Path

import numpy as np
import yaml

from lvio_trajectory import NS, TrajectoryError, normalize_quats, read_gt_csv, read_tum, rigid_from_spec

QUAT_TOLERANCE = 0.02   # some ground-truth rows print qw with too few digits (COCO stairs: '0.96', '-0.3')


def thin(t_ns, rate_hz):
    """Indices keeping at most rate_hz (greedy: each kept stamp >= 1/rate after the previous one)."""
    step = int(NS / rate_hz * 0.999)
    keep, last = [], None
    for i, t in enumerate(t_ns):
        if last is None or t - last >= step:
            keep.append(i)
            last = t
    return np.asarray(keep, dtype=np.int64)


def load(datasets_dir, dataset, sequence, calibration_dir):
    calib_path = Path(calibration_dir) / f'{dataset.lower()}_calibration.yaml'
    if not calib_path.is_file():
        raise TrajectoryError(f'no calibration file {calib_path} for dataset {dataset}')
    calib = yaml.safe_load(calib_path.read_text())
    spec = calib.get('ground_truth')
    if not spec:
        raise TrajectoryError(f'{calib_path} has no ground_truth section')
    path = Path(datasets_dir) / dataset / spec['file'].format(sequence=sequence)
    if not path.is_file():
        raise TrajectoryError(f'missing ground truth {path}')
    if spec['format'] == 'nc_csv':
        t, p, q = read_gt_csv(path)
    elif spec['format'] == 'tum':
        t, p, q = read_tum(path)
    else:
        raise TrajectoryError(f"{calib_path}: unknown ground_truth format {spec['format']!r}")
    order = np.argsort(t, kind='stable')
    t, p, q = t[order], p[order], q[order]
    deviation = np.abs(np.linalg.norm(q, axis=1) - 1)
    q = normalize_quats(q, tolerance=QUAT_TOLERANCE)
    raw_count = len(t)
    if spec.get('max_rate_hz'):
        keep = thin(t, float(spec['max_rate_hz']))
        t, p, q = t[keep], p[keep], q[keep]
    T_gt_lidar = rigid_from_spec(calib['base_frame']['T_gt_lidar'], calib_path.parent)
    return {'t': t, 'p': p, 'q': q, 'path': path, 'file': f"{dataset}/{spec['file'].format(sequence=sequence)}",
            'raw_poses': raw_count, 'renormalised': int((deviation > 1e-4).sum()), 'max_deviation': float(deviation.max()),
            'max_rate_hz': spec.get('max_rate_hz'), 'T_gt_lidar': T_gt_lidar,
            'base_source': calib['base_frame'].get('source', ''), 'calibration': calib_path.name}
