"""Trajectory helpers shared by Steps 4-6 (numpy and PyYAML only).

Poses are kept as int64 ns stamps, positions (N,3) float64 and unit quaternions (N,4) in
x y z w order (TUM order). T_a_b is the pose of frame b in frame a: p_a = R_a_b p_b + t_a_b.
"""

from pathlib import Path

import numpy as np
import yaml

NS = 1_000_000_000


class TrajectoryError(ValueError):
    """Unusable trajectory or spec input."""


# ------------------------------------------------------------------ files

def parse_time_ns(text):
    """Exact ns from a decimal seconds string ('1625132455.999615908')."""
    sec, _, frac = text.partition('.')
    if sec.startswith('-') or not (sec.isdigit() and (frac.isdigit() or frac == '')):
        raise ValueError(f'not a timestamp: {text!r}')
    return int(sec) * NS + int((frac + '000000000')[:9])


def format_time_ns(t_ns):
    t_ns = int(t_ns)
    return f'{t_ns // NS}.{t_ns % NS:09d}'


def read_tum(path):
    """TUM file ('t x y z qx qy qz qw', '#' comments, spaces or commas) -> (t_ns, p, q)."""
    t, p, q = [], [], []
    with open(path) as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            fields = line.replace(',', ' ').split()
            if len(fields) != 8:
                raise TrajectoryError(f'{path}:{line_no}: expected 8 TUM fields, got {len(fields)}')
            t.append(parse_time_ns(fields[0]))
            p.append([float(v) for v in fields[1:4]])
            q.append([float(v) for v in fields[4:8]])
    return (np.asarray(t, dtype=np.int64), np.asarray(p, dtype=np.float64).reshape(-1, 3),
            np.asarray(q, dtype=np.float64).reshape(-1, 4))


def write_tum(path, t_ns, p, q):
    """TUM as in the LIO benchmark: no header, 9-decimal stamps, 10-decimal values."""
    with open(path, 'w') as f:
        for t, pp, qq in zip(t_ns, p, q):
            f.write(f'{format_time_ns(t)} {pp[0]:.10f} {pp[1]:.10f} {pp[2]:.10f} '
                    f'{qq[0]:.10f} {qq[1]:.10f} {qq[2]:.10f} {qq[3]:.10f}\n')


def read_gt_csv(path):
    """Newer College ground truth '#sec,nsec,x,y,z,qx,qy,qz,qw' -> (t_ns, p, q), exact stamps."""
    t, p, q = [], [], []
    with open(path) as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            fields = line.split(',')
            if len(fields) != 9:
                raise TrajectoryError(f'{path}:{line_no}: expected 9 fields, got {len(fields)}')
            t.append(int(fields[0]) * NS + int(fields[1]))
            p.append([float(v) for v in fields[2:5]])
            q.append([float(v) for v in fields[5:9]])
    return (np.asarray(t, dtype=np.int64), np.asarray(p, dtype=np.float64).reshape(-1, 3),
            np.asarray(q, dtype=np.float64).reshape(-1, 4))


def resolve(value, base_dir):
    """A literal, or {file, key}: the value at a dotted key of a YAML file (relative to base_dir)."""
    if isinstance(value, dict) and 'file' in value and 'key' in value:
        path = Path(base_dir) / value['file']
        if not path.is_file():
            raise TrajectoryError(f'missing {path} (referenced by a spec)')
        node = yaml.safe_load(path.read_text())
        for part in str(value['key']).split('.'):
            if not isinstance(node, dict) or part not in node:
                raise TrajectoryError(f"key '{value['key']}' not found in {path}")
            node = node[part]
        return node
    return value


def rigid_from_spec(spec, base_dir):
    """{rotation: 9 values row-major or 'identity', translation: 3 values} (each may be {file, key}) -> 4x4."""
    rot = resolve(spec.get('rotation', 'identity'), base_dir)
    T = np.eye(4)
    if not (isinstance(rot, str) and rot == 'identity'):
        T[:3, :3] = np.asarray(rot, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(resolve(spec.get('translation', [0, 0, 0]), base_dir), dtype=np.float64).reshape(3)
    if abs(np.linalg.det(T[:3, :3]) - 1) > 1e-3:
        raise TrajectoryError(f'not a rotation matrix (det {np.linalg.det(T[:3, :3]):.4f}): {spec}')
    return T


# ------------------------------------------------------------------ rotations

def quat_to_rot(q):
    """(N,4) xyzw -> (N,3,3)."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], axis=-1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], axis=-1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], axis=-1),
    ], axis=1)


def rot_to_quat(R):
    """(N,3,3) -> (N,4) xyzw, w >= 0."""
    R = np.asarray(R, dtype=np.float64).reshape(-1, 3, 3)
    q = np.empty((len(R), 4))
    tr = np.trace(R, axis1=1, axis2=2)
    for i, (m, t) in enumerate(zip(R, tr)):
        if t > 0:
            s = 2 * np.sqrt(t + 1)
            q[i] = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4]
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = 2 * np.sqrt(1 + m[0, 0] - m[1, 1] - m[2, 2])
            q[i] = [s / 4, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
        elif m[1, 1] > m[2, 2]:
            s = 2 * np.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2])
            q[i] = [(m[0, 1] + m[1, 0]) / s, s / 4, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
        else:
            s = 2 * np.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1])
            q[i] = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4, (m[1, 0] - m[0, 1]) / s]
    q[q[:, 3] < 0] *= -1
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def normalize_quats(q, tolerance=1e-3):
    norm = np.linalg.norm(q, axis=1)
    if len(q) and np.abs(norm - 1).max() > tolerance:
        raise TrajectoryError(f'quaternion norm off by up to {np.abs(norm - 1).max():.2e}')
    return q / norm[:, None]


def right_multiply(p, q, T):
    """Poses T_w_a (p, q) times a constant 4x4 T_a_b -> T_w_b."""
    R = quat_to_rot(q)
    p_out = p + R @ T[:3, 3]
    q_out = rot_to_quat(R @ T[:3, :3])
    return p_out, q_out


# ------------------------------------------------------------------ interpolation and coverage

def bracket(t_src, t_query, max_gap_ns):
    """Index i with t_src[i] <= t_query <= t_src[i+1], fraction a, and the valid mask
    (inside the span and the bracketing poses at most max_gap_ns apart)."""
    i = np.clip(np.searchsorted(t_src, t_query, side='right') - 1, 0, len(t_src) - 2)
    span = t_src[i + 1] - t_src[i]
    a = np.where(span > 0, (t_query - t_src[i]) / np.maximum(span, 1), 0.0)
    valid = (t_query >= t_src[0]) & (t_query <= t_src[-1]) & (span <= max_gap_ns)
    return i, np.clip(a, 0.0, 1.0), valid


def slerp(q0, q1, a):
    """Spherical interpolation of (N,4) quaternions with fractions a (N,)."""
    q1 = q1.copy()
    dot = (q0 * q1).sum(axis=1)
    q1[dot < 0] *= -1
    dot = np.abs(dot)
    a = a[:, None]
    theta = np.arccos(np.clip(dot, -1.0, 1.0))[:, None]
    small = theta < 1e-6
    sin_t = np.where(small, 1.0, np.sin(theta))
    w0 = np.where(small, 1 - a, np.sin((1 - a) * theta) / sin_t)
    w1 = np.where(small, a, np.sin(a * theta) / sin_t)
    q = w0 * q0 + w1 * q1
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def interpolate(t_src, p_src, q_src, t_query, max_gap_ns):
    """Poses at t_query (linear position, slerp rotation) and the valid mask. Invalid rows are NaN."""
    if len(t_src) < 2:
        raise TrajectoryError(f'need at least 2 poses to interpolate, got {len(t_src)}')
    i, a, valid = bracket(t_src, t_query, max_gap_ns)
    p = p_src[i] + a[:, None] * (p_src[i + 1] - p_src[i])
    q = slerp(q_src[i], q_src[i + 1], a)
    p[~valid] = np.nan
    q[~valid] = np.nan
    return p, q, valid


def coverage(t_est, t_gt, max_gap_ns):
    """How much of the ground-truth time line an estimate covers (all times in ns)."""
    gaps = np.diff(t_est)
    _, _, valid = bracket(t_est, t_gt, max_gap_ns) if len(t_est) >= 2 else (None, None, np.zeros(len(t_gt), bool))
    return {
        'poses': int(len(t_est)),
        'rate_hz': round(NS / float(np.median(gaps)), 2) if len(gaps) else None,
        'start_delay_s': round((int(t_est[0]) - int(t_gt[0])) / NS, 3),
        'end_shortfall_s': round((int(t_gt[-1]) - int(t_est[-1])) / NS, 3),
        'longest_gap_s': round(int(gaps.max()) / NS, 3) if len(gaps) else None,
        'gt_poses': int(len(t_gt)),
        'gt_poses_covered': int(valid.sum()),
        'covered_fraction': round(float(valid.mean()), 4),
    }
