#!/usr/bin/env python3
"""Step 4: export one run's trajectory (TUM) and map (PLY), as described by algorithms/<ALGO>/export.yaml.

    export_results.py --run results/<seq>/<id> --spec algorithms/<ALGO>/export.yaml \
                      --id <id> --sequence <seq> [--common-bag common-ros1/<seq>.bag] [--voxel-size 0.05]

Writes <run>/export/ (via export.partial/, renamed when complete, so a failed or interrupted export
leaves no half-written folder):

    <id>_trajectory_tum.txt         'timestamp tx ty tz qx qy qz qw', no header (as in the LIO benchmark)
    <id>_<name>_trajectory_tum.txt  extra trajectories of the spec (e.g. LVI-SAM keyframes)
    <id>_map.ply                    binary little-endian: float x y z + float intensity | uchar red green blue
    <id>_preview.png                top (world XY) and side (world XZ) view of the map around the trajectory
    export.json                     sources, frames, counts, checks and warnings

Spec sources (see the export.yaml files):
    trajectory  odometry   nav_msgs/Odometry topic of recorded-<id>.bag; 'stamp_from: <topic>' takes each
                           pose's time from the header of the n-th message of that topic (for algorithms that
                           stamp odometry with ros::Time::now())
                tum_file   a TUM file the algorithm wrote (glob relative to the run folder, one match)
                path_last  the poses of the last nav_msgs/Path message of a topic
    map         accumulate world-frame PointCloud2 topic of recorded-<id>.bag, summed
                pcd        a PCD file the algorithm saved (ascii or binary)
                reconstruct  LiDAR scans of the common bag (per-point 'time' from the header = scan start)
                           placed with the exported trajectory, interpolated per point, and the LiDAR pose in
                           the body frame ('extrinsic'); used when an algorithm saves no map
Values may be given as {file: <yaml relative to the run folder>, key: a.b.c} to read them from the
run's own config copy.

Maps are reduced to one point per voxel (centroid, mean attribute); --voxel-size 0 keeps every point.
Needs only numpy, rosbags, PyYAML and OpenCV (the lvio-prepare image).
"""

import argparse
import datetime
import glob
import json
import os
import resource
import shutil
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from rosbags.highlevel import AnyReader

# common/lvio_py, on PYTHONPATH (export.sh mounts it)
from lvio_trajectory import NS, TrajectoryError, format_time_ns, read_tum, resolve, write_tum


class ExportError(RuntimeError):
    """A run cannot be exported (missing input, inconsistent data)."""


def log(msg):
    print(f'[export] {msg}', flush=True)


# ------------------------------------------------------------------ spec helpers

def one_file(run_dir, pattern):
    matches = sorted(glob.glob(str(run_dir / pattern)))
    if len(matches) != 1:
        raise ExportError(f'expected exactly one file matching {pattern} in {run_dir}, found {len(matches)}')
    return Path(matches[0])


def stamp_ns(header):
    return int(header.stamp.sec) * NS + int(header.stamp.nanosec)


def connection(reader, topic, bag):
    conns = [c for c in reader.connections if c.topic == topic]
    if not conns:
        raise ExportError(f'topic {topic} not in {bag}')
    return conns


# ------------------------------------------------------------------ trajectories

class Trajectory:
    """Poses as int64 ns stamps, positions (N,3) float64 and quaternions (N,4) xyzw float64."""

    def __init__(self, t_ns, p, q, source):
        self.t_ns = np.asarray(t_ns, dtype=np.int64)
        self.p = np.asarray(p, dtype=np.float64).reshape(-1, 3)
        self.q = np.asarray(q, dtype=np.float64).reshape(-1, 4)
        self.source = source
        self.notes = []

    def check(self):
        """Sort by time, drop repeated stamps, normalise quaternions; fail on unusable data."""
        n = len(self.t_ns)
        if n < 2:
            raise ExportError(f'trajectory has {n} pose(s)')
        if not (np.isfinite(self.p).all() and np.isfinite(self.q).all()):
            raise ExportError('trajectory contains non-finite values')
        order = np.argsort(self.t_ns, kind='stable')
        if (order != np.arange(n)).any():
            self.notes.append('poses were not in time order and were sorted')
        self.t_ns, self.p, self.q = self.t_ns[order], self.p[order], self.q[order]
        keep = np.append(self.t_ns[1:] != self.t_ns[:-1], True)  # keep the last pose of a repeated stamp
        if not keep.all():
            self.notes.append(f'{int((~keep).sum())} pose(s) with a repeated timestamp dropped (last one kept)')
            self.t_ns, self.p, self.q = self.t_ns[keep], self.p[keep], self.q[keep]
        norm = np.linalg.norm(self.q, axis=1)
        if np.abs(norm - 1).max() > 1e-3:
            raise ExportError(f'quaternion norm off by up to {np.abs(norm - 1).max():.2e}')
        self.q /= norm[:, None]
        return self

    def write_tum(self, path):
        write_tum(path, self.t_ns, self.p, self.q)

    def stats(self):
        duration = (self.t_ns[-1] - self.t_ns[0]) / NS
        return {
            'source': self.source,
            'poses': int(len(self.t_ns)),
            'start': format_time_ns(self.t_ns[0]),
            'end': format_time_ns(self.t_ns[-1]),
            'duration_s': round(duration, 3),
            'rate_hz': round((len(self.t_ns) - 1) / duration, 2) if duration > 0 else None,
            'length_m': round(float(np.linalg.norm(np.diff(self.p, axis=0), axis=1).sum()), 3),
            'start_to_end_m': round(float(np.linalg.norm(self.p[-1] - self.p[0])), 3),
            'notes': self.notes,
        }


def trajectory_from_tum(path):
    t, p, q = read_tum(path)
    return Trajectory(t, p, q, f'file {path.name}')


def trajectory_from_odometry(bag, topic, stamp_from=None):
    """Odometry poses; with stamp_from, the n-th pose takes the header stamp of the n-th stamp_from message.

    stamp_from is for algorithms that stamp odometry with the publish time but publish a message with the
    measurement time in the same iteration (stamp_from first, then the odometry). The pairing is checked
    with the bag record times: the n-th odometry must arrive after the n-th and before the (n+1)-th
    stamp_from message. Unlike a time-gap limit, this holds however far the algorithm falls behind
    real time, and a pairing shifted by one message in either direction breaks it.
    """
    t, p, q, pub, rec, rec_from = [], [], [], [], [], []
    with AnyReader([bag]) as reader:
        for conn, bag_t, raw in reader.messages(connections=connection(reader, topic, bag)):
            msg = reader.deserialize(raw, conn.msgtype)
            pose = msg.pose.pose
            pub.append(stamp_ns(msg.header))
            rec.append(bag_t)
            p.append([pose.position.x, pose.position.y, pose.position.z])
            q.append([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        traj = Trajectory(pub, p, q, f'{topic} in {bag.name}')
        if stamp_from is None:
            return traj
        for conn, bag_t, raw in reader.messages(connections=connection(reader, stamp_from, bag)):
            # Only the header is needed; deserialising a large cloud is still cheap (data is a view).
            t.append(stamp_ns(reader.deserialize(raw, conn.msgtype).header))
            rec_from.append(bag_t)
    if len(t) != len(pub):
        raise ExportError(f'{topic} has {len(pub)} messages but {stamp_from} has {len(t)}: cannot pair stamps')
    rec, rec_from = np.asarray(rec, dtype=np.int64), np.asarray(rec_from, dtype=np.int64)

    def interleaved(odom, other):
        """Fraction of odometry messages recorded after their partner and before the partner's successor."""
        nxt = np.append(other[1:], np.iinfo(np.int64).max)
        tol = 2_000_000  # 2 ms
        return float(((odom >= other - tol) & (odom <= nxt + tol)).mean())

    paired = interleaved(rec, rec_from)
    shifted = max(interleaved(rec[:-1], rec_from[1:]), interleaved(rec[1:], rec_from[:-1]))
    lag = (np.asarray(pub, dtype=np.int64) - np.asarray(t, dtype=np.int64)) / NS
    if paired < 0.98 or shifted > 0.5 or lag.min() < 0:
        raise ExportError(f'{topic} and {stamp_from} do not pair 1:1: {paired:.1%} of the pairs are in record '
                          f'order ({shifted:.1%} when shifted by one); publish-minus-stamp min {lag.min():.3f} s')
    traj.t_ns = np.asarray(t, dtype=np.int64)
    traj.source = f'{topic} in {bag.name}, stamps from {stamp_from}'
    traj.notes.append(f'pose stamps taken from the paired {stamp_from} message ({paired:.1%} of the pairs in '
                      f'record order, {shifted:.1%} when shifted by one); the publish-time stamps lagged by '
                      f'median {np.median(lag):.2f} s, max {lag.max():.2f} s (processing behind real time)')
    return traj


def trajectory_from_last_path(bag, topic):
    last = None
    with AnyReader([bag]) as reader:
        for conn, _bag_t, raw in reader.messages(connections=connection(reader, topic, bag)):
            last = (conn, raw)
        if last is None:
            raise ExportError(f'no {topic} message in {bag}')
        msg = reader.deserialize(last[1], last[0].msgtype)
    t = [stamp_ns(ps.header) for ps in msg.poses]
    p = [[ps.pose.position.x, ps.pose.position.y, ps.pose.position.z] for ps in msg.poses]
    q = [[ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w]
         for ps in msg.poses]
    return Trajectory(t, p, q, f'last {topic} message in {bag.name}')


COVERAGE_END_S = 2.0     # warn if the trajectory ends earlier than this before the input bag ends
COVERAGE_START_S = 10.0  # warn if it starts later than this after the input bag starts (initialisation)


def trajectory_coverage(traj, common_bag):
    """Start/end of the trajectory relative to the input bag (None without the common bag)."""
    if common_bag is None or not common_bag.is_file():
        return None
    with AnyReader([common_bag]) as reader:
        start, end = reader.start_time, reader.end_time
    return {'bag_start': format_time_ns(start), 'bag_end': format_time_ns(end),
            'starts_after_bag_s': round((int(traj.t_ns[0]) - start) / NS, 3),
            'ends_before_bag_s': round((end - int(traj.t_ns[-1])) / NS, 3)}


def load_trajectory(spec, run_dir, bag):
    source = spec.get('source')
    if source == 'odometry':
        return trajectory_from_odometry(bag, spec['topic'], spec.get('stamp_from'))
    if source == 'tum_file':
        return trajectory_from_tum(one_file(run_dir, spec['file']))
    if source == 'path_last':
        return trajectory_from_last_path(bag, spec['topic'])
    raise ExportError(f'unknown trajectory source {source!r}')


# ------------------------------------------------------------------ point clouds

PF_DTYPES = {1: 'i1', 2: 'u1', 3: '<i2', 4: '<u2', 5: '<i4', 6: '<u4', 7: '<f4', 8: '<f8'}


def cloud_array(msg, names):
    """Structured view of a PointCloud2 with only the named fields (missing field -> ExportError)."""
    by_name = {f.name: f for f in msg.fields}
    missing = [n for n in names if n not in by_name]
    if missing:
        raise ExportError(f'cloud has no field(s) {missing}; fields: {sorted(by_name)}')
    dtype = np.dtype({'names': list(names),
                      'formats': [PF_DTYPES[by_name[n].datatype] for n in names],
                      'offsets': [by_name[n].offset for n in names],
                      'itemsize': msg.point_step})
    count = msg.width * msg.height
    if count == 0:
        return np.empty(0, dtype=dtype)
    if msg.is_bigendian:
        raise ExportError('big-endian clouds are not supported')
    return np.frombuffer(msg.data, dtype=dtype, count=count)  # rosbags gives data as a uint8 array


def columns(arr, attribute):
    """(N,3) float64 xyz, (N,k) float32 attribute columns and the mask of finite points."""
    xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float64)
    if attribute == 'intensity':
        attr = arr['intensity'].astype(np.float32)[:, None]
    elif attribute == 'rgb':
        packed = np.ascontiguousarray(arr['rgb']).view(np.uint32)
        attr = np.stack([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255], axis=1).astype(np.float32)
    else:
        attr = np.empty((len(arr), 0), dtype=np.float32)
    ok = np.isfinite(xyz).all(axis=1) & np.isfinite(attr).all(axis=1)
    return xyz, attr, ok


def xyz_attr(arr, attribute):
    """Finite points only: (N,3) float64 xyz and (N,k) float32 attributes."""
    xyz, attr, ok = columns(arr, attribute)
    return xyz[ok], attr[ok]


ATTR_FIELDS = {'intensity': ['intensity'], 'rgb': ['rgb'], None: []}   # PointCloud2 / PCD field names
ATTR_COLUMNS = {'intensity': 1, 'rgb': 3, None: 0}                      # attribute columns per point


def attr_fields(attribute):
    if attribute not in ATTR_FIELDS:
        raise ExportError(f'unknown map attribute {attribute!r} (intensity, rgb or null)')
    return ATTR_FIELDS[attribute]


def read_pcd(path):
    """Read a PCD file (ascii or binary) into a structured array."""
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline()
            if not line:
                raise ExportError(f'{path}: no DATA line in the PCD header')
            text = line.decode('ascii', 'replace').strip()
            if not text or text.startswith('#'):
                continue
            key, *values = text.split()
            header[key.upper()] = values
            if key.upper() == 'DATA':
                break
        body = f.read()
    names = header['FIELDS']
    sizes = [int(s) for s in header['SIZE']]
    types = header['TYPE']
    counts = [int(c) for c in header.get('COUNT', ['1'] * len(names))]
    n = int(header['POINTS'][0])
    kind = header['DATA'][0].lower()
    formats = []
    for size, typ, count in zip(sizes, types, counts):
        base = np.dtype({'F': 'f', 'U': 'u', 'I': 'i'}[typ.upper()] + str(size)).newbyteorder('<')
        formats.append((base, count) if count > 1 else base)
    dtype = np.dtype({'names': names, 'formats': formats})
    if kind == 'binary':
        if len(body) < n * dtype.itemsize:
            raise ExportError(f'{path}: truncated binary data')
        return np.frombuffer(body, dtype=dtype, count=n)
    if kind == 'ascii':
        if any(c > 1 for c in counts):
            raise ExportError(f'{path}: ascii PCD with COUNT > 1 is not supported')
        table = np.loadtxt(body.decode('ascii').splitlines(), dtype=np.float64, ndmin=2)
        if table.shape != (n, len(names)):
            raise ExportError(f'{path}: expected {n} x {len(names)} values, got {table.shape}')
        out = np.empty(n, dtype=dtype)
        for i, name in enumerate(names):
            out[name] = table[:, i]
        return out
    raise ExportError(f'{path}: PCD DATA {kind} is not supported')


# ------------------------------------------------------------------ map accumulation

class PlyWriter:
    """Streaming binary PLY writer; the vertex count is patched in at close()."""

    COUNT_WIDTH = 12  # zero-padded so it can be rewritten in place

    def __init__(self, path, attribute, comments=()):
        props = [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]
        if attribute == 'intensity':
            props.append(('intensity', '<f4'))
        elif attribute == 'rgb':
            props += [('red', 'u1'), ('green', 'u1'), ('blue', 'u1')]
        self.dtype = np.dtype(props)
        self.attribute = attribute
        ply_type = {'<f4': 'float', 'u1': 'uchar'}
        lines = ['ply', 'format binary_little_endian 1.0']
        lines += [f'comment {c}' for c in comments]
        lines.append(f'element vertex {0:0{self.COUNT_WIDTH}d}')
        lines += [f'property {ply_type[t]} {n}' for n, t in props]
        lines.append('end_header')
        header = ('\n'.join(lines) + '\n').encode('ascii')
        self.count_pos = header.index(b'element vertex ') + len(b'element vertex ')
        self.header_len = len(header)
        self.f = open(path, 'wb')
        self.f.write(header)
        self.count = 0

    def write(self, xyz, attr):
        rec = np.empty(len(xyz), dtype=self.dtype)
        rec['x'], rec['y'], rec['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        if self.attribute == 'intensity':
            rec['intensity'] = attr[:, 0]
        elif self.attribute == 'rgb':
            rgb = np.clip(np.rint(attr), 0, 255).astype(np.uint8)
            rec['red'], rec['green'], rec['blue'] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        rec.tofile(self.f)
        self.count += len(rec)

    def close(self):
        self.f.seek(self.count_pos)
        self.f.write(f'{self.count:0{self.COUNT_WIDTH}d}'.encode('ascii'))
        self.f.close()


def verify_ply(path, expected):
    """Re-read the PLY: header count, file size, finite coordinates and bounding box."""
    with open(path, 'rb') as f:
        head = f.read(4096)
    end = head.index(b'end_header\n') + len(b'end_header\n')
    lines = head[:end].decode('ascii').splitlines()
    count = int(next(line for line in lines if line.startswith('element vertex')).split()[2])
    props = [line.split() for line in lines if line.startswith('property')]
    dtype = np.dtype([(p[2], {'float': '<f4', 'uchar': 'u1'}[p[1]]) for p in props])
    if count != expected:
        raise ExportError(f'{path.name}: header has {count} vertices, expected {expected}')
    if os.path.getsize(path) != end + count * dtype.itemsize:
        raise ExportError(f'{path.name}: size does not match {count} vertices')
    data = np.memmap(path, dtype=dtype, mode='r', offset=end, shape=(count,))
    lo, hi = [], []
    for axis in 'xyz':
        column = np.asarray(data[axis])
        if not np.isfinite(column).all():
            raise ExportError(f'{path.name}: non-finite {axis} coordinates')
        lo.append(float(column.min()))
        hi.append(float(column.max()))
    return lo, hi


class MapBuilder:
    """Collects world-frame points: voxel centroids (voxel > 0) or every point streamed to the PLY."""

    BITS = 21                 # per axis: +-2^20 voxels (+-52 km at 5 cm)
    FLUSH_POINTS = 20_000_000
    PREVIEW_STRIDE = 20       # full-resolution mode: every n-th point is kept for the preview

    def __init__(self, voxel, attribute, ply_path, comments):
        self.voxel = voxel
        self.attribute = attribute
        self.ply = PlyWriter(ply_path, attribute, comments)
        self.n_attr = ATTR_COLUMNS[attribute]
        self.points_in = 0
        self.buf, self.buf_n = [], 0
        self.keys = np.empty(0, dtype=np.int64)
        self.sums = np.empty((0, 3 + self.n_attr), dtype=np.float64)
        self.counts = np.empty(0, dtype=np.int64)
        self.preview = []  # subsample for the preview in full-resolution mode

    def add(self, xyz, attr):
        if len(xyz) == 0:
            return
        self.points_in += len(xyz)
        if self.voxel <= 0:
            self.ply.write(xyz, attr)
            self.preview.append(xyz[::self.PREVIEW_STRIDE].astype(np.float32))
            return
        idx = np.floor(xyz / self.voxel).astype(np.int64) + (1 << (self.BITS - 1))
        if idx.min() < 0 or idx.max() >= (1 << self.BITS):
            raise ExportError(f'points beyond +-{(1 << (self.BITS - 1)) * self.voxel:.0f} m: increase --voxel-size')
        keys = (idx[:, 0] << (2 * self.BITS)) | (idx[:, 1] << self.BITS) | idx[:, 2]
        self.buf.append((keys, np.hstack([xyz, attr.astype(np.float64)])))
        self.buf_n += len(keys)
        if self.buf_n >= self.FLUSH_POINTS:
            self._flush()

    def _flush(self):
        if not self.buf:
            return
        keys = np.concatenate([self.keys] + [k for k, _ in self.buf])
        vals = np.concatenate([self.sums] + [v for _, v in self.buf])
        counts = np.concatenate([self.counts, np.ones(self.buf_n, dtype=np.int64)])
        self.buf, self.buf_n = [], 0
        uniq, inv = np.unique(keys, return_inverse=True)
        self.keys = uniq
        self.counts = np.bincount(inv, weights=counts, minlength=len(uniq)).astype(np.int64)
        self.sums = np.stack([np.bincount(inv, weights=vals[:, i], minlength=len(uniq))
                              for i in range(vals.shape[1])], axis=1)

    def finish(self):
        """Write the voxel centroids (if any), close the PLY; return (points_out, preview xyz)."""
        if self.voxel > 0:
            self._flush()
            mean = self.sums / self.counts[:, None]
            xyz = mean[:, :3]
            self.ply.write(xyz, mean[:, 3:].astype(np.float32))
            preview = xyz
        else:
            preview = np.concatenate(self.preview) if self.preview else np.empty((0, 3))
        self.ply.close()
        return self.ply.count, preview


def quat_rotate(q, v):
    """Rotate vectors v (N,3) by unit quaternions q (N,4, xyzw)."""
    u, w = q[:, :3], q[:, 3:4]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def map_accumulate(builder, bag, topic, attribute):
    fields = ['x', 'y', 'z'] + attr_fields(attribute)
    messages = empty = 0
    with AnyReader([bag]) as reader:
        for conn, _bag_t, raw in reader.messages(connections=connection(reader, topic, bag)):
            msg = reader.deserialize(raw, conn.msgtype)
            messages += 1
            if msg.width * msg.height == 0:
                empty += 1
                continue
            builder.add(*xyz_attr(cloud_array(msg, fields), attribute))
    if messages == empty:
        raise ExportError(f'no points in {topic}')
    notes = [f'{empty} empty message(s) skipped'] if empty else []
    return {'source': f'{topic} in {bag.name}', 'messages': messages}, notes


def map_from_pcd(builder, path, attribute):
    arr = read_pcd(path)
    builder.add(*xyz_attr(arr, attribute))
    return {'source': f'file {path.name}'}, []


def map_reconstruct(builder, common_bag, spec, run_dir, traj):
    """Place the common bag's LiDAR scans with the trajectory, interpolated at every point's time."""
    if common_bag is None or not common_bag.is_file():
        raise ExportError(f'map reconstruction needs the common bag (--common-bag), not found: {common_bag}')
    ext = resolve(spec['extrinsic'], run_dir)
    r_bl = np.asarray(ext['Rot'], dtype=np.float64).reshape(3, 3)
    t_bl = np.asarray(ext['Trans'], dtype=np.float64).reshape(3)
    min_range = float(resolve(spec.get('min_range', 0.0), run_dir))
    max_range = float(resolve(spec.get('max_range', 1e9), run_dir))
    time_field = spec.get('time_field', 'time')
    attribute = spec.get('attribute', 'intensity')
    fields = ['x', 'y', 'z', time_field] + attr_fields(attribute)

    t0 = traj.t_ns[0]
    tt = (traj.t_ns - t0) / NS
    scans = out_of_range = outside = 0
    with AnyReader([common_bag]) as reader:
        for conn, _bag_t, raw in reader.messages(connections=connection(reader, spec['topic'], common_bag)):
            msg = reader.deserialize(raw, conn.msgtype)
            arr = cloud_array(msg, fields)
            if len(arr) == 0:
                continue
            scans += 1
            xyz, attr, finite = columns(arr, attribute)
            t = (stamp_ns(msg.header) - t0) / NS + arr[time_field].astype(np.float64)  # header = scan start
            rng = np.linalg.norm(np.where(finite[:, None], xyz, 0.0), axis=1)
            keep = finite & (rng >= min_range) & (rng <= max_range)
            out_of_range += int((finite & ~keep).sum())
            inside = (t >= tt[0]) & (t <= tt[-1])
            outside += int((keep & ~inside).sum())
            keep &= inside
            if not keep.any():
                continue
            xyz, attr, t = xyz[keep], attr[keep], t[keep]
            i = np.clip(np.searchsorted(tt, t, side='right') - 1, 0, len(tt) - 2)
            a = ((t - tt[i]) / (tt[i + 1] - tt[i]))[:, None]
            pos = traj.p[i] + a * (traj.p[i + 1] - traj.p[i])
            q0, q1 = traj.q[i], traj.q[i + 1].copy()
            q1[(q0 * q1).sum(axis=1) < 0] *= -1          # shortest path
            q = q0 + a * (q1 - q0)                      # nlerp: poses are 10 ms apart
            q /= np.linalg.norm(q, axis=1, keepdims=True)
            body = xyz @ r_bl.T + t_bl
            builder.add(quat_rotate(q, body) + pos, attr)
    if scans == 0:
        raise ExportError(f"no scans in {spec['topic']} of {common_bag}")
    notes = [f'{scans} scans placed with the exported trajectory (per-point time, linear/nlerp interpolation)',
             f'{out_of_range} points outside [{min_range}, {max_range}] m dropped (the algorithm\'s own range limits)',
             f'{outside} points outside the trajectory time span dropped']
    return {'source': f"{spec['topic']} in {common_bag.name} + the exported trajectory",
            'extrinsic_body_lidar': {'Rot': r_bl.ravel().tolist(), 'Trans': t_bl.tolist()},
            'scans': scans}, notes


# ------------------------------------------------------------------ preview

def preview_panel(pts, tp, lo, hi, scale, label, margin=40):
    """One orthographic view: map point density (grey) and the trajectory (red; start green, end blue)."""
    span = hi - lo
    w, h = (span * scale).astype(int) + 2 * margin

    def to_px(xy):
        return np.stack([(xy[:, 0] - lo[0]) * scale + margin, h - ((xy[:, 1] - lo[1]) * scale + margin)], axis=1)

    img = np.full((h, w), 255, dtype=np.uint8)
    if len(pts):
        uv = np.rint(to_px(pts)).astype(np.int64)
        ok = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
        dens = np.log1p(np.bincount(uv[ok, 1] * w + uv[ok, 0], minlength=w * h).reshape(h, w).astype(np.float64))
        if dens.max() > 0:
            img = (255 - 230 * dens / dens.max()).astype(np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    line = np.rint(to_px(tp)).astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(img, [line], False, (0, 0, 220), 2, cv2.LINE_AA)
    cv2.circle(img, tuple(int(c) for c in line[0, 0]), 7, (0, 170, 0), -1, cv2.LINE_AA)
    cv2.circle(img, tuple(int(c) for c in line[-1, 0]), 7, (220, 80, 0), -1, cv2.LINE_AA)
    cv2.putText(img, label, (margin, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
    return img


def write_preview(path, xyz, traj, title, max_side=1600, crop=25.0, margin=40):
    """Top view (world XY) and side view (world XZ) around the trajectory (+crop m), same scale."""
    pts = np.asarray(xyz, dtype=np.float64)
    if len(pts) > 6_000_000:
        pts = pts[:: len(pts) // 6_000_000 + 1]
    lo, hi = traj.p.min(axis=0) - crop, traj.p.max(axis=0) + crop
    if len(pts):  # no further than the map reaches
        lo = np.minimum(np.maximum(lo, np.percentile(pts, 1, axis=0)), traj.p.min(axis=0) - 1)
        hi = np.maximum(np.minimum(hi, np.percentile(pts, 99, axis=0)), traj.p.max(axis=0) + 1)
    pts = pts[((pts >= lo) & (pts <= hi)).all(axis=1)] if len(pts) else pts
    scale = (max_side - 2 * margin) / max(hi[0] - lo[0], hi[1] - lo[1])   # px per m
    top = preview_panel(pts[:, :2], traj.p[:, :2], lo[:2], hi[:2], scale, 'top view (world XY)', margin)
    side = preview_panel(pts[:, [0, 2]], traj.p[:, [0, 2]], lo[[0, 2]], hi[[0, 2]], scale,
                         'side view (world XZ)', margin)
    img = np.vstack([top, np.full((4, top.shape[1], 3), 160, dtype=np.uint8), side])
    header = np.full((50, img.shape[1], 3), 255, dtype=np.uint8)
    cv2.putText(header, title, (margin, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2, cv2.LINE_AA)
    bar_m = 10.0 if (hi - lo).max() > 30 else 1.0
    cv2.line(header, (img.shape[1] - margin - int(bar_m * scale), 30), (img.shape[1] - margin, 30), (0, 0, 0), 3)
    cv2.putText(header, f'{bar_m:g} m', (img.shape[1] - margin - int(bar_m * scale), 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.imwrite(str(path), np.vstack([header, img]))


# ------------------------------------------------------------------ main

def export(args):
    run_dir = args.run.resolve()
    spec = yaml.safe_load(args.spec.read_text())
    bag = run_dir / f'recorded-{args.id}.bag'
    final = run_dir / 'export'
    work = run_dir / 'export.partial'
    if work.exists():
        shutil.rmtree(work)
    work.mkdir()
    warnings = []
    info = {'algorithm': args.id, 'sequence': args.sequence,
            'created': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds'),
            'frames': spec.get('frames', {}), 'files': {}}

    # Trajectory
    log(f"trajectory: {spec['trajectory']}")
    traj = load_trajectory(spec['trajectory'], run_dir, bag).check()
    name = f'{args.id}_trajectory_tum.txt'
    traj.write_tum(work / name)
    info['files']['trajectory'] = name
    info['trajectory'] = traj.stats()
    log(f"  {info['trajectory']['poses']} poses, {info['trajectory']['duration_s']} s, "
        f"{info['trajectory']['length_m']} m -> {name}")
    coverage = trajectory_coverage(traj, args.common_bag)
    if coverage:
        info['trajectory']['coverage'] = coverage
        log(f"  starts {coverage['starts_after_bag_s']} s after the bag start, "
            f"ends {coverage['ends_before_bag_s']} s before the bag end")
        if coverage['ends_before_bag_s'] > COVERAGE_END_S:
            warnings.append(f"the trajectory ends {coverage['ends_before_bag_s']} s before the end of the input bag: "
                            f"the algorithm fell behind real time or stopped early, the rest is missing")
        if coverage['starts_after_bag_s'] > COVERAGE_START_S:
            warnings.append(f"the trajectory starts {coverage['starts_after_bag_s']} s after the start of the input bag")

    info['extra_trajectories'] = {}
    for extra_name, extra_spec in (spec.get('extra_trajectories') or {}).items():
        log(f'extra trajectory {extra_name}: {extra_spec}')
        extra = load_trajectory(extra_spec, run_dir, bag).check()
        fname = f'{args.id}_{extra_name}_trajectory_tum.txt'
        extra.write_tum(work / fname)
        info['files'][f'trajectory_{extra_name}'] = fname
        info['extra_trajectories'][extra_name] = extra.stats()
        log(f'  {len(extra.t_ns)} poses -> {fname}')

    # Map
    mspec = spec['map']
    attribute = mspec.get('attribute', 'intensity')
    name = f'{args.id}_map.ply'
    log(f'map: {mspec} (voxel {args.voxel_size} m)')
    comments = [f'LVIO benchmark export: {args.id} on {args.sequence}',
                f"frame {spec.get('frames', {}).get('world', '?')}",
                f'voxel {args.voxel_size} m' if args.voxel_size > 0 else 'full resolution']
    builder = MapBuilder(args.voxel_size, attribute, work / name, comments)
    source = mspec.get('source')
    if source == 'accumulate':
        meta, notes = map_accumulate(builder, bag, mspec['topic'], attribute)
        kind = 'accumulated'
    elif source == 'pcd':
        meta, notes = map_from_pcd(builder, run_dir / mspec['file'], attribute)
        kind = 'native'
    elif source == 'reconstruct':
        meta, notes = map_reconstruct(builder, args.common_bag, mspec, run_dir, traj)
        kind = 'reconstructed'
    else:
        raise ExportError(f'unknown map source {source!r}')
    points_out, preview = builder.finish()
    if points_out == 0:
        raise ExportError('the map is empty')
    bbox_min, bbox_max = verify_ply(work / name, points_out)
    info['files']['map'] = name
    info['map'] = {'kind': kind, 'description': mspec.get('description', ''), **meta, 'attribute': attribute,
                   'voxel_size_m': args.voxel_size, 'points_in': int(builder.points_in),
                   'points_out': int(points_out),
                   'bbox_min': [round(v, 3) for v in bbox_min], 'bbox_max': [round(v, 3) for v in bbox_max],
                   'notes': notes}
    log(f'  {builder.points_in} points -> {points_out} -> {name}')

    preview_name = f'{args.id}_preview.png'
    write_preview(work / preview_name, preview, traj,
                  f'{args.id} / {args.sequence}: {kind} map, {points_out:,} pts, '
                  f"trajectory {info['trajectory']['length_m']:.0f} m")
    info['files']['preview'] = preview_name

    # Cross-checks between map and trajectory
    margin = 5.0
    if (traj.p.min(axis=0) < np.asarray(bbox_min) - margin).any() or \
            (traj.p.max(axis=0) > np.asarray(bbox_max) + margin).any():
        warnings.append('the trajectory leaves the map bounding box by more than 5 m (different frames?)')
    info['warnings'] = warnings
    (work / 'export.json').write_text(json.dumps(info, indent=2) + '\n')

    old = run_dir / 'export.old'
    if final.exists():
        final.rename(old)
    work.rename(final)
    if old.exists():
        shutil.rmtree(old)
    for w in warnings:
        log(f'WARNING: {w}')
    log(f'OK -> {final}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--run', type=Path, required=True, help='result folder of one run')
    parser.add_argument('--spec', type=Path, required=True, help='algorithms/<ALGO>/export.yaml')
    parser.add_argument('--id', required=True, help='algorithm id (file names)')
    parser.add_argument('--sequence', required=True, help='sequence id (metadata)')
    parser.add_argument('--common-bag', type=Path, help='common bag of the sequence (map source "reconstruct")')
    parser.add_argument('--voxel-size', type=float, default=0.05, help='map voxel in m; 0 = every point')
    args = parser.parse_args()
    start = time.time()
    try:
        export(args)
    except (ExportError, TrajectoryError) as err:
        log(f'FAILED: {err}')
        shutil.rmtree(args.run / 'export.partial', ignore_errors=True)
        return 1
    except KeyboardInterrupt:
        log('interrupted')
        shutil.rmtree(args.run / 'export.partial', ignore_errors=True)
        return 130
    peak_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024  # Linux: KiB
    log(f'done in {time.time() - start:.0f} s, peak memory {peak_mb:.0f} MB')
    return 0


if __name__ == '__main__':
    sys.exit(main())
