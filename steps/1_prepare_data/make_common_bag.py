#!/usr/bin/env python3
"""Step 1: build the LVIO common ROS 1 bag for one sequence.

COCO (Newer College Multi-Cam 2021) is described below; BUNKER (Bunker DVI) is in bunker_common_bag.py
(selected with --dataset BUNKER).

The common bag is the single input every (ROS 1) algorithm reads (see README.md). It contains:

  copied unchanged
    /os_cloud_node/points                   Ouster OS0-128, organized 128 x 1024
    /os_cloud_node/imu                      Ouster IMU, 100 Hz
    /alphasense_driver_ros/imu              Alphasense IMU, 200 Hz
    /alphasense_driver_ros/cam0/compressed  cam0 fisheye JPEG, 30 Hz
  derived
    /lvio/points_xyzirt                     the Ouster cloud in Velodyne layout:
                                            x y z intensity (f32), ring (u16), time (f32, s from scan start),
                                            zero returns removed, points sorted by time
    /lvio/cam0_rect/compressed              cam0 rectified to a pinhole camera without distortion (JPEG)

All other topics (raw Ouster packets, cam1/3/4) are dropped. Derived messages keep the header
(stamp, frame_id) and bag time of their source message. The output is written to
<output>.partial and renamed only on success.

The bag is written UNCOMPRESSED. rosbags' LZ4 chunks use python-lz4's default frame format
(linked blocks, content size, no content checksum), which ROS 1's roslz4 rejects
("ROSLZ4_DATA_ERROR: malformed data to decompress", ros_comm utilities/roslz4/src/lz4s.c).
Uncompressed bags are readable by every ROS 1 version and need no decompression during playback.

Requires: rosbags, numpy, opencv-python-headless, pyyaml (no ROS installation).
"""

import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from rosbags.rosbag1 import Reader, Writer
from rosbags.typesys import Stores, get_typestore

CLOUD_IN = '/os_cloud_node/points'
COPY_TOPICS = [
    CLOUD_IN,
    '/os_cloud_node/imu',
    '/alphasense_driver_ros/imu',
    '/alphasense_driver_ros/cam0/compressed',
]
CLOUD_OUT = '/lvio/points_xyzirt'

# sensor_msgs/PointField datatypes
PF_UINT8, PF_UINT16, PF_UINT32, PF_FLOAT32 = 2, 4, 6, 7
XYZIRT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
                         ('ring', '<u2'), ('time', '<f4')])  # packed, 22 bytes (as in the LVI-SAM dataset)
REQUIRED_FIELDS = {'x': (PF_FLOAT32,), 'y': (PF_FLOAT32,), 'z': (PF_FLOAT32,),
                   'intensity': (PF_FLOAT32,), 't': (PF_UINT32,), 'ring': (PF_UINT8, PF_UINT16)}
NP_TYPES = {PF_UINT8: '<u1', PF_UINT16: '<u2', PF_UINT32: '<u4', PF_FLOAT32: '<f4'}

TS = get_typestore(Stores.ROS1_NOETIC)
PointField = TS.types['sensor_msgs/msg/PointField']
PointCloud2 = TS.types['sensor_msgs/msg/PointCloud2']
CompressedImage = TS.types['sensor_msgs/msg/CompressedImage']


def xyzirt_cloud(msg):
    """Ouster PointCloud2 (fields x y z intensity t ring ...) -> Velodyne-layout PointCloud2."""
    fields = {f.name: f for f in msg.fields}
    for name, types in REQUIRED_FIELDS.items():
        if name not in fields or fields[name].datatype not in types:
            raise ValueError(f'{CLOUD_IN}: field {name!r} missing or of unexpected type')
    if msg.is_bigendian:
        raise ValueError(f'{CLOUD_IN}: big-endian clouds are not supported')

    n = msg.width * msg.height
    raw = np.asarray(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

    def column(name):
        f = fields[name]
        dt = np.dtype(NP_TYPES[f.datatype])
        return raw[:, f.offset:f.offset + dt.itemsize].copy().view(dt).ravel()

    x, y, z = column('x'), column('y'), column('z')
    keep = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & ~((x == 0) & (y == 0) & (z == 0))
    t_ns = column('t')[keep]
    order = np.argsort(t_ns, kind='stable')  # firing order; stable keeps ring order within a column

    out = np.empty(int(keep.sum()), dtype=XYZIRT_DTYPE)
    out['x'] = x[keep][order]
    out['y'] = y[keep][order]
    out['z'] = z[keep][order]
    out['intensity'] = column('intensity')[keep][order]
    out['ring'] = column('ring')[keep][order]
    out['time'] = (t_ns[order] * 1e-9).astype(np.float32)

    out_fields = [PointField(name=name, offset=XYZIRT_DTYPE.fields[name][1], datatype=dtype, count=1)
                  for name, dtype in (('x', PF_FLOAT32), ('y', PF_FLOAT32), ('z', PF_FLOAT32),
                                      ('intensity', PF_FLOAT32), ('ring', PF_UINT16), ('time', PF_FLOAT32))]
    return PointCloud2(header=msg.header, height=1, width=len(out), fields=out_fields, is_bigendian=False,
                       point_step=XYZIRT_DTYPE.itemsize, row_step=XYZIRT_DTYPE.itemsize * len(out),
                       data=out.view(np.uint8).ravel(), is_dense=True)


class Rectifier:
    """Equidistant (Kalibr pinhole-equidistant) -> undistorted pinhole, same size, rotation = I."""

    def __init__(self, calib):
        cam, rect = calib['cam0'], calib['cam0_rect']
        fx, fy, cx, cy = cam['intrinsics']
        self.size = tuple(cam['resolution'])
        if cam['distortion_model'] != 'equidistant':
            raise ValueError('cam0 distortion_model must be equidistant')
        k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        d = np.array(cam['distortion_coeffs'], dtype=np.float64).reshape(4, 1)
        rfx, rfy, rcx, rcy = rect['intrinsics']
        k_new = np.array([[rfx, 0, rcx], [0, rfy, rcy], [0, 0, 1]], dtype=np.float64)
        rect_size = tuple(rect['resolution'])
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(k, d, np.eye(3), k_new, rect_size, cv2.CV_16SC2)
        self.topic_in, self.topic_out = cam['topic'], rect['topic']
        self.params = [int(cv2.IMWRITE_JPEG_QUALITY), int(rect['jpeg_quality'])]
        valid = cv2.remap(np.full(self.size[::-1], 255, np.uint8), self.map1, self.map2, cv2.INTER_NEAREST,
                          borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        self.valid_fraction = float((valid > 0).mean())

    def __call__(self, msg):
        img = cv2.imdecode(np.asarray(msg.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        if img is None or img.shape[:2] != self.size[::-1]:
            raise ValueError(f'{self.topic_in}: cannot decode image or unexpected size')
        rect = cv2.remap(img, self.map1, self.map2, cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        ok, enc = cv2.imencode('.jpg', rect, self.params)
        if not ok:
            raise ValueError(f'{self.topic_out}: JPEG encoding failed')
        encoding = 'mono8' if rect.ndim == 2 else 'bgr8'
        return CompressedImage(header=msg.header, format=f'{encoding}; jpeg compressed', data=enc.ravel())


def convert(src, dst, calib, duration=None):
    rectifier = Rectifier(calib)
    partial = dst.with_name(dst.name + '.partial')
    if partial.exists():
        partial.unlink()
    counts = {}
    t_wall = time.time()

    with Reader(src) as reader:
        conns = [c for c in reader.connections if c.topic in COPY_TOPICS]
        missing = sorted(set(COPY_TOPICS) - {c.topic for c in conns})
        if missing:
            raise SystemExit(f'ERROR: {src.name} lacks topics: {", ".join(missing)}')
        t_end = reader.start_time + int(duration * 1e9) if duration else None
        total_s = ((t_end or reader.end_time) - reader.start_time) / 1e9

        writer = Writer(partial)  # uncompressed on purpose, see the module docstring
        with writer:
            out = {}
            for c in conns:  # one output connection per topic, with the source definition
                if c.topic not in out:
                    out[c.topic] = writer.add_connection(c.topic, c.msgtype, msgdef=c.msgdef.data, md5sum=c.digest,
                                                         callerid=c.ext.callerid, latching=c.ext.latching)
            out[CLOUD_OUT] = writer.add_connection(CLOUD_OUT, 'sensor_msgs/msg/PointCloud2', typestore=TS,
                                                   callerid='/lvio_make_common_bag')
            out[rectifier.topic_out] = writer.add_connection(rectifier.topic_out, 'sensor_msgs/msg/CompressedImage',
                                                             typestore=TS, callerid='/lvio_make_common_bag')

            last_report = 0.0
            for conn, stamp, raw in reader.messages(connections=conns, stop=t_end):
                writer.write(out[conn.topic], stamp, raw)
                counts[conn.topic] = counts.get(conn.topic, 0) + 1
                if conn.topic == CLOUD_IN:
                    msg = TS.deserialize_ros1(raw, conn.msgtype)
                    writer.write(out[CLOUD_OUT], stamp, TS.serialize_ros1(xyzirt_cloud(msg), PointCloud2.__msgtype__))
                    counts[CLOUD_OUT] = counts.get(CLOUD_OUT, 0) + 1
                elif conn.topic == rectifier.topic_in:
                    msg = TS.deserialize_ros1(raw, conn.msgtype)
                    writer.write(out[rectifier.topic_out], stamp,
                                 TS.serialize_ros1(rectifier(msg), CompressedImage.__msgtype__))
                    counts[rectifier.topic_out] = counts.get(rectifier.topic_out, 0) + 1
                if time.time() - last_report > 10:
                    last_report = time.time()
                    done = (stamp - reader.start_time) / 1e9
                    print(f'  {done:7.1f} / {total_s:.1f} s of bag time', flush=True)

    os.replace(partial, dst)
    print(f'Wrote {dst} ({dst.stat().st_size / 1e9:.2f} GB) in {time.time() - t_wall:.0f} s; '
          f'rectified valid pixels {100 * rectifier.valid_fraction:.1f} %')
    for topic in sorted(counts):
        print(f'  {topic:42s} {counts[topic]:7d} msgs')
    if counts.get(CLOUD_OUT) != counts.get(CLOUD_IN) or counts.get(rectifier.topic_out) != counts.get(rectifier.topic_in):
        raise SystemExit('ERROR: derived topic counts differ from their sources')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('src', type=Path, help='original bag')
    parser.add_argument('dst', type=Path, help='common bag to write')
    parser.add_argument('--dataset', default='COCO', choices=['COCO', 'BUNKER'], help='recipe to use')
    parser.add_argument('--calib', type=Path, default=None, help='default: <dataset>_calibration.yaml beside this script')
    parser.add_argument('--duration', type=float, default=None,
                        help='convert only the first N seconds (quick tests)')
    args = parser.parse_args()
    if args.dst.exists():
        sys.exit(f'ERROR: {args.dst} exists')
    calib_path = args.calib or Path(__file__).with_name(f'{args.dataset.lower()}_calibration.yaml')
    with open(calib_path) as handle:
        calib = yaml.safe_load(handle)
    if args.dataset == 'BUNKER':
        import bunker_common_bag   # beside this script
        bunker_common_bag.convert(args.src, args.dst, calib, TS, args.duration)
    else:
        convert(args.src, args.dst, calib, args.duration)


if __name__ == '__main__':
    main()
