"""Step 1 recipe for the BUNKER dataset (Bunker DVI, reg-1): called by make_common_bag.py --dataset BUNKER.

  copied unchanged
    /livox/lidar                        Livox Mid-360, livox_ros_driver2/CustomMsg, 10 Hz
    /livox/imu                          Livox Mid-360 built-in IMU, 200 Hz, acceleration in g
    /d455f/color/image_raw/compressed   D455F colour, RGB JPEG 640 x 480, 15 Hz
    /d455f/color/camera_info            its factory intrinsics (plumb_bob)
  derived
    /lvio/livox/lidar                   the same scans as livox_ros_driver/CustomMsg: the ROS 1 algorithms are
                                        built against livox_ros_driver; the fields (and bytes) are identical
    /lvio/points_xyzirt                 the scans as PointCloud2 x y z intensity (f32), ring (u16 = Livox line),
                                        time (f32, s from the header stamp = scan start), sorted by time,
                                        zero points removed
    /lvio/d455_rect/compressed          D455F colour undistorted to a pinhole camera (bunker_calibration.yaml
                                        camera_rect), JPEG

Every other topic (Alphasense, Kinect, depth streams, other IMUs) is dropped. The Livox scans are parsed
from their ROS 1 bytes with numpy (CustomPoint: u32 offset_time, 3 x f32, 3 x u8 = 19 bytes, packed).
"""

import os
import time

import cv2
import numpy as np
from rosbags.rosbag1 import Reader, Writer
from rosbags.typesys import get_types_from_msg

LIDAR_IN = '/livox/lidar'
IMU_IN = '/livox/imu'
CAMERA_IN = '/d455f/color/image_raw/compressed'
CAMERA_INFO = '/d455f/color/camera_info'
COPY_TOPICS = [LIDAR_IN, IMU_IN, CAMERA_IN, CAMERA_INFO]
LIVOX_V1_OUT = '/lvio/livox/lidar'
CLOUD_OUT = '/lvio/points_xyzirt'

CUSTOM_POINT_DTYPE = np.dtype([('offset_time', '<u4'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                               ('reflectivity', 'u1'), ('tag', 'u1'), ('line', 'u1')])   # packed, 19 bytes
XYZIRT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
                         ('ring', '<u2'), ('time', '<f4')])                               # as the COCO recipe
PF_UINT16, PF_FLOAT32 = 4, 7

LIVOX_V1_TYPES = {
    'livox_ros_driver/msg/CustomPoint':
        'uint32 offset_time\nfloat32 x\nfloat32 y\nfloat32 z\nuint8 reflectivity\nuint8 tag\nuint8 line\n',
    'livox_ros_driver/msg/CustomMsg':
        'std_msgs/Header header\nuint64 timebase\nuint32 point_num\nuint8 lidar_id\nuint8[3] rsvd\n'
        'livox_ros_driver/CustomPoint[] points\n',
}


def parse_custom_msg(raw):
    """ROS 1 bytes of a Livox CustomMsg -> (seq, sec, nsec, frame_id, timebase_ns, points array)."""
    buf = memoryview(raw)
    seq, sec, nsec, flen = np.frombuffer(buf[:16], dtype='<u4')
    frame_id = bytes(buf[16:16 + flen]).decode()
    o = 16 + int(flen)
    timebase = int(np.frombuffer(buf[o:o + 8], dtype='<u8')[0])
    o += 8 + 4 + 1 + 3                                   # timebase, point_num, lidar_id, rsvd[3]
    n = int(np.frombuffer(buf[o:o + 4], dtype='<u4')[0])
    o += 4
    pts = np.frombuffer(buf[o:o + n * CUSTOM_POINT_DTYPE.itemsize], dtype=CUSTOM_POINT_DTYPE)
    return int(seq), int(sec), int(nsec), frame_id, timebase, pts


def xyzirt_cloud(ts, raw):
    seq, sec, nsec, frame_id, timebase, pts = parse_custom_msg(raw)
    keep = (np.isfinite(pts['x']) & np.isfinite(pts['y']) & np.isfinite(pts['z'])
            & ~((pts['x'] == 0) & (pts['y'] == 0) & (pts['z'] == 0)))
    pts = pts[keep]
    t = (timebase + pts['offset_time'].astype(np.int64) - (sec * 1_000_000_000 + nsec)) * 1e-9
    order = np.argsort(t, kind='stable')
    out = np.empty(len(pts), dtype=XYZIRT_DTYPE)
    for name in ('x', 'y', 'z'):
        out[name] = pts[name][order]
    out['intensity'] = pts['reflectivity'][order]
    out['ring'] = pts['line'][order]
    out['time'] = t[order].astype(np.float32)
    PointField, PointCloud2 = ts.types['sensor_msgs/msg/PointField'], ts.types['sensor_msgs/msg/PointCloud2']
    Header, Time = ts.types['std_msgs/msg/Header'], ts.types['builtin_interfaces/msg/Time']
    fields = [PointField(name=name, offset=XYZIRT_DTYPE.fields[name][1], datatype=dtype, count=1)
              for name, dtype in (('x', PF_FLOAT32), ('y', PF_FLOAT32), ('z', PF_FLOAT32),
                                  ('intensity', PF_FLOAT32), ('ring', PF_UINT16), ('time', PF_FLOAT32))]
    msg = PointCloud2(header=Header(seq=seq, stamp=Time(sec=sec, nanosec=nsec), frame_id=frame_id),
                      height=1, width=len(out), fields=fields, is_bigendian=False,
                      point_step=XYZIRT_DTYPE.itemsize, row_step=XYZIRT_DTYPE.itemsize * len(out),
                      data=out.view(np.uint8).ravel(), is_dense=True)
    return ts.serialize_ros1(msg, 'sensor_msgs/msg/PointCloud2')


class Rectifier:
    """plumb_bob colour camera -> undistorted pinhole (bunker_calibration.yaml camera / camera_rect)."""

    def __init__(self, calib, ts):
        cam, rect = calib['camera'], calib['camera_rect']
        fx, fy, cx, cy = cam['intrinsics']
        k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        d = np.array(cam['distortion_coeffs'], dtype=np.float64)
        rfx, rfy, rcx, rcy = rect['intrinsics']
        k_new = np.array([[rfx, 0, rcx], [0, rfy, rcy], [0, 0, 1]], dtype=np.float64)
        self.size = tuple(cam['resolution'])
        self.map1, self.map2 = cv2.initUndistortRectifyMap(k, d, None, k_new, tuple(rect['resolution']), cv2.CV_16SC2)
        self.topic_out = rect['topic']
        self.params = [int(cv2.IMWRITE_JPEG_QUALITY), int(rect['jpeg_quality'])]
        self.ts = ts

    def __call__(self, raw, msgtype):
        msg = self.ts.deserialize_ros1(raw, msgtype)
        img = cv2.imdecode(np.asarray(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None or img.shape[:2] != self.size[::-1]:
            raise ValueError(f'{CAMERA_IN}: cannot decode image or unexpected size')
        ok, enc = cv2.imencode('.jpg', cv2.remap(img, self.map1, self.map2, cv2.INTER_LINEAR), self.params)
        if not ok:
            raise ValueError(f'{self.topic_out}: JPEG encoding failed')
        CompressedImage = self.ts.types['sensor_msgs/msg/CompressedImage']
        out = CompressedImage(header=msg.header, format='bgr8; jpeg compressed bgr8', data=enc.ravel())
        return self.ts.serialize_ros1(out, 'sensor_msgs/msg/CompressedImage')


def convert(src, dst, calib, ts, duration=None):
    for name, text in LIVOX_V1_TYPES.items():
        ts.register(get_types_from_msg(text, name))
    rectifier = Rectifier(calib, ts)
    partial = dst.with_name(dst.name + '.partial')
    if partial.exists():
        partial.unlink()
    counts, t_wall = {}, time.time()
    with Reader(src) as reader:
        conns = [c for c in reader.connections if c.topic in COPY_TOPICS]
        missing = sorted(set(COPY_TOPICS) - {c.topic for c in conns})
        if missing:
            raise SystemExit(f'ERROR: {src.name} lacks topics: {", ".join(missing)}')
        t_end = reader.start_time + int(duration * 1e9) if duration else None
        total_s = ((t_end or reader.end_time) - reader.start_time) / 1e9
        with Writer(partial) as writer:   # uncompressed on purpose (see make_common_bag.py)
            out = {}
            for c in conns:
                if c.topic not in out:
                    out[c.topic] = writer.add_connection(c.topic, c.msgtype, msgdef=c.msgdef.data, md5sum=c.digest,
                                                         callerid=c.ext.callerid, latching=c.ext.latching)
                if c.topic == LIDAR_IN:
                    v2_digest = c.digest
            caller = '/lvio_make_common_bag'
            out[LIVOX_V1_OUT] = writer.add_connection(LIVOX_V1_OUT, 'livox_ros_driver/msg/CustomMsg', typestore=ts,
                                                      callerid=caller)
            out[CLOUD_OUT] = writer.add_connection(CLOUD_OUT, 'sensor_msgs/msg/PointCloud2', typestore=ts, callerid=caller)
            out[rectifier.topic_out] = writer.add_connection(rectifier.topic_out, 'sensor_msgs/msg/CompressedImage',
                                                             typestore=ts, callerid=caller)
            if out[LIVOX_V1_OUT].digest != v2_digest:
                print(f'NOTE: livox_ros_driver md5 {out[LIVOX_V1_OUT].digest} differs from livox_ros_driver2 {v2_digest}')
            last_report = 0.0
            for conn, stamp, raw in reader.messages(connections=conns, stop=t_end):
                writer.write(out[conn.topic], stamp, raw)
                counts[conn.topic] = counts.get(conn.topic, 0) + 1
                derived = []
                if conn.topic == LIDAR_IN:
                    derived = [(LIVOX_V1_OUT, raw), (CLOUD_OUT, xyzirt_cloud(ts, raw))]
                elif conn.topic == CAMERA_IN:
                    derived = [(rectifier.topic_out, rectifier(raw, conn.msgtype))]
                for topic, data in derived:
                    writer.write(out[topic], stamp, data)
                    counts[topic] = counts.get(topic, 0) + 1
                if time.time() - last_report > 10:
                    last_report = time.time()
                    print(f'  {(stamp - reader.start_time) / 1e9:7.1f} / {total_s:.1f} s of bag time', flush=True)
    os.replace(partial, dst)
    print(f'Wrote {dst} ({dst.stat().st_size / 1e9:.2f} GB) in {time.time() - t_wall:.0f} s')
    for topic in sorted(counts):
        print(f'  {topic:42s} {counts[topic]:7d} msgs')
    if not counts.get(LIDAR_IN) == counts.get(LIVOX_V1_OUT) == counts.get(CLOUD_OUT) or \
            counts.get(CAMERA_IN) != counts.get(rectifier.topic_out):
        raise SystemExit('ERROR: derived topic counts differ from their sources')
