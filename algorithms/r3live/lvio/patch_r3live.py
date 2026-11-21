#!/usr/bin/env python3
"""LVIO benchmark patches to r3live, applied in the Dockerfile before the build (see ../NOTES.md).

1. Lidar_offset_to_IMU (src/loam/include/common_lib.h) is a compile-time constant (shipped: Livox Avia).
   It becomes the value of the environment variable LVIO_LIDAR_OFFSET_TO_IMU ("x y z", the LiDAR origin
   in the IMU frame, m), read once at start-up, so one image serves every dataset. Default: the Ouster
   os_sensor origin in os_imu (COCO), so a launch file that does not set it behaves as before.
2. The Livox (HORIZON) front end keeps only points with x > 0.7 m, i.e. in front of a forward-looking Avia.
   A Mid-360 sees 360 deg, so the condition becomes a range check against the front end's own 'blind'.
Usage: patch_r3live.py <r3live package dir>
"""
import sys
from pathlib import Path

root = Path(sys.argv[1])

header = root / 'src/loam/include/common_lib.h'
text = header.read_text()
old = next(line for line in text.splitlines() if line.startswith('static const Eigen::Vector3d Lidar_offset_to_IMU('))
new = '''#include <cstdio>
#include <cstdlib>
// LVIO benchmark: LiDAR origin in the IMU frame from LVIO_LIDAR_OFFSET_TO_IMU ("x y z"), default Ouster (COCO).
inline Eigen::Vector3d lvio_lidar_offset_to_imu()
{
    double x = -0.006253, y = 0.011775, z = -0.007645;
    const char *value = std::getenv("LVIO_LIDAR_OFFSET_TO_IMU");
    if (value && std::sscanf(value, "%lf %lf %lf", &x, &y, &z) != 3)
    {
        std::fprintf(stderr, "LVIO_LIDAR_OFFSET_TO_IMU must hold 3 numbers, got '%s'\\n", value);
        std::abort();
    }
    return Eigen::Vector3d(x, y, z);
}
static const Eigen::Vector3d Lidar_offset_to_IMU = lvio_lidar_offset_to_imu();'''
header.write_text(text.replace(old, new, 1))

front = root / 'src/loam/LiDAR_front_end.cpp'
text = front.read_text()
old = '&& msg->points[ i ].x > 0.7 )'
assert text.count(old) == 1, f'{front}: expected one "{old}"'
new = ('&& ( msg->points[ i ].x * msg->points[ i ].x + msg->points[ i ].y * msg->points[ i ].y'
       ' + msg->points[ i ].z * msg->points[ i ].z ) > blind * blind ) // LVIO benchmark: was x > 0.7 (Avia)')
front.write_text(text.replace(old, new, 1))
print('patched', header, front)
