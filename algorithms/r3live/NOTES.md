# r3live: notes for the COCO (Newer College) dataset

Repo: `mwlasiuk/mw-r3live` (fork, no own commits yet) + Livox SDK / `livox_ros_driver` (build dependency). ROS Noetic.
Cross-algorithm plan: `notes/05-run-plan-coco.md` (git-ignored).

## Summary

r3live is LiDAR + IMU + one camera (RGB map). Ouster LiDAR is supported natively. The camera must be
**rectified pinhole**, and the IMU **must be the Ouster IMU**. It needs **one source patch** (a
compile-time LiDAR→IMU offset). Its own `r3live_bag_ouster.launch` never ran a camera
(`Image_topic=/NotAvail`), so expect tuning.

## Inputs

| Sensor | Topic in common bag | Adaptation in `config/r3live.launch` |
| --- | --- | --- |
| LiDAR | `/os_cloud_node/points` | none; this name is **hard-coded** in `LiDAR_front_end.cpp:145` (type 3) |
| IMU | `/os_cloud_node/imu` (100 Hz) | none |
| Camera | `/lvio/cam0_rect/compressed` (30 Hz, rectified) | `topic_tools drop … 1 2` → `/lvio/cam0_rect_15hz/compressed` |

Why these choices:
- **IMU: Ouster is forced.** Three independent reasons:
  1. LiDAR→IMU is a **translation-only compile-time constant**: `common_lib.h:61`
     `Lidar_offset_to_IMU(0.04165, 0.02326, -0.0284)` (Livox Avia). There is no rotation, and the
     Alphasense IMU is rotated ~180° about x relative to the LiDAR.
  2. Init fixes `gravity = (0, 0, 9.805)` and `rot_end = I` (`IMU_Processing.cpp:96-97`). This needs a
     **z-up** IMU and a roughly level start. The Ouster IMU is z-up (acc_z ≈ +9.4); the Alphasense IMU is z-down.
  3. Its own Ouster launch uses `/os_cloud_node/imu`.
- **Camera: rectified.** The camera model is pinhole + radtan only (`cv::initUndistortRectifyMap`,
  `camera_dist_coeffs` k1 k2 p1 p2 k3). It reads `Image_topic` or `Image_topic + "/compressed"`
  (`r3live.hpp:333-350`), converting to `bgr8`. Mono input gives a grey "RGB" map.
- **15 Hz**: every received frame is processed. The reference dataset (`ziv-lin/r3live_dataset`)
  records the camera at 15 Hz (LiDAR 10 Hz, IMU 200 Hz).

## Source patch (implemented: Dockerfile `sed` + `grep` check, the build fails if it does not apply)

```
r3live/src/loam/include/common_lib.h:61
static const Eigen::Vector3d Lidar_offset_to_IMU(-0.006253, 0.011775, -0.007645); // Ouster os_sensor in os_imu
```

This is the `os_sensor` origin in the `os_imu` frame, from the Ouster metadata `imu_to_sensor_transform`.
Semantics: `p_imu = p_lidar + Lidar_offset_to_IMU` (`r3live_lio.cpp:203`).

## Configuration (implemented: `config/r3live.launch` + `config/r3live_coco.yaml`)

Based on `r3live_bag_ouster.launch` + `config/r3live_config.yaml`. `map_output_dir` is set to
`<output>/r3live_output` in the launch file. Key values:

```yaml
/IMU_topic: /os_cloud_node/imu
/Image_topic: /lvio/cam0_rect_15hz          # r3live appends /compressed itself
/LiDAR_pointcloud_topic: /laser_cloud_flat  # front-end output, as shipped
Lidar_front_end: { lidar_type: 3, point_step: 1 }
r3live_lio: { lio_update_point_step: 6 }    # as in the Ouster launch
r3live_vio:
  image_width: 720
  image_height: 540
  camera_intrinsic: [272.29, 0.0, 357.52,  0.0, 273.70, 255.50,  0.0, 0.0, 1.0]   # rectified cam0
  camera_dist_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]
  # pose of cam0 in the IMU frame, row-major (r3live_vio.cpp:475-476: t_wc = R_wi*t + t_wi, R_wc = R_wi*R)
  camera_ext_R: [-0.001446,  0.008235,  0.999965,
                 -0.999999,  0.000548, -0.001451,
                 -0.000559, -0.999966,  0.008234]
  camera_ext_t: [0.099899, -0.068157, -0.030894]
r3live_common: { estimate_i2c_extrinsic: 1, estimate_intrinsic: 1 }   # shipped defaults, keep
```

The rectified intrinsics come from `cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(balance=0)`,
720×540, with 100 % valid pixels (see `notes/05`).

Time: image stamps vs the Ouster IMU clock differ by ≈ +2.9 ms (Kalibr 1.8 ms + IMU–IMU ≈ 1.1 ms).
r3live estimates the camera–IMU time offset online, so no setting is needed.

## Outputs

| What | Where |
| --- | --- |
| Trajectory | `/aft_mapped_to_init` (`nav_msgs/Odometry`) |
| Map | `/cloud_registered` (recorded). The RGB map is written to `map_output_dir` **only when 's' is pressed in r3live's OpenCV "Control panel" window** (`cv_keyboard_callback`, `r3live_vio.cpp:1076`), not on shutdown. To get it, run with the real display (the default; not with `--no-rviz`) and press 's' before the bag ends. It is meshable with `r3live_reconstruct_mesh.launch`. |

## Export (Step 4, `export.yaml`)

- Trajectory: `/aft_mapped_to_init` is stamped with `ros::Time::now()` (`r3live_lio.cpp:1022`), the
  publish time.
  - Each pose takes the stamp of the `/cloud_registered` message published in the same LIO
    iteration. That stamp is `lidar_end_time` (`r3live_lio.cpp:962/1006`).
  - The messages are paired by index, and the pairing is checked with the bag record times.
  - Stairs: paired messages were recorded at most 111 ms apart (median 20 ms). The publish lag
    reached 1.24 s when r3live fell behind for ~17 scans near the end.
- Map: the sum of `/cloud_registered`: x y z + intensity, no colour (see above for the RGB map).

**r3live falls behind real time on the quad sequences (full runs, 2026-11-14).**

| Sequence | publish lag, median / max | trajectory ends before the bag | loop start→end |
| --- | --- | --- | --- |
| stairs | 0.13 / 1.2 s | 0.1 s | 0.44 m |
| quad-easy | 4.6 / 25.7 s | **25.6 s** (1731 of ~1990 scans) | 28 m (cut short) |
| quad-medium | 0.6 / 6.0 s | **4.0 s** | 2.0 m |
| quad-hard | 0.2 / 5.1 s | 0.1 s | 3.8 m (double walls in the map) |

- No scan is dropped: the cloud stamps step by 0.1 s.
- The cause is the recorder stopping 10 s after the bag ends, while r3live is still working through
  its backlog. Once there was also a ~9 s stall with no output at all.
- The other algorithms close the quad loops within 0.4 m.
- Possible fixes, not applied:
  - a longer drain for r3live (`LVIO_DRAIN_SECONDS`, which `docker_ros1_session.sh` would need to pass on);
  - a slower `rosbag play` rate.

## Display and shutdown (found in the first runs)

- `service_VIO_update` always calls `cv::imshow("Control panel", ...)` (`r3live_vio.cpp:1095`). Without a
  display GTK fails (`cannot open display`) and `r3live_mapping` segfaults (exit -11). `run.sh` sets
  `LVIO_NEEDS_DISPLAY=1`, so headless runs (`--no-rviz`) get Xvfb inside the container. By default the
  real display is used, which also shows the control panel.
- r3live does not exit on SIGINT, and it saves nothing on shutdown (see the map row above). `run.sh` therefore sets
  `LVIO_SHUTDOWN_TIMEOUT=20` instead of the default 180 s.
- Verified on the 30 s stairs extract: 296 `/aft_mapped_to_init` + 296 `/cloud_registered` messages, 74 s run.

## Dockerfile (`algorithms/r3live/Dockerfile`)

- Done: the `Lidar_offset_to_IMU` patch before `catkin build`. The image is now Ouster-specific.
- Done: `xvfb` for headless runs.
- Runs as root in `/root/catkin_ws`; built as `r3live_noetic`. `mw-r3live` (analysed at `6143a38`),
  Livox SDK and `livox_ros_driver` follow HEAD.

## BUNKER (Bunker DVI reg-1): `config/bunker/`

Sensors are the same for every algorithm: the Livox Mid-360 (`/lvio/livox/lidar`, the
`livox_ros_driver/CustomMsg` copy, or `/lvio/points_xyzirt`), its built-in IMU (`/livox/imu`, 200 Hz,
acceleration in g) and the D455F colour camera undistorted by Step 1 (`/lvio/d455_rect`, 640 × 480,
fx 381.93 fy 381.16 cx 325.59 cy 239.48, no distortion, 15 Hz).

The extrinsics come from `steps/1_prepare_data/bunker_calibration.yaml`:
- **Livox IMU in the LiDAR:** (11.0, 23.29, −44.12) mm, axes aligned (Mid-360 manual);
- **colour camera in the LiDAR:** estimated by registering D455F depth to the FARO map (notes/scripts/bunker_d455_extrinsic.py);
- **camera time offset:** −0.03 s (camera stamp − 0.03 s = LiDAR time).

Not run yet (2026-11-21).

- **Image patch** (`lvio/patch_r3live.py`, applied in the Dockerfile before the build):
  1. `Lidar_offset_to_IMU` is read from `LVIO_LIDAR_OFFSET_TO_IMU`. `config/bunker/r3live.launch`
     sets `-0.011 -0.02329 0.04412` on `r3live_mapping`. Without it, the Ouster value used for COCO
     applies, so COCO behaves as before.
  2. The Livox (HORIZON) front end kept only `x > 0.7 m` (in front of a forward-looking Avia). For
     the 360° Mid-360 this is now `range > blind` (`Lidar_front_end/blind: 0.5`). Its tag filter
     (noise bits 0–3) is unchanged.
- `r3live.yaml`:
  - `lidar_type: 1`, `N_SCANS: 4`;
  - camera 640 × 480 with the rectified intrinsics;
  - `camera_ext_R/t` = T_imu_camera;
  - `lio_update_point_step: 4` (Livox default);
  - online extrinsic, intrinsic and time-offset estimation stays on.
- `r3live.launch`: `/lvio/livox/lidar` is remapped onto the front end's hard-coded `/livox/lidar`.
  `IMU_topic` is `/livox/imu`, `Image_topic` is `/lvio/d455_rect` (15 Hz, all frames).
