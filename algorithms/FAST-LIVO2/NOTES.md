# FAST-LIVO2: notes for the COCO (Newer College) dataset

Repo: `mwlasiuk/mw-FAST-LIVO2` (fork, no own commits yet) + `xuankuzcr/rpg_vikit`. ROS Noetic.
Cross-algorithm plan: `notes/05-run-plan-coco.md` (git-ignored).

## Summary

FAST-LIVO2 is LiDAR + IMU + one camera. Everything it needs is native on COCO: Ouster LiDAR,
fisheye camera via vikit `EquidistantCamera`, and the Alphasense IMU. Only the image format
(compressed → raw) and the image rate (30 → 10 Hz) are adapted in the run script.
`config/HILTI22.yaml` uses the same Alphasense camera and IMU and is the closest template.

## Inputs

| Sensor | Topic in common bag | Adaptation in `config/fast_livo2.launch` |
| --- | --- | --- |
| LiDAR | `/os_cloud_node/points` | none (`lidar_type: 3`) |
| IMU | `/alphasense_driver_ros/imu` (200 Hz) | none |
| Camera | `/alphasense_driver_ros/cam0/compressed` (30 Hz fisheye) | `topic_tools drop … 2 3` → `/lvio/cam0_10hz/compressed`, then `image_transport republish` → `/lvio/cam0_10hz/image_raw` |

Why these choices:
- **IMU**: used (`imu_en: true` in every shipped config). Alphasense, because it is HW-synced with the
  camera and is the Kalibr reference. FAST-LIVO2 supports a full LiDAR→IMU rotation.
- **Camera**: original fisheye. vikit has `EquidistantCamera`. A raw `sensor_msgs/Image` is required
  (`img_cbk` takes `ImageConstPtr`; it converts to `bgr8`, and mono8 input is fine).
- **10 Hz**: in LIVO mode each image triggers one LIO+VIO update, with the scan cut at image time
  (`sync_packages`). All shipped setups run 10 Hz images; HILTI22 decimates 40→10 with `hilti_en`
  (hard-coded `% 4`, so not usable for 30→10). Hence `topic_tools drop`.

## Configuration (implemented: `config/coco.yaml`, `config/camera_coco.yaml`, `config/fast_livo2.launch`)

`coco.yaml` is `HILTI22.yaml` (same Alphasense camera + IMU) with the LiDAR/LIO settings of
`NTU_VIRAL.yaml` (Ouster). Key values:

```yaml
common:      { lid_topic: /os_cloud_node/points, imu_topic: /alphasense_driver_ros/imu,
               img_topic: /lvio/cam0_10hz/image_raw, img_en: 1, lidar_en: 1 }
preprocess:  { lidar_type: 3, scan_line: 128, hilti_en: false }   # blind ≈ 0.65 like COIN-LIO
imu:         { imu_en: true }
time_offset: { imu_time_offset: 0.0, img_time_offset: 0.0018, lidar_time_offset: 0.0 }
extrin_calib:
  # T_imu_lidar: p_imu = R p_lidar + T   (LIVMapper.cpp:645)
  extrinsic_T: [-0.057101, -0.032334, -0.033106]
  extrinsic_R: [ 0.999945, -0.004891,  0.009238,
                -0.004830, -0.999967, -0.006607,
                 0.009270,  0.006562, -0.999936]
  # T_cam_lidar: p_cam = Rcl p_lidar + Pcl  (vio.cpp:57-58, Rci = Rcl * Rli)
  Rcl: [-0.001446, -0.999999, -0.000559,
         0.008235,  0.000548, -0.999966,
         0.999965, -0.001451,  0.008234]
  Pcl: [-0.079792, -0.024078, -0.106073]
```

```yaml
# camera_coco.yaml (Kalibr cam0, 2021-07-01)
cam_model: EquidistantCamera
cam_width: 720
cam_height: 540
scale: 1.0
cam_fx: 352.779
cam_fy: 354.609
cam_cx: 359.035
cam_cy: 260.546
k1: -0.04217
k2: -0.00413
k3: 0.00179
k4: -0.00063
```

- `img_time_offset: +0.0018`: Kalibr `t_imu = t_cam + 0.0018`. FAST-LIVO2 adds it to the image stamp.
- `lidar_time_offset: 0`: the Ouster header is the scan start (NTU-VIRAL needed −0.1 because its stamps are at scan end).
- Gravity/acc scale come from the mean acceleration (`IMU_Processing.cpp:144`), so the z-down Alphasense IMU is fine.

## Outputs

| What | Where |
| --- | --- |
| Trajectory | `result/trajectory.txt`: FAST-LIVO2's own TUM file (`evo/pose_output_en`, `LIVMapper.cpp:384-404`), one pose per LIO update, stamped with `last_lio_update_time`. `/aft_mapped_to_init` (`nav_msgs/Odometry`) is still recorded, but it is stamped with `ros::Time::now()` (`LIVMapper.cpp:1340`), the publish time |
| Map | `/cloud_registered`, world frame `camera_init`. In LIVO mode the LIO step publishes an **empty** cloud and the VIO step the dense scan's points **inside cam0's view**, coloured from the image (`publish_frame_world`, `LIVMapper.cpp:1133-1189`). So there are 2 messages per scan, and the map covers only what cam0 saw |

The FAST-LIO converter from the LIO benchmark needs the odometry topic renamed and `PointXYZRGB` support.

Verified on the 30 s stairs extract: 296 `/aft_mapped_to_init` + 591 `/cloud_registered` messages (54 s run).

**Native trajectory file.** FAST-LIVO2 writes `Log/result/<evo/seq_name>.txt` under `ROOT_DIR`, the
source directory fixed at compile time (`CMakeLists.txt:40`).
- The Dockerfile replaces that folder with a link to `/lvio/out/result`.
- `run.sh` creates `result/` (`LVIO_OUTPUT_SUBDIRS`) and requires `result/trajectory.txt` (`LVIO_RESULT_GLOB`).
- `config/coco.yaml` sets `seq_name: "trajectory"`.
- The file uses TUM order (`t x y z qx qy qz qw`), body = Alphasense IMU, 6 decimals.

**`pcd_save_en` stays off.**
- Type 0 saves the same coloured FOV points as `/cloud_registered`.
- Type 1 (all points, intensity) keeps the whole map in RAM until shutdown, and the authors warn that
  saving hurts real time (`LIVMapper.cpp:1190-1192`).

## Export (Step 4, `export.yaml`)

- Trajectory: `result/trajectory.txt`, re-written in the common TUM format.
- Map: the sum of the coloured `/cloud_registered` clouds (empty ones skipped), as `uchar red green blue` PLY.

## Dockerfile (`algorithms/FAST-LIVO2/Dockerfile`)

- Done: `empy==3.3.4` pinned. PyPI `empy` 4.x would shadow apt's 3.3.4 and break catkin's template expansion.
- Left as is: `catkin init/config` followed by `catkin_make` (the catkin-tools config is ignored, but harmless).
- Runs as root in `/root/catkin_ws`; built as `fast-livo2_noetic`. `mw-FAST-LIVO2` (analysed at `0d2c034`)
  and `rpg_vikit` (`6c886c8`) follow HEAD.
- Sophus `a621ff` + `so2.cpp` patch: kept (required by FAST-LIVO2).

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

- `fast_livo2.yaml`:
  - LiDAR: `lidar_type: 1` (Livox CustomMsg), `scan_line: 4`, `blind: 0.5`, `point_filter_num: 1`.
    The Avia handler's `tag & 0x30 == 0x10` filter applies only with feature extraction, which is
    off (`preprocess.cpp:120` vs `:168`), so Mid-360 points (tag mostly 0x00) are kept.
  - IMU: `gyr_cov: 0.3` as `avia.yaml` (Livox IMU); the g units are handled by FAST-LIVO2's
    mean-acceleration scaling.
  - Extrinsics: `extrinsic_T` = LiDAR in the IMU (−0.011, −0.02329, 0.04412), `extrinsic_R` = I;
    `Rcl/Pcl` = T_cam_lidar.
  - `img_time_offset: −0.03`.
  - The rest (vio, lio, evo native trajectory) is as `coco.yaml`.
- `camera.yaml`: `Pinhole`, rectified intrinsics, zero distortion.
- `fast_livo2.launch`: `topic_tools drop 1 3` (15 → 10 Hz), then `republish` compressed → raw.
