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
| Trajectory | `/aft_mapped_to_init` (`nav_msgs/Odometry`) |
| Map | `/cloud_registered` (RGB-coloured in LIVO mode, world frame); `pcd_save_en: true` → `Log/pcd/all_raw_points.pcd` |

The FAST-LIO converter from the LIO benchmark needs the odometry topic renamed and `PointXYZRGB` support.

Verified on the 30 s stairs extract: 296 `/aft_mapped_to_init` + 591 `/cloud_registered` messages (54 s run).

`pose_output_en` and `pcd_save_en` are off: they would write into the package's `Log/` inside the
container. The trajectory and map come from the recorded topics instead.

## Dockerfile (`algorithms/FAST-LIVO2/Dockerfile`)

- Done: `empy==3.3.4` pinned. PyPI `empy` 4.x would shadow apt's 3.3.4 and break catkin's template expansion.
- Left as is: `catkin init/config` followed by `catkin_make` (the catkin-tools config is ignored, but harmless).
- Runs as root in `/root/catkin_ws`; built as `fast-livo2_noetic`. `mw-FAST-LIVO2` (analysed at `0d2c034`)
  and `rpg_vikit` (`6c886c8`) follow HEAD.
- Sophus `a621ff` + `so2.cpp` patch: kept (required by FAST-LIVO2).
