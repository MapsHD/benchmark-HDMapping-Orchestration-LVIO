# LVI-SAM: notes for the COCO (Newer College) dataset

Repo: `mwlasiuk/mw-LVI-SAM` (fork, no own commits yet). **ROS Melodic / Ubuntu 18.04.**
Cross-algorithm plan: `notes/05-run-plan-coco.md` (git-ignored).

## Summary

LVI-SAM is LiDAR + 9-axis IMU + one camera. The fisheye camera is supported natively (camodocal
`KANNALA_BRANDT`). It needs:
- the Velodyne-layout cloud from the common bag;
- an IMU orientation (`imu_filter_madgwick`);
- a fix for its **hard-coded LiDAR↔IMU axis assumptions**. These are solved without touching the
  source by feeding it a "virtual IMU" (below).

## Inputs

| Sensor | Topic in common bag | Adaptation in `config/lvi_sam.launch` |
| --- | --- | --- |
| LiDAR | `/lvio/points_xyzirt` (`ring` u16, `time` f32 s) | none |
| IMU | `/alphasense_driver_ros/imu` (200 Hz) | `lvio_imu_virtual_frame.py` (rotate into V) → `/lvio/imu_v/data_raw` → `imu_filter_madgwick` → `/lvio/imu_v/data` |
| Camera | `/alphasense_driver_ros/cam0/compressed` (30 Hz fisheye) | `image_transport republish` (as in `module_sam.launch`) → `/lvio/cam0/image_raw` |

Why these choices:
- **LiDAR**: `imageProjection.cpp` registers `PointXYZIRT` with `uint16 ring` + `float time`. The native
  Ouster layout (`uint8 ring`, `uint32 t`) does not match.
- **IMU**: 9-axis required (orientation used for initialisation), and neither COCO IMU has it.
  The madgwick setup copies the LIO benchmark LIO-SAM overlay (`use_mag=false`, `publish_tf=false`,
  `world_frame=enu`). The Alphasense IMU is used because it is HW-synced with the cameras and is the Kalibr reference.
- **Camera**: original fisheye as `KANNALA_BRANDT` (identical to Kalibr pinhole-equidistant).
  `fisheye: 0`: COCO images have no circular border; the shipped mask is for LVI-SAM's own MEI lens.
- **30 Hz**: LVI-SAM's own dataset records the camera at 30 Hz (measured in `datasets/LVI`). The
  feature tracker throttles internally to `freq: 20`.

## Hard-coded frame assumptions and the virtual IMU

LVI-SAM assumes its own rig, where `R_lidar_imu = Ry(180°)`:
- `visual_estimator/initial/initial_alignment.h:85-86`: `q_lidar_to_cam = (0,1,0,0)`,
  `q_lidar_to_cam_eigen = (w0,x0,y0,z1)`, plus a global `Rz(180°)` at line 133;
- `visual_estimator/utility/visualization.cpp:85`: `q_cam_to_lidar(0,1,0,0)`.

COCO's Alphasense IMU is `R_lidar_imu ≈ Rx(180°)` (= Rz(180°)·Ry(180°)), so LiDAR-aided VINS
initialisation and depth registration would get wrong rotations.

**Fix without a source change:** re-express the Alphasense IMU in a virtual frame V (same origin)
with `R_lidar_V = Ry(180°)` exactly, i.e. rotate every IMU sample:

```
omega_V = R_V_alpha * omega_alpha,   acc_V = R_V_alpha * acc_alpha
R_V_alpha = [[-0.999945,  0.004830, -0.009270],
             [-0.004891, -0.999967,  0.006562],
             [-0.009238,  0.006607,  0.999936]]     # ≈ Rz(180°), 0.71° off
```

Check: with V, `R_V_cam0` comes out ≈ `[0,0,-1; -1,0,0; 0,1,0]`, identical to LVI-SAM's own `params_camera.yaml`.

Alternative: `Cc19245/LVI-SAM-Easyused` (linked from the LVI-SAM README) parameterises these
rotations ("does NOT modify the algorithm part"). It is config-only but a third-party fork.

The adapter is `lvio/lvio_imu_virtual_frame.py` (Python 2/3), copied into the `lvi_sam` package
by the Dockerfile so `roslaunch` can start it. The rotation is passed as a param in `lvi_sam.launch`.

## Configuration (implemented: `config/lvi_sam.launch`, `config/params_lidar.yaml`, `config/params_camera.yaml`)

`config/params_lidar.yaml`. LIO-SAM conventions (`utility.h:220-232`, `imuPreintegration.cpp:74-75`):

```yaml
pointCloudTopic: /lvio/points_xyzirt
imuTopic: /lvio/imu_v/data
N_SCAN: 128
Horizon_SCAN: 1024
timeField: time
downsampleRate: 1            # 2 if it cannot keep up
useImuHeadingInitialization: false
extrinsicRot: [-1, 0, 0,  0, 1, 0,  0, 0, -1]   # acc_lidar = extRot * acc_V
extrinsicRPY: [-1, 0, 0,  0, 1, 0,  0, 0, -1]   # q_lidar = q_V(madgwick) * extQRPY -> R_V_lidar
extrinsicTrans: [0.057248, -0.032395, -0.032790] # IMU (V) origin in LiDAR frame; rotation ignored there by LVI-SAM
savePCD: true                # map on shutdown (deletes/recreates savePCDDirectory)
```

`config/params_camera.yaml`:

```yaml
imu_topic: /lvio/imu_v/data
image_topic: /lvio/cam0/image_raw
model_type: KANNALA_BRANDT
image_width: 720
image_height: 540
projection_parameters: { k2: -0.04217, k3: -0.00413, k4: 0.00179, k5: -0.00063,
                         mu: 352.779, mv: 354.609, u0: 359.035, v0: 260.546 }
fisheye: 0
extrinsicRotation: [ 0.001446, -0.008235, -0.999965,      # R_V_cam0 (imu^R_cam)
                    -0.999999,  0.000548, -0.001451,
                     0.000559,  0.999966, -0.008234]
extrinsicTranslation: [-0.048904, -0.047538, -0.009542]   # cam0 origin in V
# depth registration: points LiDAR -> camera-centred FLU frame (x=optical z, y=-optical x, z=-optical y)
# feature_tracker_node.cpp:276 (same derivation as LVI-SAM-Easyused T_cFLU_lidar)
lidar_to_cam_tx: -0.106073
lidar_to_cam_ty: 0.079792
lidar_to_cam_tz: 0.024078
lidar_to_cam_rx: -0.000548
lidar_to_cam_ry: 0.008235
lidar_to_cam_rz: 0.001447
td: 0.0018                   # Kalibr: t_imu = t_cam + 0.0018
freq: 20                     # shipped default
```

Note: the depth registration keeps only LiDAR points with x ≥ 0 (in front of the LiDAR). COCO's cam0
looks along LiDAR +x, so this holds.

## Outputs

| What | Where |
| --- | --- |
| Trajectory | `lvi_sam/lidar/mapping/odometry`, `lvi_sam/lidar/mapping/path` |
| Map | `lvi_sam/lidar/mapping/map_global`, `.../cloud_registered`; PCD on shutdown with `savePCD: true` |

`savePCD: true` saves to `$HOME/lvi_sam_map/` on shutdown, and `run.sh` sets `HOME` to the output dir.
The session gives roslaunch up to 180 s (`--sigint-timeout`, which Melodic's roslaunch supports) so the save completes.
Verified on the 30 s stairs extract: 149 odometry/path/cloud messages recorded, and `lvi_sam_map/` holds
`cloudGlobal.pcd`, `cloudSurf.pcd`, `cloudCorner.pcd`, `trajectory.pcd` and `transformations.pcd`.

## Export (Step 4, `export.yaml`)

- Trajectory: `/lvi_sam/lidar/mapping/odometry`, the online scan-to-map poses at about 5 Hz,
  stamped with the scan time. Body = LiDAR.
- Extra trajectory: `lvi-sam_keyframes_trajectory_tum.txt` comes from the last `/path` message. It
  holds the loop-closed keyframe poses, the same as `transformations.pcd`.
- Map: `lvi_sam_map/cloudGlobal.pcd` (ASCII): the corner and surface features of all keyframes,
  placed with the loop-closed poses (`mapOptmization.cpp:374-412`).
  - It is not downsampled: the `downSizeFilterSurf.setInputCloud` there is never followed by `filter()`.
  - It is sparser than the dense maps of the other algorithms.
- Not used for the map:
  - `cloud_registered` holds only ~600 features per scan.
  - `cloud_registered_raw` (the full deskewed scan, `mapOptmization.cpp:1647-1653`) is published only
    when something subscribes, and is not recorded.

## Dockerfile (`algorithms/LVI-SAM/Dockerfile`)

- Done: `ros-melodic-imu-filter-madgwick` and the IMU adapter script.
- Runs as root in `/root/catkin_ws`; built as `lvi-sam_melodic` (`algorithms.conf` distro column).
  `mw-LVI-SAM` (analysed at `0d822f6`) follows HEAD.
- 18.04/Melodic and the borglab GTSAM PPA are EOL but still build (verified). This is the most fragile image.
