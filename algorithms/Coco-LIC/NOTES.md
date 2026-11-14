# Coco-LIC: notes for the COCO (Newer College) dataset

Repo: `APRIL-ZJU/Coco-LIC` (**upstream, not an `mw-` fork**). ROS Noetic, PCL 1.13, Ceres 2.0.
Cross-algorithm plan: `notes/05-run-plan-coco.md` (git-ignored).

Name clash: "COCO" (the dataset folder) and "Coco-LIC" (this algorithm) are unrelated.

## Summary

Coco-LIC is continuous-time (non-uniform B-spline) LiDAR + IMU + one camera. **It reads the bag
file itself** (`bag_path`), not topics. Everything it needs must therefore be *in the bag*: the
Velodyne-layout cloud and the rectified image.

## Inputs (read directly from the common bag)

| Sensor | Topic in common bag | Notes |
| --- | --- | --- |
| LiDAR | `/lvio/points_xyzirt` | parsed as `PointXYZIRTTmp` (`ring` u16, `time` f32 s) in `velodyne_feature_extraction.cpp`; the Ouster layout does not match |
| IMU | `/alphasense_driver_ros/imu` (200 Hz) | – |
| Camera | `/lvio/cam0_rect/compressed` (30 Hz, rectified) | reads `image_topic` or `image_topic + "/compressed"` (`msg_manager.cpp:74-75`) |

Why these choices:
- **IMU**: mandatory. Alphasense, because it is HW-synced with the camera, is the Kalibr reference and
  runs at 200 Hz. A full LiDAR→IMU and camera→IMU pose is supported. Initial attitude comes from the
  accelerometer with yaw removed (`imu_initializer.cpp`), so the z-down IMU is fine.
- **Camera: rectified.** Pinhole + radtan only (`cam_d0..d4`). Upstream's own
  `ct_odometry_lvisam.yaml` runs **LIO-only** (`odometry_mode: 0`, unedited r3live `camera.yaml`) because
  that fisheye wasn't supported either.
- **Rate: 10 Hz effective, no action.** `MsgManager::GetMsgs` attaches **at most one image per spline
  segment** (`t_add: 0.1` s), the last one in the window. Feeding 30 Hz only costs decode time.
- The `cols == 640 || cols == 1280` resize special case (`msg_manager.cpp:689`) does not trigger at 720 px.

## Configuration (implemented: `config/ct_odometry_coco.yaml`, `config/coco/{lidar,imu,camera}.yaml`, `config/coco_lic.launch`)

The IMU noise is Coco-LIC's value for the BMI088 (its R3LIVE-dataset config). COCO's Alphasense
IMU is a BMI085 from the same family. Key values (the snippets below show the structure):

Extrinsics are the **sensor pose in the IMU frame**, Rot row-major: `pose_S_to_G = pose_I_to_G * EP_StoI`
(`spline/trajectory.cpp:167`).

```yaml
# ct_odometry_coco.yaml
odometry_mode: 1          # LICO
lidar_yaml: /coco/lidar.yaml
imu_yaml: /coco/imu.yaml
camera_yaml: /coco/camera.yaml
bag_path: <common bag>
t_add: 0.1
t_begin_add_cam: 5.0
is_evo_viral: false
if_3dgs: false
```

```yaml
# coco/lidar.yaml
lidar0:
  topic: /lvio/points_xyzirt
  is_livox: false
  Extrinsics:              # LiDAR (os_sensor) in Alphasense IMU
    Trans: [-0.057101, -0.032334, -0.033106]
    Rot: [ 0.999945, -0.004891,  0.009238,
          -0.004830, -0.999967, -0.006607,
           0.009270,  0.006562, -0.999936]
num_lidars: 1
lidar_timestamp_end: false # Ouster header = scan start
VLP16: { N_SCAN: 128, Horizon_SCAN: 1024 }   # "VLP16" = any rotating LiDAR here
```

```yaml
# coco/imu.yaml
imu_topic: /alphasense_driver_ros/imu
gravity_mag: 9.81
IMUExtrinsics: { Trans: [0, 0, 0], Rot: [1, 0, 0, 0, 1, 0, 0, 0, 1] }
```

```yaml
# coco/camera.yaml (rectified cam0, see notes/05)
image_topic: /lvio/cam0_rect
image_width: 720
image_height: 540
cam_fx: 272.29
cam_fy: 273.70
cam_cx: 357.52
cam_cy: 255.50
cam_d0: 0.0
cam_d1: 0.0
cam_d2: 0.0
cam_d3: 0.0
cam_d4: 0.0
CameraExtrinsics:          # cam0 in Alphasense IMU (rectification uses R = I, so same as raw cam0)
  Trans: [0.049222, 0.047237, -0.009400]
  Rot: [ 0.003439, -0.001005,  0.999994,
         0.999976,  0.006020, -0.003433,
        -0.006016,  0.999981,  0.001026]
img_time_offset: 0.0018    # Kalibr: t_imu = t_cam + 0.0018
```

## Run model (implemented)

`run.sh` → `common/docker_ros1_session.sh`, **offline mode**. `config/coco_lic.launch` starts
`odometry_node` (`required="true"`) with these node params:
- `project_path = <output dir>`, so sub-configs are read from `<output>/config/coco/*.yaml`;
- `config_path = <output>/config/ct_odometry_coco.yaml`;
- `bag_path` = the common bag.

The node iterates the bag itself (no `rosbag play`, no `/clock`), writes the trajectory and exits.
`run.sh` sets `LVIO_RESULT_GLOB=data/*.txt`, so the run counts as failed if no non-empty TUM file was written.
roslaunch then exits too. RViz is optional (`--rviz`) and not `required` (upstream's is).

## Outputs

| What | Where |
| --- | --- |
| Trajectory | TUM txt `<output>/data/<bag name>_*.txt`: `cache_path = parent(project_path/config) + "/data/" + bag stem` (`odometry_manager.cpp:148,900`); `run.sh` creates `data/` |
| Map | not saved; only RViz topics (`lidar_feature/*`) |

Verified on the 30 s stairs extract: `data/stairs_LICO.txt` with 2853 poses (100 Hz, 17 s run).
Format: TUM (`t x y z qx qy qz qw`), body = Alphasense IMU frame (z down), so the first quaternion is ≈ (1, 0, 0, 0).

## Dockerfile (`algorithms/Coco-LIC/Dockerfile`)

- Done: Coco-LIC pinned with `ARG COCO_LIC_REF=4ead7e4…` (the analysed commit; upstream HEAD moves with the Gaussian-LIC work).
- Left as is: PCL 1.13.1 from source next to ROS's PCL 1.10 (`pcl_ros`). It is required by
  `find_package(PCL 1.13.0)`, but it is an ABI-mixing risk to watch at the first build.
- Runs as root in `/root/catkin_coco`; built as `coco-lic_noetic`.
