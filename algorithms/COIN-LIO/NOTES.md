# COIN-LIO: notes for the COCO (Newer College) dataset

Repo: `mwlasiuk/mw-COIN-LIO` (fork, no own commits yet). ROS Noetic.
Cross-algorithm plan: `notes/05-run-plan-coco.md` (git-ignored).

## Summary

COIN-LIO is **LiDAR-inertial**. It builds "images" from Ouster intensity returns and uses no camera.
It ships a ready Newer College config, so it runs on COCO without any data conversion.

## Inputs (from the common bag)

| Sensor | Topic | Notes |
| --- | --- | --- |
| LiDAR | `/os_cloud_node/points` | native Ouster, **must stay organized** (128 × 1024) for the intensity projection |
| IMU | `/os_cloud_node/imu` (100 Hz) | Ouster internal IMU, as in the shipped config |
| Camera | – | not used |

Ouster-only: the projection needs `beam_altitude_angles` etc. from `config/os_newer_college.json`
and the column shift `image/u_shift: 31` (NC-specific, set in the launch file).

## Configuration

Use `launch/mapping_newer_college.launch` as shipped. It loads `params.yaml`, `line_removal.yaml` and
`os_newer_college.json`, and sets `lid_topic`, `imu_topic` and `u_shift`.

- `extrinsic_T: [-0.00625, 0.011775, 0.028535]`, `extrinsic_R: I`: **kept as shipped** (decision 2026-11-13).
  Note: Ouster metadata implies z = −0.007645 for `os_sensor` (shipped z matches the `os_lidar` origin height).
  Revisit only if results look biased.
- Gravity/IMU scale are estimated from the mean acceleration at start (`imu_processing.cpp:108,184`),
  so the IMU orientation and units don't matter.

## Run model (implemented)

`run.sh` → `common/docker_ros1_session.sh`, play mode. `config/coin_lio.launch` sets `use_sim_time`
and includes the shipped `mapping_newer_college.launch` unchanged (`bag_file` empty, so it does
not play the bag itself). The session plays `/os_cloud_node/points /os_cloud_node/imu` and records
the outputs below.

## Outputs

| What | Where |
| --- | --- |
| Trajectory | `/Odometry` (`nav_msgs/Odometry`) |
| Map | `/cloud_registered` (world frame) and optionally `pcd_save_en: true` |

These are the same topics as FAST-LIO, so the LIO benchmark converter
`benchmark-FAST-LIO-to-HDMapping/src/fast-lio-to-hdmapping` (reads `/Odometry` + `/cloud_registered`)
should apply unchanged.

## Dockerfile (`algorithms/COIN-LIO/Dockerfile`)

- **Build fix:** upstream `CMakeLists.txt` builds the `coin_lio` library from sources that include the
  generated `coin_lio/Pose6D.h`, but declares no dependency on message generation. A parallel
  `catkin build` therefore failed with `common_lib.h:37: 'coin_lio' does not name a type`. The
  Dockerfile appends `add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)`, with a
  leading newline because upstream's file has no trailing newline. Verified: the image builds.
  Since `mw-COIN-LIO` is your fork, the same line could go into the fork instead.
- Built as `coin-lio_noetic` by Step 2. Verified on the 30 s stairs extract: 297 `/Odometry` + 297 `/cloud_registered`.
- Runs as root in `/root/catkin_ws`. The session `chown`s the outputs to the host user.
- `git clone mw-COIN-LIO` follows HEAD (analysed at `76729cc`). Rebuild with `--no-cache` to update.
