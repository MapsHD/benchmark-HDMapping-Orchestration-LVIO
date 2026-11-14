# benchmark-HDMapping-Orchestration-LVIO (LiDAR-Visual-Inertial Odometry)

Runs LiDAR-visual-inertial odometry algorithms on one common dataset, each in its own Docker
image, and collects their trajectories and maps. It is the LVIO counterpart of
[benchmark-HDMapping-Orchestration](https://github.com/MapsHD/benchmark-HDMapping-Orchestration)
(LIO), with the same step structure and an `algorithms.conf` single source of truth.

| Algorithm | Repository built by the image |
| --- | --- |
| COIN-LIO | [mwlasiuk/mw-COIN-LIO](https://github.com/mwlasiuk/mw-COIN-LIO) |
| FAST-LIVO2 | [mwlasiuk/mw-FAST-LIVO2](https://github.com/mwlasiuk/mw-FAST-LIVO2) |
| r3live | [mwlasiuk/mw-r3live](https://github.com/mwlasiuk/mw-r3live) |
| LVI-SAM | [mwlasiuk/mw-LVI-SAM](https://github.com/mwlasiuk/mw-LVI-SAM) |
| Coco-LIC | [APRIL-ZJU/Coco-LIC](https://github.com/APRIL-ZJU/Coco-LIC) (pinned commit) |

> **Status.** All five images build. On the four full-length sequences, 19 of 20 runs succeed, and
> Step 4 exports each of them as a TUM trajectory and a PLY map.
> - Coco-LIC on `quad-hard` stops with an OpenCV exception in its visual front end.
> - r3live falls behind real time on `quad-easy` and `quad-medium`, so its trajectory ends 25.6 s
>   and 4.0 s before the bag (see the coverage warning in `export.json`).
>
> Parameter tuning is still open. Ground-truth evaluation is not part of the pipeline yet.

## Contents

- [Repository layout](#repository-layout)
- [0. Dataset and prerequisites](#0-dataset-and-prerequisites)
- [1. Make scripts executable](#1-make-scripts-executable)
- [2. Run](#2-run)
  - [2.1 Full automation](#21-full-automation)
  - [Step 1 Prepare the common bags](#step-1-prepare-the-common-bags)
  - [Step 2 Build the algorithm images](#step-2-build-the-algorithm-images)
  - [Step 3 Run the algorithms](#step-3-run-the-algorithms)
  - [Step 4 Export trajectories and maps](#step-4-export-trajectories-and-maps)
- [3. Algorithms and their inputs](#3-algorithms-and-their-inputs)
- [4. Results](#4-results)
- [5. Adding an algorithm](#5-adding-an-algorithm)
- [6. Troubleshooting](#6-troubleshooting)

## Repository layout

```text
.
├── algorithms.conf                 # algorithms: distro | dir | id | input
├── sequences.conf                  # sequences: id | original bag
├── algos_lib.sh                    # readers for both .conf files (ONLY_ALGOS / ONLY_SEQUENCES)
├── start_benchmark.sh              # Steps 1-3 in one go
├── prepare_data_step1/             # Step 1: original bag -> common ROS 1 bag
│   ├── prepare_data_step1.sh
│   ├── make_common_bag.py          #   converter (rosbags + OpenCV, no ROS needed)
│   ├── coco_calibration.yaml       #   cam0 calibration and the rectified-camera intrinsics
│   └── Dockerfile                  #   helper image lvio-prepare
├── build_images_step2/             # Step 2: docker build of every algorithm
├── run_benchmark_step3/            # Step 3: every algorithm on every sequence
├── export_step4/                   # Step 4: trajectories -> TUM, maps -> PLY (Python, no ROS)
│   ├── export_step4.sh
│   └── export_results.py
├── common/                         # generic Docker / ROS 1 session used by every run.sh
├── algorithms/<ALGORITHM>/
│   ├── Dockerfile
│   ├── run.sh                      # one run: run.sh <common_bag> <output_dir>
│   ├── config/                     # launch file + algorithm configs for the COCO dataset
│   ├── export.yaml                 # Step 4: where its trajectory and map come from
│   └── NOTES.md                    # why each input/parameter was chosen, with code references
├── datasets/                       # data (git-ignored, see datasets/README.md)
├── results/                        # run outputs (git-ignored)
└── notes/                          # working notes (git-ignored)
```

# 0. Dataset and prerequisites

## Dataset

The benchmark uses the **COCO** set: four sequences of the
[Newer College Dataset, Multi-Cam extension (2021)](https://ori-drs.github.io/newer-college-dataset/multi-cam/).
They were recorded with an Ouster OS0-128 LiDAR and an Alphasense Core (4 cameras + IMU).
Place the original bags in `datasets/COCO/` under the names listed in [sequences.conf](sequences.conf).
The expected layout is in [datasets/README.md](datasets/README.md).

| Sequence | Bag | Duration |
| --- | --- | --- |
| quad-easy | `2021-07-01-10-37-38-quad-easy-004.bag` | 199 s |
| quad-medium | `2021-07-01-11-31-35_0-quad-medium-001.bag` | 191 s |
| quad-hard | `2021-07-01-11-35-14_0-quad-hard-002.bag` | 188 s |
| stairs | `2021-07-01-10-40-50_0-stairs-003.bag` | 119 s |

## Prerequisites

- Linux with Bash and Docker; your user must be able to run `docker` without `sudo`.
  ROS is **not** needed on the host.
- Internet access for the image builds (algorithm sources are cloned from GitHub).
- Disk space: about **65 GB** for the four common bags (uncompressed), plus 6–9 GB per algorithm
  image (≈ 36 GB for all five).
- Optional: an X session for RViz (`--rviz`).
- Optional: for Step 1 without Docker (`--no-docker`), Python 3 with
  `rosbags==0.11.9 numpy opencv-python-headless PyYAML`.

# 1. Make scripts executable

The scripts carry the executable bit. If your copy lost it:

```shell
cd mw-benchmark-HDMapping-Orchestration-LVIO
chmod +x start_benchmark.sh algos_lib.sh common/*.sh \
    prepare_data_step1/prepare_data_step1.sh prepare_data_step1/make_common_bag.py \
    build_images_step2/build_images_step2.sh run_benchmark_step3/run_benchmark_step3.sh \
    export_step4/export_step4.sh export_step4/export_results.py \
    algorithms/*/run.sh algorithms/LVI-SAM/lvio/lvio_imu_virtual_frame.py
```

# 2. Run

All commands are run from the repository root. Algorithm ids are the `id` column of
[algorithms.conf](algorithms.conf): `coin-lio fast-livo2 r3live lvi-sam coco-lic`. Sequence ids are
the `id` column of [sequences.conf](sequences.conf). An unknown id aborts and prints the known ones.

## 2.1 Full automation

```shell
./start_benchmark.sh                       # all algorithms, all sequences
./start_benchmark.sh fast-livo2 r3live     # only these algorithms
ONLY_SEQUENCES="stairs" ./start_benchmark.sh --rviz
```

| Argument | Purpose | Default |
| --- | --- | --- |
| `algorithm-id ...` | restrict Steps 2, 3 and 4 to these algorithms | all |
| `--force` | Step 3: delete and redo existing results | skip existing |
| `--rviz` | Step 3: show RViz for every run | off |
| `--rate RATE` | Step 3: `rosbag play` rate | `1.0` |
| `ONLY_SEQUENCES` | restrict Steps 1, 3 and 4 to these sequences | all |

Step 1 always keeps existing common bags, and Step 4 existing exports. Use the step scripts
directly to rebuild them. Step 4 also runs when some Step 3 runs failed (it skips those), but not
after Ctrl-C.

## Step 1 Prepare the common bags

Every algorithm reads the same per-sequence **common bag**. It contains the original sensor data
plus the two derived streams some algorithms need, so nothing algorithm-specific is precomputed.

```shell
./prepare_data_step1/prepare_data_step1.sh
```

The script builds the helper image `lvio-prepare` (Python, no ROS) and converts every sequence:

```text
datasets/COCO/<bag of sequences.conf>  ->  datasets/COCO/common-ros1/<sequence>.bag
```

The common bags are **ROS 1 bags, written uncompressed**. LZ4 chunks written by the Python
`rosbags` library use an LZ4 frame format that ROS 1's `roslz4` rejects
(`ROSLZ4_DATA_ERROR: malformed data to decompress` as soon as a node subscribes). Uncompressed bags
play in every ROS 1 version and need no decompression during real-time playback.
All current algorithms are ROS 1 (`input` kind `common-ros1` in `algorithms.conf`). A ROS 2
algorithm would get a second format, e.g. `common-ros2/` written by the same converter, and its
own `input` kind.

| Topic in the common bag | Content | Used by |
| --- | --- | --- |
| `/os_cloud_node/points` | original Ouster cloud, organized 128 × 1024 | COIN-LIO, FAST-LIVO2, r3live |
| `/os_cloud_node/imu` | original Ouster IMU, 100 Hz | COIN-LIO, r3live |
| `/alphasense_driver_ros/imu` | original Alphasense IMU, 200 Hz | FAST-LIVO2, LVI-SAM, Coco-LIC |
| `/alphasense_driver_ros/cam0/compressed` | original cam0 fisheye, JPEG, 30 Hz | FAST-LIVO2, LVI-SAM |
| `/lvio/points_xyzirt` | **derived**: the Ouster cloud in Velodyne layout (`x y z intensity` f32, `ring` u16, `time` f32 s from scan start), zero returns removed, sorted by time | LVI-SAM, Coco-LIC |
| `/lvio/cam0_rect/compressed` | **derived**: cam0 undistorted to a pinhole camera (720 × 540, fx 272.29, fy 273.70, cx 357.52, cy 255.50, no distortion), JPEG, 30 Hz | r3live, Coco-LIC |

The raw Ouster packets and cameras 1, 3 and 4 are dropped. Derived messages keep the header and
bag time of their source message. Bags are written to `<name>.partial` first and renamed only when
complete. They take about 93 MB per second of recording: ≈ 11 GB for stairs (119 s), ≈ 65 GB for all
four sequences. Conversion takes well under a minute per sequence when the source is in the page
cache, using ~125 MB of RAM.

| Argument or setting | Purpose | Default |
| --- | --- | --- |
| `--data-dir DIR` | directory with the original bags | `datasets/COCO` |
| `--output-dir DIR` | where the common bags are written | `<data-dir>/common-ros1` |
| `--duration SECONDS` | convert only the first SECONDS (quick tests) | whole bag |
| `--force` | replace existing common bags | skip existing |
| `--no-docker` | run the converter with the local `python3` (`PYTHON=` to choose one) | Docker |
| `ONLY_SEQUENCES` | sequences to convert | all |

## Step 2 Build the algorithm images

```shell
./build_images_step2/build_images_step2.sh
ONLY_ALGOS="r3live lvi-sam" ./build_images_step2/build_images_step2.sh
```

Each `algorithms/<dir>/Dockerfile` is built as `<id>_<distro>`:

| Image | Base | Notes |
| --- | --- | --- |
| `coin-lio_noetic` | Ubuntu 20.04, ROS Noetic | adds the message-generation dependency missing from upstream's `CMakeLists.txt` (parallel builds failed with `'coin_lio' does not name a type`) |
| `fast-livo2_noetic` | Ubuntu 20.04, ROS Noetic | Sophus `a621ff`, `rpg_vikit`, `empy==3.3.4`; links its `Log/result/` to the run's `result/` (native TUM trajectory) |
| `r3live_noetic` | Ubuntu 20.04, ROS Noetic | patches `Lidar_offset_to_IMU` to the Ouster value; adds `xvfb` (r3live always opens a GUI window) |
| `lvi-sam_melodic` | Ubuntu 18.04, ROS Melodic | GTSAM 4.0 PPA, Ceres 1.14, `imu_filter_madgwick`, IMU adapter script |
| `coco-lic_noetic` | Ubuntu 20.04, ROS Noetic | Ceres 2.0, PCL 1.13.1 from source, Coco-LIC pinned to `4ead7e4` |

| Argument or setting | Purpose | Default |
| --- | --- | --- |
| `--no-cache` | build without the layer cache (fresh clones of the algorithm repos) | cache on |
| `ONLY_ALGOS` | algorithms to build | all |

The `mw-` repositories are cloned at their current HEAD. Rebuild with `--no-cache` to pick up new
commits. Failed builds are listed at the end and do not stop the others.

## Step 3 Run the algorithms

```shell
./run_benchmark_step3/run_benchmark_step3.sh
ONLY_ALGOS="fast-livo2" ONLY_SEQUENCES="stairs" ./run_benchmark_step3/run_benchmark_step3.sh --rviz
```

For every (sequence, algorithm) pair, `algorithms/<dir>/run.sh` starts the image with the common
bag mounted read-only and `results/<sequence>/<id>/` mounted as the output:

- **play mode** (COIN-LIO, FAST-LIVO2, r3live, LVI-SAM): `roslaunch` the algorithm, `rosbag record`
  its output topics, `rosbag play --clock` only the topics it needs, wait 10 s, stop the recorder,
  then stop the algorithm with SIGINT, allowing up to 180 s for map saving.
- **offline mode** (Coco-LIC): the algorithm reads the bag file itself and exits when done.

A run's status (`ok` / `failed` / `interrupted`) is checked, not assumed. A run **fails** if:
- `rosbag play` reports an error (`[FATAL]`, `ROSLZ4_…`, bag exceptions);
- the algorithm exits before the end of the bag;
- nothing was recorded on its output topics;
- a result file the algorithm writes itself is missing (`LVIO_RESULT_GLOB`: the trajectory files of
  FAST-LIVO2 and Coco-LIC).

The `[lvio] FAILED: ...` lines say why.

**Ctrl-C** stops the current run gracefully (stops the bag, closes the recorded bag, lets the
algorithm save its map), records it as `interrupted` and ends Step 3. Press Ctrl-C a second time to
abort without waiting.

Each algorithm's frame-rate reduction, compressed→raw conversion and IMU adapter run inside
its own launch file (see [section 3](#3-algorithms-and-their-inputs)). Algorithms that open GUI
windows (r3live) get a virtual X server (Xvfb) unless `--rviz` provides the real display. The run's
config is copied into the result folder, and the files are owned by your user.

| Argument or setting | Purpose | Default |
| --- | --- | --- |
| `--data-dir DIR` | dataset directory | `datasets/COCO` |
| `--common-dir DIR` | common ROS 1 bags from Step 1 | `<data-dir>/common-ros1` |
| `--output-dir DIR` | results | `results` |
| `--force` | delete and redo existing results | skip existing |
| `--rviz` | show RViz (runs `xhost +local:docker`) | off |
| `--rate RATE` | `rosbag play` rate | `1.0` |
| `ONLY_ALGOS`, `ONLY_SEQUENCES` | restrict the runs | all |

Every run appends `finished_at,sequence,algorithm,status,seconds,output` to
`results/run_summary.csv`.

A single run without Step 3 (the output folder must be new or empty):

```shell
algorithms/r3live/run.sh datasets/COCO/common-ros1/stairs.bag results/stairs/r3live
LVIO_RVIZ=1 LVIO_PLAY_RATE=0.5 algorithms/r3live/run.sh datasets/COCO/common-ros1/stairs.bag /tmp/r3live-test
```

For a quick end-to-end test, make a short common bag (here 30 s) in a separate folder and point
Step 3 at it:

```shell
ONLY_SEQUENCES=stairs ./prepare_data_step1/prepare_data_step1.sh --duration 30 --output-dir /tmp/lvio-test/common-ros1
ONLY_SEQUENCES=stairs ./run_benchmark_step3/run_benchmark_step3.sh --common-dir /tmp/lvio-test/common-ros1 --output-dir /tmp/lvio-test/results
ONLY_SEQUENCES=stairs ./export_step4/export_step4.sh --common-dir /tmp/lvio-test/common-ros1 --results-dir /tmp/lvio-test/results
```

## Step 4 Export trajectories and maps

```shell
./export_step4/export_step4.sh
ONLY_ALGOS="coco-lic" ONLY_SEQUENCES="stairs" ./export_step4/export_step4.sh --force
```

Step 4 exports every successful run, meaning its latest status in `results/run_summary.csv` is `ok`:
- the trajectory as a **TUM** text file;
- the map as a **PLY** file.

`export_step4/export_results.py` reads what `algorithms/<dir>/export.yaml` names:
- recorded topics;
- files the algorithm wrote itself;
- for Coco-LIC, the common bag.

It runs in the `lvio-prepare` helper image (Python, no ROS) and never re-runs an algorithm.
On a quad sequence an export takes 2–25 s, and rebuilding the Coco-LIC map about 45 s.
All 19 exports take about 2.5 min with `--jobs 2`.
Each export is written to `export.partial/` and renamed to `export/` when it is complete.

```text
results/<sequence>/<id>/export/
├── <id>_trajectory_tum.txt             timestamp tx ty tz qx qy qz qw (space-separated, no header,
│                                       9-decimal timestamps): body pose in the algorithm's world frame
├── lvi-sam_keyframes_trajectory_tum.txt  LVI-SAM only: its loop-closed keyframe poses
├── <id>_map.ply                        binary little-endian PLY in the same world frame:
│                                       float x y z + float intensity (FAST-LIVO2: uchar red green blue)
├── <id>_preview.png                    top (world XY) and side (world XZ) view of the map + trajectory
└── export.json                         sources, frames, counts, time span, path length, notes, warnings
```

| Algorithm | Trajectory: source, world → body frame, rate | Map |
| --- | --- | --- |
| COIN-LIO | `/Odometry`, `camera_init` → Ouster IMU, 10 Hz | sum of `/cloud_registered` (dense scans) |
| FAST-LIVO2 | its own TUM file `result/trajectory.txt` (LIO update time), `camera_init` → Alphasense IMU, 10 Hz | sum of the coloured `/cloud_registered`, which contains **only the points inside cam0's view** (FAST-LIVO2's colour map) |
| r3live | `/aft_mapped_to_init` with the stamp of the paired `/cloud_registered`, `world` → Ouster IMU, 10 Hz | sum of `/cloud_registered` (dense scans, no colour) |
| LVI-SAM | `/lvi_sam/lidar/mapping/odometry`, `odom` → LiDAR, 5 Hz; keyframes from the last `/lvi_sam/lidar/mapping/path` | its own `lvi_sam_map/cloudGlobal.pcd`: features of the loop-closed keyframes, so sparser than the others |
| Coco-LIC | its own TUM file `data/<sequence>_LICO.txt`, `map` → Alphasense IMU, 100 Hz | **rebuilt** (Coco-LIC saves no map): the common bag's `/lvio/points_xyzirt` scans, each point placed with the trajectory at its own time and the LiDAR–IMU extrinsic of the run's `config/coco/lidar.yaml` |

- **Timestamps.** FAST-LIVO2 and r3live stamp `/aft_mapped_to_init` with the publish time
  (`ros::Time::now()`), which lags the measurement. For r3live the lag is a median of 0.13–4.6 s per
  sequence, and up to 26 s on quad-easy. The export uses measurement times instead:
  - FAST-LIVO2: its own trajectory file (`evo/pose_output_en` in `config/coco.yaml`);
  - r3live: the `/cloud_registered` message published in the same iteration, stamped with the scan
    end time. The pairing is checked with the bag record times: each odometry message must arrive
    after its cloud and before the next one.
- **Frames.** Every algorithm keeps its own world and body frames (listed in `export.json`).
  Trajectories are not aligned to each other or to the ground truth.
- **Map density.** Points are reduced to one per 5 cm voxel (centroid, mean intensity/colour).
  `--voxel-size 0` keeps every point (e.g. ≈100 M points, 1.6 GB for COIN-LIO on stairs).
- **Checks.** The run fails if:
  - fewer than two poses remain, or values are not finite;
  - a quaternion norm is off by more than 1e-3;
  - the re-read PLY does not match its header.

  Unsorted or repeated timestamps are fixed and noted in `export.json`.
- **Warnings** (in `export.json` and the log): the trajectory ends more than 2 s before the common
  bag, starts more than 10 s after it, or leaves the map's bounding box. `trajectory.coverage`
  gives the exact offsets.

| Argument or setting | Purpose | Default |
| --- | --- | --- |
| `--results-dir DIR` | results of Step 3 | `results` |
| `--data-dir DIR` | dataset directory | `datasets/COCO` |
| `--common-dir DIR` | common ROS 1 bags (needed for the Coco-LIC map) | `<data-dir>/common-ros1` |
| `--voxel-size METERS` | map voxel size, `0` keeps every point | `0.05` |
| `--jobs N` | exports at the same time (each needs up to ~4 GB of RAM) | `1` |
| `--force` | redo existing exports | skip existing |
| `ONLY_ALGOS`, `ONLY_SEQUENCES` | restrict the exports | all |

Every export appends `finished_at,sequence,algorithm,status,seconds,output` to
`results/export_summary.csv`, with status `ok`, `failed` or `skipped` (a failed run).
The log of each export is in `results/<sequence>/<id>/logs/export.log`.

# 3. Algorithms and their inputs

Each algorithm gets what it uses: an IMU only where it uses one, the camera form it supports, and
the image rate it was designed for. Details and code references: `algorithms/<ALGORITHM>/NOTES.md`.

| Algorithm | LiDAR | IMU | Camera | Image rate |
| --- | --- | --- | --- | --- |
| COIN-LIO | Ouster, organized | Ouster | none (uses LiDAR intensity images) | – |
| FAST-LIVO2 | Ouster | Alphasense | cam0 fisheye (`EquidistantCamera`), raw | 10 Hz (keep 1 of 3) |
| r3live | Ouster | Ouster (r3live has no LiDAR–IMU rotation) | cam0 rectified | 15 Hz (keep 1 of 2) |
| LVI-SAM | XYZIRT | Alphasense, rotated into LVI-SAM's expected axes, + madgwick orientation | cam0 fisheye (`KANNALA_BRANDT`), raw | 30 Hz (internal 20 Hz) |
| Coco-LIC | XYZIRT | Alphasense | cam0 rectified | 10 Hz (uses 1 image per 0.1 s itself) |

The extrinsic chain (cam0 ↔ Alphasense IMU ↔ Ouster IMU ↔ LiDAR) combines three sources:
- the Kalibr camera calibration in `datasets/COCO/cam_calibration`;
- the Kalibr IMU–IMU transform from [ori-drs/halo_description](https://github.com/ori-drs/halo_description);
- the Ouster factory metadata.

The IMU–IMU rotation was checked against the bag data (within 1°).

# 4. Results

```text
results/
├── run_summary.csv                               Step 3: one line per run
├── export_summary.csv                            Step 4: one line per export
└── <sequence>/
    ├── coin-lio/      recorded-coin-lio.bag      /Odometry, /cloud_registered
    ├── fast-livo2/    recorded-fast-livo2.bag    /aft_mapped_to_init, /cloud_registered
    │                  result/trajectory.txt      FAST-LIVO2's own TUM trajectory (LIO update times)
    ├── r3live/        recorded-r3live.bag        /aft_mapped_to_init, /cloud_registered
    │                  r3live_output/             r3live logs; its RGB map is written here only when
    │                                             's' is pressed in its control panel (needs --rviz)
    ├── lvi-sam/       recorded-lvi-sam.bag       /lvi_sam/lidar/mapping/{odometry,path,cloud_registered}
    │                  lvi_sam_map/               PCD map saved on shutdown
    └── coco-lic/      data/<sequence>_*.txt      TUM trajectory (no map is saved)
```

Every folder also contains:
- `config/`: the exact config used;
- `logs/`: `launch.log`, `play.log`, `record.log`, `recorded_topics.txt` (message count per output
  topic), the ROS logs and `export.log` (Step 4);
- `export/`: the TUM trajectory, PLY map, preview and `export.json` written by
  [Step 4](#step-4-export-trajectories-and-maps).

Conversion of these outputs to HDMapping sessions, and evaluation against ground truth, are not
implemented yet.

# 5. Adding an algorithm

1. Add a row to [algorithms.conf](algorithms.conf): `distro | dir | id | input`.
2. Create `algorithms/<dir>/` with:
   - a `Dockerfile` that builds the algorithm (runs as root; the session fixes file ownership);
   - `config/<name>.launch` that accepts the args `rviz`, `input_bag` and `out_dir`, and loads its
     configs from `$(arg out_dir)/config/`;
   - `run.sh`, which exports `LVIO_SETUP`, `LVIO_MODE`, `LVIO_LAUNCH`, `LVIO_PLAY_TOPICS` and
     `LVIO_RECORD_TOPICS` and calls `common/docker_ros1_session.sh` (copy an existing one). Optional:
     - `LVIO_RESULT_GLOB`: a file the algorithm writes itself that must exist afterwards (both modes);
     - `LVIO_NEEDS_DISPLAY=1`: the algorithm opens GUI windows; the image then needs `xvfb`;
     - `LVIO_SHUTDOWN_TIMEOUT`: seconds for a graceful stop, default 180;
     - `LVIO_CONTAINER_HOME`, `LVIO_OUTPUT_SUBDIRS`.

     The list with descriptions is at the top of `common/docker_ros1_session.sh`;
   - `export.yaml` for Step 4: where its trajectory and map come from and its world/body frames.
     The sources (`odometry`, `tum_file`, `path_last`; `accumulate`, `pcd`, `reconstruct`) are
     described at the top of `export_step4/export_results.py`. Copy the closest existing one;
   - `NOTES.md` with the reasons for its inputs and parameters.
3. Build it with `ONLY_ALGOS=<id> ./build_images_step2/build_images_step2.sh`, then test it on a
   short common bag (see the quick end-to-end test in Step 3), including Step 4.

# 6. Troubleshooting

| Symptom | Cause / fix |
| --- | --- |
| `ROSLZ4_DATA_ERROR: malformed data to decompress` | Common bags made by an older version of Step 1 (LZ4, in `datasets/COCO/common/`). Re-run Step 1, which now writes `common-ros1/`, and delete the old `common/` folder. |
| A bag "plays fine" in a manual test but the algorithm gets nothing | `rosbag play` does not read message data when nobody subscribes. Test with a subscriber (e.g. `rosbag record` or `rostopic echo`) running. |
| Step 3 skips a run (`... exists (use --force to redo)`) | The result folder is not empty, e.g. from an earlier failed run. Use `--force` or delete `results/<sequence>/<id>/`. |
| An image build failed | `docker buildx history ls`, then `docker buildx history logs <build id>` show the full log. |
| `REQUIRED process [...] has died! ... exit code -11` after `cannot open display` in `logs/launch.log` | The algorithm opens a GUI window. Set `LVIO_NEEDS_DISPLAY=1` in its `run.sh` and add `xvfb` to its image (as for r3live). |
| A run `failed` | Read the `[lvio] FAILED:` lines, then `logs/launch.log` / `logs/play.log` in its result folder. |
| Step 4 skips a run (`... export exists` or `its latest run is 'failed'`) | Use `--force` to redo an export. A failed run has nothing to export; redo the run in Step 3. |
| An export `failed` | Read the `[export] FAILED:` line in `results/<sequence>/<id>/logs/export.log`. |
| `export.json` warns `the trajectory ends N s before the end of the input bag` | The algorithm fell behind real time, and the recorder stopped 10 s after the bag ended (`LVIO_DRAIN_SECONDS` in `common/ros1_session.sh`). It stopped before the algorithm caught up. Seen for r3live on quad-easy and quad-medium. |
