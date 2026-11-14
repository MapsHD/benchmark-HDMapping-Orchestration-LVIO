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

> **Status.** All five images build. On a 30 s extract of `stairs`, all five algorithms ran and
> produced a trajectory and a map: 4 maps from recorded registered clouds, LVI-SAM also a PCD map,
> Coco-LIC a trajectory only. Step 1 was checked on the real data, including ROS 1 playback with
> subscribers. Full-length runs and parameter tuning are still open. Ground-truth evaluation is
> not part of the pipeline yet.

## Contents

- [Repository layout](#repository-layout)
- [0. Dataset and prerequisites](#0-dataset-and-prerequisites)
- [1. Make scripts executable](#1-make-scripts-executable)
- [2. Run](#2-run)
  - [2.1 Full automation](#21-full-automation)
  - [Step 1 Prepare the common bags](#step-1-prepare-the-common-bags)
  - [Step 2 Build the algorithm images](#step-2-build-the-algorithm-images)
  - [Step 3 Run the algorithms](#step-3-run-the-algorithms)
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
├── common/                         # generic Docker / ROS 1 session used by every run.sh
├── algorithms/<ALGORITHM>/
│   ├── Dockerfile
│   ├── run.sh                      # one run: run.sh <common_bag> <output_dir>
│   ├── config/                     # launch file + algorithm configs for the COCO dataset
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
| `algorithm-id ...` | restrict Steps 2 and 3 to these algorithms | all |
| `--force` | Step 3: delete and redo existing results | skip existing |
| `--rviz` | Step 3: show RViz for every run | off |
| `--rate RATE` | Step 3: `rosbag play` rate | `1.0` |
| `ONLY_SEQUENCES` | restrict Steps 1 and 3 to these sequences | all |

Step 1 always keeps existing common bags. Use the Step 1 script directly to rebuild them.

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
| `fast-livo2_noetic` | Ubuntu 20.04, ROS Noetic | Sophus `a621ff`, `rpg_vikit`, `empy==3.3.4` |
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
- (offline mode) the expected result file is missing.

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
```

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
├── run_summary.csv
└── <sequence>/
    ├── coin-lio/      recorded-coin-lio.bag      /Odometry, /cloud_registered
    ├── fast-livo2/    recorded-fast-livo2.bag    /aft_mapped_to_init, /cloud_registered
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
  topic) and the ROS logs.

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
     - `LVIO_RESULT_GLOB`: offline mode, the file that must exist afterwards;
     - `LVIO_NEEDS_DISPLAY=1`: the algorithm opens GUI windows; the image then needs `xvfb`;
     - `LVIO_SHUTDOWN_TIMEOUT`: seconds for a graceful stop, default 180;
     - `LVIO_CONTAINER_HOME`, `LVIO_OUTPUT_SUBDIRS`.

     The list with descriptions is at the top of `common/docker_ros1_session.sh`;
   - `NOTES.md` with the reasons for its inputs and parameters.
3. Build it with `ONLY_ALGOS=<id> ./build_images_step2/build_images_step2.sh`, then test it on a
   short common bag (see the quick end-to-end test in Step 3).

# 6. Troubleshooting

| Symptom | Cause / fix |
| --- | --- |
| `ROSLZ4_DATA_ERROR: malformed data to decompress` | Common bags made by an older version of Step 1 (LZ4, in `datasets/COCO/common/`). Re-run Step 1, which now writes `common-ros1/`, and delete the old `common/` folder. |
| A bag "plays fine" in a manual test but the algorithm gets nothing | `rosbag play` does not read message data when nobody subscribes. Test with a subscriber (e.g. `rosbag record` or `rostopic echo`) running. |
| Step 3 skips a run (`... exists (use --force to redo)`) | The result folder is not empty, e.g. from an earlier failed run. Use `--force` or delete `results/<sequence>/<id>/`. |
| An image build failed | `docker buildx history ls`, then `docker buildx history logs <build id>` show the full log. |
| `REQUIRED process [...] has died! ... exit code -11` after `cannot open display` in `logs/launch.log` | The algorithm opens a GUI window. Set `LVIO_NEEDS_DISPLAY=1` in its `run.sh` and add `xvfb` to its image (as for r3live). |
| A run `failed` | Read the `[lvio] FAILED:` lines, then `logs/launch.log` / `logs/play.log` in its result folder. |
