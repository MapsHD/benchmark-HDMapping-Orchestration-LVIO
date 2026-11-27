# For clean, no cache environment:
# docker system prune -a && sudo docker-clean --all
#
# Build docker image
# docker build --no-cache -t r3live_noetic .
# (the benchmark builds it with steps/2_build_images/build_images.sh)
#
# Give docker access to display:
# xhost +local:docker
#
# Run docker image:
# docker run -it --rm \
#     --net=host \
#     --privileged \
#     -e DISPLAY=$DISPLAY \
#     -e QT_X11_NO_MITSHM=1 \
#     -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
#     -v PATH_TO_DIRECTORY_CONTAINING_DATASET_BAGS:/root/data \
#     r3live_noetic
#
# Run r3live:
# docker terminal 1 : roslaunch r3live r3live_bag.launch
# docker terminal 2 : rosbag play root/data/DATASET.bag
#

FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV CATKIN_WS=/root/catkin_ws

SHELL ["/bin/bash", "-c"]


# ============================================================
# Basic Ubuntu packages
# ============================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        gnupg2 \
        lsb-release \
        git \
        build-essential \
        cmake \
        python3 \
        python3-pip \
        python3-dev \
        libcgal-dev \
        pcl-tools \
        libopencv-dev \
        libopencv-contrib-dev \
        dirmngr && \
    rm -rf /var/lib/apt/lists/*


# ============================================================
# ROS Noetic repository
# ============================================================

RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | apt-key add -

RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" \
    > /etc/apt/sources.list.d/ros1.list


# ============================================================
# ROS Noetic + development tools
# ============================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-noetic-desktop-full \
        python3-catkin-tools \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        libgoogle-glog-dev \
        ros-noetic-cv-bridge \
        ros-noetic-tf \
        ros-noetic-message-filters \
        ros-noetic-image-transport && \
    rm -rf /var/lib/apt/lists/*


# ============================================================
# ROS environment
# ============================================================

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc


# ============================================================
# rosdep
# ============================================================

RUN rosdep init || true
RUN rosdep update


# ============================================================
# Install Livox SDK
# ============================================================

WORKDIR /root

RUN git clone https://github.com/Livox-SDK/Livox-SDK.git

WORKDIR /root/Livox-SDK/build

RUN cmake .. && \
    make -j"$(nproc)" && \
    make install


# ============================================================
# Create catkin workspace
# ============================================================

RUN mkdir -p ${CATKIN_WS}/src

WORKDIR ${CATKIN_WS}

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin init && \
    catkin config --extend /opt/ros/${ROS_DISTRO} && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    catkin config --merge-devel


# ============================================================
# Clone Livox ROS Driver
# ============================================================

WORKDIR ${CATKIN_WS}/src

RUN git clone https://github.com/Livox-SDK/livox_ros_driver.git


# ============================================================
# Clone mw-R3LIVE
# ============================================================

RUN git clone https://github.com/mwlasiuk/mw-r3live.git


# ============================================================
# LVIO benchmark patches (lvio/patch_r3live.py, see NOTES.md)
#
# 1. Lidar_offset_to_IMU, a compile-time constant upstream, is
#    read from LVIO_LIDAR_OFFSET_TO_IMU (set by each dataset's
#    launch file; default: the Ouster value used for COCO).
# 2. The Livox front end's "x > 0.7 m" filter (forward-looking
#    Avia) becomes a range check, for the 360 deg Mid-360.
# ============================================================

COPY lvio/patch_r3live.py /tmp/patch_r3live.py
RUN python3 /tmp/patch_r3live.py ${CATKIN_WS}/src/mw-r3live/r3live && \
    grep -q 'LVIO_LIDAR_OFFSET_TO_IMU' ${CATKIN_WS}/src/mw-r3live/r3live/src/loam/include/common_lib.h && \
    grep -q 'blind \* blind' ${CATKIN_WS}/src/mw-r3live/r3live/src/loam/LiDAR_front_end.cpp


# ============================================================
# Install ROS package dependencies
# ============================================================

WORKDIR ${CATKIN_WS}

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
        -r -y


# ============================================================
# Build Livox ROS Driver + R3LIVE
# ============================================================

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin build --force-cmake


# ============================================================
# LVIO benchmark: virtual X server
#
# r3live always opens an OpenCV "Control panel" window
# (cv::imshow in service_VIO_update) and segfaults without a
# display. ros1_session.sh starts Xvfb for headless runs
# (run.sh sets LVIO_NEEDS_DISPLAY=1). See NOTES.md.
# ============================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends xvfb && \
    rm -rf /var/lib/apt/lists/*


# ============================================================
# Automatically source ROS + workspace
# ============================================================

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc


# ============================================================
# Start in workspace
# ============================================================

WORKDIR /root/catkin_ws

CMD ["/bin/bash"]