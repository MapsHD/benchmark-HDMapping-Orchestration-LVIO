# For clean, no cache environment:
# docker system prune -a && sudo docker-clean --all
#
# Build docker image
# docker build --no-cache -t coco-lic_noetic .
# (the benchmark builds it with build_images_step2/build_images_step2.sh)
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
#     -v PATH_TO_DIRECTORY_CONTAINING_DATASET:/root/catkin_coco/src/Coco-LIC/data \
#     coco-lic_noetic
#
# Edit config for a desired dataset (example for r3live dataset because why not?):
# nano config/ct_odometry_r3live.yaml
# bag_path: /root/catkin_coco/src/Coco-LIC/data/hku_campus_seq_01.bag // or any other garbage (ros1 (ros2 is also garbage ...)) bag
#
# Run Coco-LIC:
# docker terminal : roslaunch cocolic odometry.launch config_path:=...
#

FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV CATKIN_WS=/root/catkin_coco

SHELL ["/bin/bash", "-c"]


# ============================================================
# Basic Ubuntu packages
# ============================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        nano \
        vim \
        ca-certificates \
        curl \
        gnupg2 \
        lsb-release \
        git \
        wget \
        build-essential \
        cmake \
        pkg-config \
        python3 \
        python3-pip \
        python3-dev \
        dirmngr \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# ROS Noetic repository
# ============================================================

RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | apt-key add -

RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" \
    > /etc/apt/sources.list.d/ros1.list


# ============================================================
# ROS Noetic + ROS development tools
# ============================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-noetic-desktop-full \
        python3-catkin-tools \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        ros-noetic-cv-bridge \
        ros-noetic-tf \
        ros-noetic-message-filters \
        ros-noetic-image-transport \
        ros-noetic-pcl-ros \
        ros-noetic-pcl-conversions \
        ros-noetic-sensor-msgs \
        ros-noetic-geometry-msgs \
        ros-noetic-nav-msgs \
        ros-noetic-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*


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
# Non-ROS dependencies
# ============================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libcgal-dev \
        libyaml-cpp-dev \
        libgoogle-glog-dev \
        libgflags-dev \
        libsuitesparse-dev \
        libatlas-base-dev \
        liblapack-dev \
        libblas-dev \
        libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# Install Ceres Solver 2.0.0
# ============================================================

WORKDIR /root

RUN git clone \
        --branch 2.0.0 \
        --depth 1 \
        https://github.com/ceres-solver/ceres-solver.git \
        ceres-solver

WORKDIR /root/ceres-solver

RUN mkdir build && \
    cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DSUITESPARSE=ON && \
    make -j"$(nproc)" && \
    make install && \
    ldconfig


# ============================================================
# Install PCL 1.13.1
# ============================================================

WORKDIR /root

RUN git clone \
        --branch pcl-1.13.1 \
        --depth 1 \
        https://github.com/PointCloudLibrary/pcl.git \
        pcl

WORKDIR /root/pcl

RUN mkdir build && \
    cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_apps=OFF \
        -DBUILD_examples=OFF \
        -DBUILD_global_tests=OFF \
        -DBUILD_simulation=OFF \
        -DWITH_OPENNI=OFF \
        -DWITH_OPENNI2=OFF \
        -DWITH_QT=OFF \
        -DWITH_VTK=OFF \
        -DWITH_PCAP=OFF && \
    make -j"$(nproc)" && \
    make install && \
    ldconfig


# ============================================================
# Install Livox SDK
# ============================================================

WORKDIR /root

RUN git clone https://github.com/Livox-SDK/Livox-SDK.git

WORKDIR /root/Livox-SDK

RUN cd build && \
    cmake .. && \
    make -j"$(nproc)" && \
    make install && \
    ldconfig


# ============================================================
# Create catkin workspace
# ============================================================

RUN mkdir -p ${CATKIN_WS}/src

WORKDIR ${CATKIN_WS}

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin init && \
    catkin config --extend /opt/ros/${ROS_DISTRO} && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release


# ============================================================
# Clone Livox ROS Driver
# ============================================================

WORKDIR ${CATKIN_WS}/src

RUN git clone https://github.com/Livox-SDK/livox_ros_driver.git


# ============================================================
# Build Livox ROS Driver
# ============================================================

WORKDIR ${CATKIN_WS}

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make


# ============================================================
# Clone Coco-LIC
# ============================================================

WORKDIR ${CATKIN_WS}/src

# Pinned: upstream keeps moving (Gaussian-LIC front-end work); the benchmark
# configs (config/) were written against this commit. Override with
# docker build --build-arg COCO_LIC_REF=<commit> .
ARG COCO_LIC_REF=4ead7e423f67b3e15a86de4ee551bb7c752f2cd8

RUN git clone https://github.com/APRIL-ZJU/Coco-LIC.git && \
    git -C Coco-LIC checkout ${COCO_LIC_REF}


# ============================================================
# Install Coco-LIC ROS dependencies
# ============================================================

WORKDIR ${CATKIN_WS}

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
        -r -y


# ============================================================
# Build Coco-LIC
# ============================================================

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make


# ============================================================
# Create Coco-LIC data directory
# ============================================================

RUN mkdir -p ${CATKIN_WS}/src/Coco-LIC/data


# ============================================================
# Automatically source ROS + workspace
# ============================================================

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_coco/devel/setup.bash" >> /root/.bashrc


# ============================================================
# Start in workspace
# ============================================================

WORKDIR /root/catkin_coco

CMD ["/bin/bash"]