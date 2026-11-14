# For clean, no cache environment:
# docker system prune -a && sudo docker-clean --all
#
# Build docker image:
# docker build --no-cache -t fast-livo2_noetic .
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
#     -v PATH_TO_DIRECTORY_CONTAINING_DATASET_BAGS:/root/data \
#     fast-livo2_noetic
#
# Run FAST-LIVO2:
# roslaunch fast_livo mapping_avia.launch
# rosbag play root/data/DATASET.bag
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
        wget \
        unzip \
        pkg-config \
        libatlas-base-dev \
        libgoogle-glog-dev \
        libgflags-dev \
        libboost-all-dev && \
    rm -rf /var/lib/apt/lists/*


# ============================================================
# Python tools
# ============================================================

# empy pinned: empy 4.x (PyPI default) would shadow apt's 3.3.4 and
# breaks catkin's template expansion (no em.RAW_OPT).
RUN pip3 install --no-cache-dir \
        catkin-tools \
        empy==3.3.4


# ============================================================
# ROS Noetic repository
# ============================================================

RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | apt-key add -

RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" \
    > /etc/apt/sources.list.d/ros.list


# ============================================================
# ROS Noetic + development tools
# ============================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-noetic-desktop-full \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool && \
    rm -rf /var/lib/apt/lists/*


# ============================================================
# PCL
#
# PCL >= 1.8
# Ubuntu 20.04 provides PCL 1.10
# ============================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libpcl-dev \
        ros-noetic-pcl-ros \
        ros-noetic-pcl-conversions && \
    rm -rf /var/lib/apt/lists/*


# ============================================================
# Eigen
#
# Eigen >= 3.3.4
# Ubuntu 20.04 provides Eigen 3.3.7
# ============================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libeigen3-dev && \
    rm -rf /var/lib/apt/lists/*


# ============================================================
# OpenCV
#
# OpenCV >= 4.2
# Ubuntu 20.04 provides OpenCV 4.2.0
# ============================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libopencv-dev \
        python3-opencv \
        ros-noetic-cv-bridge \
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
# Sophus
#
# Required version:
# commit a621ff
# ============================================================

WORKDIR /root

RUN git clone https://github.com/strasdat/Sophus.git && \
    cd Sophus && \
    git checkout a621ff


# ============================================================
# Sophus compatibility patch
#
# The a621ff version uses Eigen's real()/imag() accessors
# as assignable references. With the Eigen version supplied
# by Ubuntu 20.04 this does not compile.
#
# Change:
#
#     unit_complex_.real() = 1.;
#     unit_complex_.imag() = 0.;
#
# to:
#
#     unit_complex_.real(1.);
#     unit_complex_.imag(0.);
# ============================================================

WORKDIR /root/Sophus

RUN sed -i \
        's/unit_complex_\.real() = 1\./unit_complex_.real(1.)/' \
        sophus/so2.cpp && \
    sed -i \
        's/unit_complex_\.imag() = 0\./unit_complex_.imag(0.)/' \
        sophus/so2.cpp


# ============================================================
# Build Sophus
# ============================================================

RUN mkdir build && \
    cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release && \
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
# Clone Vikit
#
# Different from the one used in FAST-LIVO1.
# ============================================================

WORKDIR ${CATKIN_WS}/src

RUN git clone https://github.com/xuankuzcr/rpg_vikit.git


# ============================================================
# Clone mw-FAST-LIVO2
# ============================================================

RUN git clone https://github.com/mwlasiuk/mw-FAST-LIVO2.git


# ============================================================
# Install ROS dependencies
# ============================================================

WORKDIR ${CATKIN_WS}

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
        -r -y


# ============================================================
# Build FAST-LIVO2
# ============================================================

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make \
        -DCMAKE_BUILD_TYPE=Release


# ============================================================
# Native trajectory output into the run's output directory
#
# With evo/pose_output_en, FAST-LIVO2 writes its TUM trajectory to
# Log/result/<evo/seq_name>.txt under ROOT_DIR, the source directory
# fixed at compile time. Link that folder to result/ in the output
# directory that common/docker_ros1_session.sh mounts at /lvio/out
# (run.sh creates it through LVIO_OUTPUT_SUBDIRS).
# ============================================================

RUN rm -rf ${CATKIN_WS}/src/mw-FAST-LIVO2/Log/result && \
    ln -s /lvio/out/result ${CATKIN_WS}/src/mw-FAST-LIVO2/Log/result


# ============================================================
# Automatically source ROS + workspace
# ============================================================

RUN echo 'source "/opt/ros/noetic/setup.bash"' >> /root/.bashrc && \
    echo 'source "/root/catkin_ws/devel/setup.bash"' >> /root/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH' >> /root/.bashrc


# ============================================================
# Start in workspace
# ============================================================

WORKDIR /root/catkin_ws

CMD ["/bin/bash"]