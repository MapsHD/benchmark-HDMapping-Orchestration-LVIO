# For clean, no cache environment:
# docker system prune -a && sudo docker-clean --all
#
# Build docker image:
# docker build --no-cache -t mw-lvi-sam .
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
#     mw-lvi-sam
#
# Run LVI-SAM:
# roslaunch lvi_sam run.launch
# rosbag play root/data/DATASET.bag
#

FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=melodic
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
        python \
        python-pip \
        python-dev \
        dirmngr \
        software-properties-common \
        wget \
        unzip \
        libatlas-base-dev \
        libgoogle-glog-dev && \
    rm -rf /var/lib/apt/lists/*


# ============================================================
# GTSAM repository
# ============================================================

RUN add-apt-repository ppa:borglab/gtsam-release-4.0


# ============================================================
# GTSAM
# ============================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libgtsam-dev \
        libgtsam-unstable-dev && \
    rm -rf /var/lib/apt/lists/*


# ============================================================
# ROS Melodic repository
# ============================================================

RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | apt-key add -

RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" \
    > /etc/apt/sources.list.d/ros1.list


# ============================================================
# ROS Melodic + development tools
# ============================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-melodic-desktop-full \
        python-catkin-tools \
        python-rosdep \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool && \
    rm -rf /var/lib/apt/lists/*


# ============================================================
# ROS environment
# ============================================================

RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc


# ============================================================
# rosdep
# ============================================================

RUN rosdep init || true
RUN rosdep update


# ============================================================
# Ceres Solver 1.14.0
# ============================================================

WORKDIR /root/Downloads

RUN wget -O ceres.zip \
        https://github.com/ceres-solver/ceres-solver/archive/1.14.0.zip && \
    unzip ceres.zip && \
    mv ceres-solver-1.14.0 ceres-solver


WORKDIR /root/Downloads/ceres-solver

RUN mkdir ceres-bin && \
    cd ceres-bin && \
    cmake .. && \
    make install -j"$(nproc)"


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
# Clone mw-LVI-SAM
# ============================================================

WORKDIR ${CATKIN_WS}/src

RUN git clone https://github.com/mwlasiuk/mw-LVI-SAM.git


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
# Build LVI-SAM
# ============================================================

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make


# ============================================================
# Automatically source ROS + workspace
# ============================================================

RUN echo 'source "/opt/ros/melodic/setup.bash"' >> /root/.bashrc && \
    echo 'source "/root/catkin_ws/devel/setup.bash"' >> /root/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH' >> /root/.bashrc


# ============================================================
# Start in workspace
# ============================================================

WORKDIR /root/catkin_ws

CMD ["/bin/bash"]