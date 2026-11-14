# For clean, no cache environment:
# docker system prune -a && sudo docker-clean --all
#
# Build docker image :
# docker build --no-cache -t coin-lio_noetic .
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
#     coin-lio_noetic
#
# Run coin-lio
# roslaunch coin_lio mapping_newer_college.launch bag_file:=/root/data/DATASET.bag
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
        python3 \
        python3-pip \
        python3-dev \
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
        libgoogle-glog-dev && \
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
# Clone mw-COIN-LIO
# ============================================================

WORKDIR ${CATKIN_WS}/src

RUN git clone https://github.com/mwlasiuk/mw-COIN-LIO.git


# ============================================================
# LVIO benchmark fix: message-generation dependency
#
# Upstream CMakeLists.txt builds the coin_lio library from
# sources that include the generated coin_lio/Pose6D.h, but
# never declares the dependency on message generation. A
# parallel catkin build then fails with
#   common_lib.h: 'coin_lio' does not name a type
# Declaring it (valid at the end of the file) fixes the race.
# The leading newline matters: upstream's file has no trailing newline.
# ============================================================

RUN printf '\n%s\n' 'add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)  # LVIO benchmark' \
        >> ${CATKIN_WS}/src/mw-COIN-LIO/CMakeLists.txt && \
    tail -n 2 ${CATKIN_WS}/src/mw-COIN-LIO/CMakeLists.txt


# ============================================================
# Install COIN-LIO dependencies
# ============================================================

WORKDIR ${CATKIN_WS}

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
        -r -y


# ============================================================
# Build COIN-LIO
# ============================================================

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin build coin_lio --force-cmake


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