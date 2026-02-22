FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# ── 1. Common build tools + GPU rendering support ────────────────────────────
# libglvnd: vendor-neutral GL dispatch library. Allows the container to select
#   between Intel Mesa and NVIDIA OpenGL at runtime, whichever is injected by
#   nvidia-container-toolkit (--gpus all) or the DRI device.
# libgl1 / libegl1 / libgles2: standard OpenGL/EGL/GLES interfaces used by
#   Gazebo Ignition (OGRE2 renderer), RViz2, and OpenCV imshow.
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-dev \
    python3-numpy \
    python3-yaml \
    swig \
    libusb-1.0-0-dev \
    libbluetooth-dev \
    libcwiid-dev \
    libspnav-dev \
    udev \
    wget \
    curl \
    lsb-release \
    gnupg2 \
    x11-apps \
    mesa-utils \
    libglvnd-dev \
    libgl1 \
    libegl1 \
    libgles2 \
    libglx-mesa0 \
    vulkan-tools \
    && rm -rf /var/lib/apt/lists/*

# ── 2. Gazebo Fortress repository ────────────────────────────────────────────
RUN curl https://packages.osrfoundation.org/gazebo.gpg \
        --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) \
          signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
          http://packages.osrfoundation.org/gazebo/ubuntu-stable \
          $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list

# ── 3. Intel RealSense SDK 2 repository ──────────────────────────────────────
# Required for ros-humble-realsense2-camera (real robot only; safe to install
# in the container so the image works on both sim and real deployments).
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
        | gpg --dearmor > /usr/share/keyrings/librealsense.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/librealsense.gpg] \
          https://librealsense.intel.com/Debian/apt-repo \
          $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/librealsense.list

# ── 4. Core ROS 2 / Gazebo / Nav2 / MoveIt2 dependencies ────────────────────
RUN apt-get update && apt-get install -y \
    # Navigation2
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-nav2-amcl \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-waypoint-follower \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-map-server \
    ros-humble-nav2-behaviors \
    # MoveIt2 (arm planning)
    ros-humble-moveit \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-ros-planning-interface \
    # ROS 2 Control
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ign-ros2-control \
    # Gazebo Fortress + bridges
    ignition-fortress \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-interfaces \
    ros-humble-ros-gz-image \
    # Robot description / URDF
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    # Misc ROS 2 utilities
    ros-humble-teleop-twist-keyboard \
    ros-humble-joy-linux \
    ros-humble-diagnostic-updater \
    ros-humble-laser-proc \
    ros-humble-realtime-tools \
    && rm -rf /var/lib/apt/lists/*

# ── 5. Vision & AprilTag dependencies ────────────────────────────────────────
# These are needed by limo_manipulation for object detection and pose estimation.
RUN apt-get update && apt-get install -y \
    # OpenCV + cv_bridge
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-geometry \
    python3-opencv \
    libopencv-dev \
    # AprilTag detector (2D detection from camera images)
    ros-humble-apriltag \
    ros-humble-apriltag-msgs \
    ros-humble-apriltag-detector \
    ros-humble-apriltag-detector-mit \
    # AprilTag Python bindings (used by generate_apriltags.py script)
    python3-apriltag \
    # Pillow — used to generate AprilTag PNG textures for Gazebo models
    python3-pil \
    && rm -rf /var/lib/apt/lists/*

# ── 6. Intel RealSense SDK + ROS 2 wrapper ───────────────────────────────────
# librealsense2: SDK libraries (hardware not needed in container; safe to install)
# ros-humble-realsense2-*: ROS 2 camera node and URDF description
RUN apt-get update && apt-get install -y \
    librealsense2 \
    librealsense2-dev \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

# ── 7. YDLidar-SDK (robot base lidar) ────────────────────────────────────────
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git && \
    mkdir -p YDLidar-SDK/build && \
    cd YDLidar-SDK/build && \
    cmake .. && \
    make && \
    make install && \
    cd .. && pip install . && \
    cd .. && rm -rf YDLidar-SDK

# ── 8. Non-root user ─────────────────────────────────────────────────────────
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# ── 9. Shell auto-source ──────────────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME
