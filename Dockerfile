# Ubuntu 24.04 ROS 2 Jazzy base image
FROM ros:jazzy-ros-core-noble as base

ARG ROSBAG_SRC_FOLDER

RUN echo "ROSBAG_SRC: $ROSBAG_SRC_FOLDER"

# Check if the ROSBAG_SRC argument is set
RUN if [ -z "$ROSBAG_SRC_FOLDER" ]; then echo "ROSBAG_SRC argument is not set.\nPlease export ROSBAG_SRC=<path to rosbag src folder>."; exit 1; fi

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    qt6-base-dev \
    qt6-multimedia-dev \
    libqt6multimedia6 \
    libopencv-dev \
    libx264-dev \
    ffmpeg \
    ros-jazzy-cv-bridge \
    ros-jazzy-rosbag2-cpp \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-rosbag2-storage-default-plugins \
    ros-jazzy-sensor-msgs \
    && rm -rf /var/lib/apt/lists/

# Initialise rosdep
RUN rosdep init \
    && rosdep update

# Create XDG_RUNTIME_DIR with correct permissions for ubuntu user (UID 1000 in base image)
RUN mkdir -p /tmp/runtime-ubuntu && chown ubuntu:ubuntu /tmp/runtime-ubuntu && chmod 700 /tmp/runtime-ubuntu

# Set the user
USER ubuntu

# Create a workspace
RUN mkdir -p /home/ubuntu/ros2_ws/src

# Set the workspace
WORKDIR /home/ubuntu/ros2_ws

# Copy the source code
COPY . /home/ubuntu/ros2_ws/src/bag2vid

USER root

# Install dependencies
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

# Set the user
USER ubuntu

# Build the workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --packages-select bag2vid"

ENV XDG_RUNTIME_DIR=/tmp/runtime-ubuntu

# Source the workspace
RUN echo "source /home/ubuntu/ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc
# Set the entrypoint
ENTRYPOINT [ "bash", "-c", "source /home/ubuntu/ros2_ws/install/setup.bash && ros2 run bag2vid bag2vid_gui" ]
