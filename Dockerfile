# Ubuntu 24.04 ROS 2 Jazzy base image
FROM ros:jazzy-ros-core-noble as base

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    qt6-base-dev \
    qt6-multimedia-dev \
    libqt6multimedia6 \
    libqt6svg6-dev \
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

USER ubuntu
WORKDIR /home/ubuntu/ros2_ws
RUN mkdir -p /home/ubuntu/ros2_ws/src/bag2vid

# Copy package manifest first so rosdep layer caches when only source changes
COPY --chown=ubuntu:ubuntu bag2vid/package.xml /home/ubuntu/ros2_ws/src/bag2vid/package.xml

USER root
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

USER ubuntu

# Copy the rest of the source and build
COPY --chown=ubuntu:ubuntu bag2vid /home/ubuntu/ros2_ws/src/bag2vid
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --packages-select bag2vid"

ENV XDG_RUNTIME_DIR=/tmp/runtime-ubuntu

# Source the workspace
RUN echo "source /home/ubuntu/ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc
# Set the entrypoint
ENTRYPOINT [ "bash", "-c", "source /home/ubuntu/ros2_ws/install/setup.bash && ros2 run bag2vid bag2vid_gui" ]
