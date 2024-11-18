# Ubuntu 18.04 ros melodic base image
FROM ros:melodic-ros-core-bionic as base

ARG ROSBAG_SRC_FOLDER

RUN echo "ROSBAG_SRC: $ROSBAG_SRC_FOLDER"

# Check if the ROSBAG_SRC argument is set
RUN if [ -z "$ROSBAG_SRC_FOLDER" ]; then echo "ROSBAG_SRC argument is not set.\nPlease export ROSBAG_SRC=<path to rosbag src folder>."; exit 1; fi

# Install dependencies
RUN apt-get update && apt-get install -y \
    python-catkin-tools \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    qtmultimedia5-dev \
    ros-melodic-cv-bridge \
    && rm -rf /var/lib/apt/lists/

# Initialise rosdep
RUN rosdep init \
    && rosdep update

# Create user user with password user
RUN useradd -ms /bin/bash user

# Create XDG_RUNTIME_DIR with correct permissions
RUN mkdir -p /tmp/runtime-user && chown user:user /tmp/runtime-user && chmod 700 /tmp/runtime-user

# Set the user
USER user

# Create a workspace
RUN mkdir -p /home/user/catkin_ws/src

# Set the workspace
WORKDIR /home/user/catkin_ws

# Copy the source code
COPY . /home/user/catkin_ws/src

USER root

# Install dependencies
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

# Set the user
USER user

# Build the workspace
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin build"

ENV XDG_RUNTIME_DIR=/tmp/runtime-user

# Source the workspace
RUN echo "source /home/user/catkin_ws/devel/setup.bash" >> /home/user/.bashrc
# Set the entrypoint
# CMD ["/bin/bash"]
ENTRYPOINT [ "bash", "-c", "source /home/user/catkin_ws/devel/setup.bash && rosrun bag2vid bag2vid_gui" ]
