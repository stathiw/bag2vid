#!/bin/bash

# Variables
BAG2VID_IMAGE="stathiw/bag2vid:latest"
CONTAINER_NAME="bag2vid"
BAG2VID_SRC="/home/${USER}/rosbags"  # Replace with the path to your ROS bags
DISPLAY_VAR=${DISPLAY}

# Run the Docker container
docker run -it --rm \
  --name ${CONTAINER_NAME} \
  --env DISPLAY=${DISPLAY_VAR} \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume ${BAG2VID_SRC}:/home/ubuntu/rosbags \
  --network host \
  --privileged \
  ${BAG2VID_IMAGE} \
  bash -c "source /opt/ros/jazzy/setup.bash && ros2 run bag2vid bag2vid_gui"
