#!/bin/bash

# Variables
BAG2VID_IMAGE="stathiw/bag2vid:latest"
CONTAINER_NAME="bag2vid"
DISPLAY_VAR=${DISPLAY}

# Run the Docker container
docker run -it --rm \
  --name ${CONTAINER_NAME} \
  --env DISPLAY=${DISPLAY_VAR} \
  --env HOME=${HOME} \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume ${HOME}:${HOME} \
  --network host \
  --privileged \
  ${BAG2VID_IMAGE} \
  bash -c "source /opt/ros/jazzy/setup.bash && ros2 run bag2vid bag2vid_gui"
