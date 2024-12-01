#!/bin/bash

# Variables
BAG2VID_IMAGE="stathiw/bag2vid:latest"
CONTAINER_NAME="bag2vid"
BAG2VID_SRC="/home/octopus/Hullbot_bag_files"  # Replace with the path to your ROS bags
DISPLAY_VAR=${DISPLAY}

# Run the Docker container
docker run -it --rm \
  --name ${CONTAINER_NAME} \
  --env DISPLAY=${DISPLAY_VAR} \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume ${BAG2VID_SRC}:/home/user/rosbags \
  --network host \
  --privileged \
  ${BAG2VID_IMAGE} \
  bash -c "source /opt/ros/melodic/setup.bash && rosrun bag2vid bag2vid_gui"
