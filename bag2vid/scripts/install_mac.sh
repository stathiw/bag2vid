# Script to add bag2vid script to bash_profile
#

# Variables
BAG2VID_IMAGE="stathiw/bag2vid:latest"
BAG2VID_SRC="/Users/${USER}/Downloads"  # Replace with the path to your ROS bags

# Add bag2vid script to bash_profile
echo "Adding bag2vid docker run alias to bash_profile"

# Check if alias already exists
if grep -q "bag2vid()" ~/.bash_profile; then
    echo "Alias already exists in ~/.bash_profile"
    exit 1
fi

echo "" >> ~/.bash_profile
echo "# bag2vid alias" >> ~/.bash_profile

echo "BAG2VID_IMAGE=${BAG2VID_IMAGE}" >> ~/.bash_profile
echo "BAG2VID_SRC=${BAG2VID_SRC}" >> ~/.bash_profile

echo "bag2vid() {" >> ~/.bash_profile
echo "    docker run -it --rm \\" >> ~/.bash_profile
echo "    --name bag2vid \\" >> ~/.bash_profile
echo "    --env DISPLAY=host.docker.internal:0 \\" >> ~/.bash_profile
echo "    --volume /tmp/.X11-unix:/tmp/.X11-unix \\" >> ~/.bash_profile
echo "    --volume \${BAG2VID_SRC}:/home/user/rosbags \\" >> ~/.bash_profile
echo "    --network host \\" >> ~/.bash_profile
echo "    --privileged \\" >> ~/.bash_profile
echo "    \${BAG2VID_IMAGE} \\" >> ~/.bash_profile
echo "    bash -c \"source /opt/ros/melodic/setup.bash && rosrun bag2vid bag2vid_gui\"" >> ~/.bash_profile
echo "}" >> ~/.bash_profile

xhost + 127.0.0.1
