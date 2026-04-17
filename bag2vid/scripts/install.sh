# Script to add bag2vid script to bashrc
#

# Variables
BAG2VID_IMAGE="stathiw/bag2vid:latest"

# Add bag2vid script to bashrc
echo "Adding bag2vid docker run alias to bashrc"

# Check if alias already exists
if grep -q "bag2vid()" ~/.bashrc; then
    echo "Alias already exists in ~/.bashrc"
    exit 1
fi

echo "" >> ~/.bashrc
echo "# bag2vid alias" >> ~/.bashrc

echo "BAG2VID_IMAGE=${BAG2VID_IMAGE}" >> ~/.bashrc

echo "bag2vid() {" >> ~/.bashrc
echo "    docker run -it --rm \\" >> ~/.bashrc
echo "    --name bag2vid \\" >> ~/.bashrc
echo "    --env DISPLAY=\${DISPLAY} \\" >> ~/.bashrc
echo "    --env HOME=\${HOME} \\" >> ~/.bashrc
echo "    --volume /tmp/.X11-unix:/tmp/.X11-unix \\" >> ~/.bashrc
echo "    --volume \${HOME}:\${HOME} \\" >> ~/.bashrc
echo "    --network host \\" >> ~/.bashrc
echo "    --privileged \\" >> ~/.bashrc
echo "    \${BAG2VID_IMAGE} \\" >> ~/.bashrc
echo "    bash -c \"source /opt/ros/jazzy/setup.bash && ros2 run bag2vid bag2vid_gui\"" >> ~/.bashrc
echo "}" >> ~/.bashrc
