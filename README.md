# bag2vid
A GUI tool to extract videos from a rosbag.



## Install
1. Instal from source

```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone http://github.com/stathiw/bag2vid
cd bag2vid
rosdep install -y -r --from-path . --ignore-src
cd ../../
catkin build
```

## How to use
```
rosrun bag2vid bag2vid_gui
```
Use the **_Load Bag_** button to select a rosbag to extract videos from.

Select the camera topic to extract from the dropdown.  Upon loading a rosbag, all camera topics will be found.

Press the **_Extract Video_** button and enter a file name and location to save the extracted video.

Drag the start (green) and end (red) time selection markers to select the times between which to extract a video.



![bag2vid_gui](https://github.com/stathiw/bag2vid/assets/23045886/d850ef8c-31a2-49af-a3e7-5a62924afcab)
