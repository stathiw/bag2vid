# bag2vid
A GUI tool to extract videos from a rosbag.



## Install
1. Clone the repository
```
git clone git@github.com:stathiw/bag2vid.git
```
2. Add bag2vid command to bashrc
```
cd bag2vid/scripts
./install.sh
```
3. Modify the environment variables in the bashrc to correctly reference your rosbag source folder and bag2vid image tag.

## Build

### Docker

1. Set rosbag src folder
```
export BAG2VID_SRC=/path/to/rosbag_src
```
2. Build or pull the docker image
```
cd bag2vid
docker compose build
```
or
```
docker pull stathiw/bag2vid:latest
```
## Run
After completing the install and either building or pulling the docker image, the application can be run using either
```
bag2vid
```
or
```
cd bag2vid
docker compose run ros_melodic rosrun bag2vid bag2vid_gui
```

#### How to use
Use the **_Load Bag_** button to select a rosbag to extract videos from.

Select the camera topic to extract from the dropdown.  Upon loading a rosbag, all camera topics will be found.

Press the **_Extract Video_** button and enter a file name and location to save the extracted video.

Drag the start (green) and end (red) time selection markers to select the times between which to extract a video.

Press the **_Capture Screenhshot_** button to save the current frame as a png.

![Screenshot from 2025-01-31 17-38-05](https://github.com/user-attachments/assets/da77686d-7cc9-47e7-90bd-b11bb74f710c)
