<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="bag2vid/resources/logo/bag2vid-wordmark-dark.svg">
    <img src="bag2vid/resources/logo/bag2vid-wordmark-light.svg" alt="bag2vid" width="420">
  </picture>
</p>

<p align="center"><em>A GUI tool to extract videos from a rosbag. Supports ROS 2 Jazzy with MCAP bag formats.</em></p>

> **Looking for the ROS 1 (Melodic) version?** See the [`ros/melodic`](https://github.com/stathiw/bag2vid/tree/ros1/melodic) branch.

<p align="center">
  <img src="docs/images/bag2vid.png" alt="bag2vid screenshot" width="100%">
</p>

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

## Build

### Docker

Build or pull the docker image:
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
docker compose up
```

## How to use

1. Click **_Load Bag_** and select a rosbag. All camera topics are detected automatically.
2. Pick a camera topic from the dropdown.
3. Use **_Play_** / **_Pause_** to preview. Adjust playback rate with the speed selector.
4. Drag the start and end markers on the timeline to set the clip range. Each marker shows its current timestamp.
5. Click **_Extract Video_** and choose where to save the clip.
6. Press **_Capture Screenshot_** at any time to save the current frame as a PNG.

## ⚖️ License & Terms of Use

This project is licensed under a **Copyleft + Non-Commercial Redistribution** model.

### 🟢 You CAN:
* **Use it for work:** Use this tool internally at your company for free.
* **Modify it:** Change the code to fit your needs (as long as those changes stay open-source).
* **Share it:** Give the code to others, provided you keep my name on it and use this same license.

### 🔴 You CANNOT:
* **Sell it:** You cannot charge people to download this software.
* **SaaS it:** You cannot build a website that charges people to use this software's features.
* **Close the Source:** You cannot take this code and put it into a "closed" proprietary product.

### 💰 Want to sell this?
If you want to integrate this into a paid product or service, you need a **Commercial Waiver**. This will involve a royalty agreement or a licensing fee. 

Reach out at: **stathi.weir@gmail.com**
