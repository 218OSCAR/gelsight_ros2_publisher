# Gelsight_mini ROS2 publisher Setup

This document describes the setup process for using **Gelsight mini** with **ROS 2**.

---

## References

- gs_sdk:  
  https://github.com/joehjhuang/gs_sdk?tab=readme-ov-file

---

## Environment

- OS: Ubuntu 22.04
- ROS 2 (Humble)

---

## Install gs_sdk

Follow the instructions from **gs_sdk**:
install the gs_sdk under the ros2_ws/src:

👉 https://github.com/joehjhuang/gs_sdk?tab=readme-ov-file

---
## Install ffmpeg
You can directly install ffmpeg using:
```
sudo apt install ffmpeg
```
Or you can choose to do it by creating a new conda env:
```
conda create -n gelsight python=3.10 ffmpeg opencv
conda activate gelsight
```


## Clone this repo, run the node and publish ros2 topic 
You have to first clone this repo to your ros2_ws:

```
cd ~/ros2_ws/src/
git clone https://github.com/218OSCAR/gelsight_ros2_publisher.git
```
After that, change the 'config_path'  in ros2_ws/src/gelsight_ros2_publisher/launch/gelsight_publisher.launch.py to your own.
Then connect the gelsight_mini tactile sensor to your PC and run the following code:
```
ros2 launch gelsight_ros2_publisher gelsight_publisher.launch.py
```

## Visulization
You can visulize the tactile image lively by using rqt_image_view:
```
ros2 run rqt_image_view rqt_image_view
```
