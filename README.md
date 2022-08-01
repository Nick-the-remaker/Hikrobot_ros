# HIKROBOT ROS
## 杭州海康机器人技术有限公司


深度相机MV-EB435i“悉灵”是一款低成本，小体积，配置全面的立体相机，凭借硬件级的深度图像处理方案，相机可在高性能输出的同时维持低功耗的水平。本例为悉灵相机在ros上的集成。

## 安装引导
```sh
mkdir -p ~/ros_ws/src               
cd ~/ros_ws/src                      
git clone https://github.com/Nick-the-remaker/Hikrobot_ros.git 
cd ~/ros_ws
catkin_make
sudo su
source devel/setup.bash
```
> 编译过程中可能会遇到本机没有相关ros安装包的情况，可以使用命令 sudo apt-get install ros-$ROS_DISTRO-功能包来安装。
例如，sudo apt-get install ros-melodic-pcl-conversions

## 依赖项
+ cv_bridge
+ OpenCV
+ image_transport
+ PCL
+ tf2

## 使用说明
本例提供四个示例程序

**1. 基本信息发布**。包含彩色图，深度图，相机信息，左右红外相机图，点云发布。
```sh
cd ros_ws
source devel/setup.bash
roslaunch hikrobot_ros base_rgbd.launch
```
**2.ROS-OpenCV图像处理界面示例**，使用cvui开发悉灵相机UI界面，拥有多窗口显示，自定义窗口，鼠标交互等功能。
```sh

roslaunch hikrobot_ros with_opencv.launch
```

**3.相机位姿发布**，集成apriltags-cpp，在rviz中察看发布的相机位姿。
**apriltag原作者： Edwin Olson <ebolson@umich.edu>**
**C++端口和修改：Matt Zucker <mzucker1@swarthmore.edu>**
```sh
roslaunch hikrobot_ros cam_pose.launch
```

**4.深度图映射到RGB图**，在rviz中查看映射后的点云。
```sh
roslaunch hikrobot_ros aligned_depth.launch
```

## 发布话题
四个示例程序中所有发布话题：
+ /camera/color/image_raw
+ /camera/color/camera_info
+ /camera/depth/image_rect_raw
+ /camera/depth/rendered_depth
+ /camera/depth/camera_info
+ /camera/infra_left/image_rect_raw
+ /camera/infra_right/image_rect_raw
+ point_cloud
+ aligned_point_cloud
+ TF

## launch文件参数
+ **image_width**: 图像宽，1280或640
+ **image_height**: 图像高，720或360
+ **exposure**: 曝光，范围10-33333
+ **gain**: 增益，范围0-19.99630
+ **enable_laser**: 是否启用激光投射器
+ **enable_color**: 是否发布彩色图
+ **enable_depth**: 是否发布深度图
+ **enable_infra**: 是否发布红外图
+ **enable_pointcloud**: 是否发布点云
+ **color_info**: 彩色相机参数文件路径
+ **depth_info**: 深度相机参数文件路径

---
# HIKROBOT ROS
## HangZhou Hikrobot Technology Co., Ltd.

MV-EB435i 'Xiling' is a low-cost, small volume, fully configured stereo camera.  This camera can maintain a low-power level while maintaining high-performance output with the hardware level depth image processing scheme. This example is the integration of Xiling on ROS.

## Installation Instructions
```sh
mkdir -p ~/ros_ws/src               
cd ~/ros_ws/src                      
git clone https://github.com/Nick-the-remaker/Hikrobot_ros.git 
cd ~/ros_ws
catkin_make
sudo su
source devel/setup.bash
```
> errors may occur if lack of necessary ros packages while compiling, try this command to install: sudo apt-get install ros-ROS_DISTRO-packages
e.g. sudo apt-get install ros-melodic-pcl-conversions


## Dependence
+ cv_bridge
+ OpenCV
+ image_transport
+ PCL
+ tf2

## Usage
These are four examples for using MV-EB435i.

**1. publish basic topics**, including color image, depth image, infra image, and point cloud.

```sh
cd ros_ws
source devel/setup.bash
roslaunch hikrobot_ros base_rgbd.launch
```

**2.OpenCV image-processing interface**, develop the UI interface of Xiling camera based on cvui, which has the functions of multi-window display, custom window, mouse interaction and so on.

```sh
roslaunch hikrobot_ros with_opencv.launch
```

**3.publish camera pose**, view the published camera pose in rviz with apriltags.
**apriltag original author： Edwin Olson <ebolson@umich.edu>**
**C++ port and modifications：Matt Zucker <mzucker1@swarthmore.edu>**
```sh
roslaunch hikrobot_ros cam_pose.launch
```

**4.depth image map to color image**, view the mappped point cloud in rviz.
```sh
roslaunch hikrobot_ros aligned_depth.launch
```

## Publish Topics
These are all published topics from four examples.
+ /camera/color/image_raw
+ /camera/color/camera_info
+ /camera/depth/image_rect_raw
+ /camera/depth/rendered_depth
+ /camera/depth/camera_info
+ /camera/infra_left/image_rect_raw
+ /camera/infra_right/image_rect_raw
+ point_cloud
+ aligned_point_cloud
+ TF

## Launch parameters
+ **image_width**: the width of color/depth image, 1280 | 640 
+ **image_height**: the height of color/depth image, 720 | 360
+ **exposure**: exposure time, range from 10-33333
+ **gain**: the gain of the camera, range from 0-19.99630
+ **enable_laser**: choose whether to enable laser projector
+ **enable_color**: choose whether to allow publishing color image
+ **enable_depth**: choose whether to allow publishing depth image
+ **enable_infra**: choose whether to allow publishing infra image
+ **enable_pointcloud**: choose whether to allow publishing point cloud
+ **color_info**: path to color image camera info
+ **depth_info**: path to depth image camera info