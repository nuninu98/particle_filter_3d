# particle_filter_3d

## Introduction
This package provides 3D LiDAR based Monte Carlo Localization algorithm. It tracks robot's 3D pose using odometry and 3D LiDAR data input. 

## Prerequisites
1. ROS noetic
2. PCL >= 1.8
3. Eigen >= 3.3
4. Point cloud map (.pcd)

## Install
```
cd ~/catkin_ws/src

git clone https://github.com/nuninu98/particle_filter_3d.git

cd ~/catkin_ws && catkin_make
```

## ROS API
### Subscribe
- /velodyne_points (sensor_msgs/PointCloud2)
  - can be modified in param/config.yaml (lidar_topic)
- /odometry/filtered (nav_msgs/Odometry)
  - can be modified in param/config.yaml (odom_topic)
### Publish
- /localization_pose (nav_msgs/Odometry)
- /tf

## Setting
The algorithm setting can be modified in param/config.yaml file.
- lidar_topic: Subscribing LiDAR topic's name
- odom_topic: Subscribing wheel odometry topic's name
- map_file: Point cloud map file directory
- publish_odom_tf: If true, odom -> baselink tf is generated
- num_partices: Particle numbers

## Demo and Experiment
The command below can run the localization. 
```
roslaunch particle_filter_3d particle_filter.launch
```
Experiment result is posted as the paper:


[Dynamic Environment Robust 3D LiDAR Based Monte Carlo Localization.pdf](https://github.com/user-attachments/files/15924770/Dynamic.Environment.Robust.3D.LiDAR.Based.Monte.Carlo.Localization.pdf)

Demo GIF


![particle filter](https://github.com/nuninu98/particle_filter_3d/assets/36870891/81c6ba3e-2962-4cfb-a360-70ef27d6d3da)
