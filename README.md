# particle_filter_3d

## Introduction
This package provides 3D LiDAR based Monte Carlo Localization algorithm. It tracks robot's 3D pose using odometry and 3D LiDAR data input. 

## Prerequisites
1. ROS noetic
2. PCL >= 1.8
3. Eigen >= 3.3
4. Point cloud map (.pcd)

## Intallation

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

## Demo and experiment
The command below can run the localization. 
```
roslaunch particle_filter_3d particle_filter.launch
```
Experiment result is posted as the paper:
https://github.com/nuninu98/particle_filter_3d/blob/main/Dynamic%20Environment%20Robust%203D%20LiDAR%20Based%20Monte%20Carlo%20Localization.pdf
