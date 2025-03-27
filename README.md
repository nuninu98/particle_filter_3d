# particle_filter_3d

## Introduction
This package provides 3D LiDAR based Monte Carlo Localization algorithm. It tracks robot's 3D pose using odometry and 3D LiDAR data input. 

## Prerequisites
1. ROS noetic
2. PCL >= 1.8
3. Eigen >= 3.3
4. Point cloud map (.pcd)
5. Semantic point cloud map (Optional)

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
- /initialpose (geometry_msgs::PoseWithCovarianceStamped)
  - Used for initial pose guess and initialization. 
  - Recommend to use by RVIZ.
  ![particle filter](https://github.com/user-attachments/assets/603fefd0-4ad9-43c5-9679-65cfef9ccccd)
### Publish
- /localization_pose (nav_msgs/Odometry)
- /tf
- /particles (for vislualization)

## Setting
```
lidar_topic: /velodyne_points
map_folder: "/home/nuninu98/catkin_ws/src/LIO-SAM-Semantic/maps/new"
map_file: "GlobalMap.pcd"
objects: ["person", "cell phone", "chair", "bench", "tv"]
#map_file: "/home/nuninu98/catkin_ws/src/LIO-SAM-Semantic/maps/new/GlobalMap.pcd"
#object_file: "/home/nuninu98/catkin_ws/src/LIO-SAM-Semantic/maps/objects.txt"
publish_odom_tf: true
num_particles: 2000
alpha_v: 3.0
alpha_w: 2.0
imu_preintegration:
  enable: true
  imu_topic: imu/data
odom_topic: jackal_velocity_controller/odom # if disable the imu_preintegration
camera:
  enable: true
  R: [0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0]
  t: [0.0, 0.0, 0.5]
  K: [528.433756558705, 0.0, 320.5, 0.0, 528.433756558705, 240.5, 0.0, 0.0, 1.0]
```
The algorithm setting can be modified in param/config.yaml file.
- lidar_topic: Subscribing LiDAR topic's name
- odom_topic: Subscribing wheel odometry topic's name. Only if the IMU preintegration is disabled.
- map_file: Point cloud map file directory
- publish_odom_tf: If true, odom -> baselink tf is generated
- num_partices: Particle numbers
- alpha_v : Particles spreads wider in the case of translational movement.
- alpha_w : Particles spreads wider in the case of rotational movement.

- imu_preintegration/enable : 

## Demo and Experiment
The command below can run the localization. 
```
roslaunch particle_filter_3d particle_filter.launch
```
Experiment result is posted as the paper:


[Dynamic Environment Robust 3D LiDAR Based Monte Carlo Localization.pdf](https://github.com/user-attachments/files/15924770/Dynamic.Environment.Robust.3D.LiDAR.Based.Monte.Carlo.Localization.pdf)

Demo GIF


![particle filter](https://github.com/nuninu98/particle_filter_3d/assets/36870891/81c6ba3e-2962-4cfb-a360-70ef27d6d3da)
