#include <particle_filter_3d/api_class/particle_filter.h>

ParticleFilter::ParticleFilter(): queue_(), spinner_(0, &queue_){
    nh_.setCallbackQueue(&queue_);
    sub_lidar_ = nh_.subscribe("lidar_topic", 1, &ParticleFilter::lidarCallback, this);
    sub_odometry_ = nh_.subscribe("odom_topic", 1, &ParticleFilter::odomCallback, this);
    spinner_.start();
}

ParticleFilter::~ParticleFilter(){  
    spinner_.stop();
}

void ParticleFilter::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud){
    unique_lock<mutex> lock(mtx_);
    //========================Particle Estimation========================

    //===================================================================
}

void ParticleFilter::odomCallback(const nav_msgs::OdometryConstPtr& odom){
    unique_lock<mutex> lock(mtx_);
    //=============================Particle Prediction=====================

    //=====================================================================
}