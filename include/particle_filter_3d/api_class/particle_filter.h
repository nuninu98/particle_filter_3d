#ifndef __MORIN_PARTICLE_FILTER_H__
#define __MORIN_PARTICLE_FILTER_H__

#include <particle_filter_3d/data_types/particle.h>
#include <vector>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <omp.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <mutex>

using namespace std;

class ParticleFilter{
    private:
        ros::CallbackQueue queue_;
        ros::AsyncSpinner spinner_;
        vector<Particle> particles_;
        ros::NodeHandle nh_;
        ros::Subscriber sub_odometry_;
        ros::Subscriber sub_lidar_;

        void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

        void odomCallback(const nav_msgs::OdometryConstPtr& odom);
        mutex mtx_;
    public:
        ParticleFilter();
        ~ParticleFilter();
};

#endif