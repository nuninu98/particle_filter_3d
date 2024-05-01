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
#include <random>

using namespace std;

class ParticleFilter{
    private:
        ros::CallbackQueue queue_;
        ros::AsyncSpinner spinner_;
        vector<Particle> particles_;
        size_t N_particles_;
        ros::NodeHandle nh_, pnh_;
        ros::Subscriber sub_odometry_;
        ros::Subscriber sub_lidar_;
        Eigen::Matrix4d pose_;
        void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

        void odomCallback(const nav_msgs::OdometryConstPtr& odom);
        
        mutex mtx_;

        Eigen::VectorXd toPose6d(const Eigen::Matrix4d& se3);

        Eigen::Matrix4d toSE3(const Eigen::VectorXd& pose6d);

        void calculatePose(); // calculate average
    public:
        ParticleFilter();

        ~ParticleFilter();

        void initialize(const Eigen::Matrix4d& pose);
};

#endif