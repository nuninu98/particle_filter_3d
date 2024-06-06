#ifndef __MORIN_GICP_LOCALIZATION_H__
#define __MORIN_GICP_LOCALIZATION_H__

using namespace std;

#include <vector>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <omp.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include <random>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <thread>
#include <condition_variable>
#include <queue>
#include <pcl/registration/gicp.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class GicpLocalization{
    private:
        ros::NodeHandle nh_, pnh_;
        ros::CallbackQueue queue_;
        ros::AsyncSpinner spinner_;
        ros::Subscriber sub_lidar_;
        ros::Subscriber sub_odom_;
        ros::Subscriber sub_initpose_;
        Eigen::Matrix4d pose_;
        Eigen::Matrix4d tf_lidar_robot_;
        Eigen::Matrix4d last_submap_update_pose_;

        mutex mtx_;
        pcl::PointCloud<pcl::PointXYZI> map_; 
        nav_msgs::Odometry last_odom_;

        pcl::VoxelGrid<pcl::PointXYZI> voxel_;

        thread submap_thread_;
        double submap_radius_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr submap_;
        queue<Eigen::Matrix4d> submap_flag_queue_;
        condition_variable submap_cv_;
        mutex submap_mtx_, pose_mtx_;

        mutex kill_mtx_;
        condition_variable kill_cv_;
        bool kill_flag_, kill_done_;

        ros::Publisher pub_map_;
        ros::Publisher pub_query_;
        ros::Publisher pub_target_;
        tf2_ros::TransformBroadcaster broadcaster_;

        ros::Publisher pub_pose_;
        bool pub_odom_tf_;

        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp_;

        void odomCallback(const nav_msgs::OdometryConstPtr& odom);

        void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);
    
        Eigen::Matrix4d odom2SE3(const nav_msgs::Odometry& odom);

        void submapFlagCallback();

        void waitForKill();

        void addSubmapFlag(const Eigen::Matrix4d& pose);

        void pubPoseTF();

        void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_2d);

    public:
        GicpLocalization();

        ~GicpLocalization();

        void initialize(const Eigen::Matrix4d& pose);
};

#endif