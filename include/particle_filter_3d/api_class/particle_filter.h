#ifndef __MORIN_PARTICLE_FILTER_H__
#define __MORIN_PARTICLE_FILTER_H__

#include <particle_filter_3d/data_types/particle.h>
#include <particle_filter_3d/data_types/gridmap_3d.h>
#include <particle_filter_3d/data_types/object.h>
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <thread>
#include <condition_variable>
#include <queue>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
#include <string>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <particle_filter_3d/api_class/imu_preintegration.h>

using namespace std;
namespace PARTICLE_FILTER_3D{

    

    class ParticleFilter{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            
        private:

            void voxelize(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>& cloud_out, double leaf_size){
                pcl::PCLPointCloud2Ptr cloud(new pcl::PCLPointCloud2());
                pcl::toPCLPointCloud2(*cloud_in, *cloud);

                pcl::PCLPointCloud2Ptr cloud_ds(new pcl::PCLPointCloud2());
                pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
                vox.setInputCloud(cloud);
                vox.setLeafSize(leaf_size, leaf_size, leaf_size);
                vox.filter(*cloud_ds);
                pcl::fromPCLPointCloud2(*cloud_ds, cloud_out);
            }
            
            ros::CallbackQueue queue_;
            ros::AsyncSpinner spinner_;
            vector<Particle> particles_;
            size_t N_particles_;
            ros::NodeHandle nh_, pnh_;
            ros::Subscriber sub_odometry_;
            ros::Subscriber sub_lidar_;
            Eigen::Matrix4d pose_;
            Eigen::Matrix4d tf_lidar2_robot_;

            tf2_ros::Buffer buffer_;
            tf2_ros::TransformListener listener_;
            tf2_ros::TransformBroadcaster broadcaster_;
            thread submap_thread_;

            GridMap3D grid_submap_;
            queue<Eigen::Matrix4d> submap_flag_queue_;
            condition_variable submap_cv_;
            mutex submap_mtx_;
            Eigen::Matrix4d submap_updated_pose_;

            mutex kill_mtx_;
            condition_variable kill_cv_;
            bool kill_flag_, kill_done_;
            pcl::PointCloud<pcl::PointXYZI> pcd_map_;
            vector<shared_ptr<Object>> objects_;
            nav_msgs::Odometry last_odom_;

            ros::Publisher pub_map_;
            ros::Publisher pub_particles_;
            ros::Publisher pub_map_objects_;

            pcl::VoxelGrid<pcl::PointXYZI> voxel_;

            ros::Subscriber sub_initpose_;
            ros::Time last_tf_stamp_;

            ros::Publisher pub_pose_;


            bool pub_odom_tf_;

            void submapFlagCallback();

            void waitForKill();

            void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

            void odomCallback(const nav_msgs::OdometryConstPtr& odom);
            
            mutex mtx_;

            Eigen::VectorXd toPose6d(const Eigen::Matrix4d& se3);

            Eigen::Matrix4d toSE3(const Eigen::VectorXd& pose6d);

            void calculatePose(); // calculate average

            void publishMap();

            void publishParticle();

            void addSubmapFlag(const Eigen::Matrix4d& pose);

            void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_2d);

            bool loadObjectMap(const string& path);

            double initOdomStamp_ = -1;

            IMUPreintegration* ip_ = nullptr;
        
            double alpha_v_;
            double alpha_w_;
        public:

            ParticleFilter();

            ~ParticleFilter();

            void initialize(const Eigen::Matrix4d& pose);
    };
}
#endif