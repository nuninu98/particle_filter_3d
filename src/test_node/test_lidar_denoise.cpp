#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <particle_filter_3d/data_types/lidar_denoiser.h>
using namespace std;
int N_horizontal = 1024;
int N_vertical = 32;
double min_bear = -M_PI;
double max_bear = M_PI;

double max_alt = 15.0 * M_PI/180.0;
double min_alt = -15.0 * M_PI/180.0;

double res_bear = (max_bear - min_bear) / N_horizontal;
double res_alt = (max_alt - min_alt) / N_vertical;
PARTICLE_FILTER_3D::LidarDenoiser denoiser;
ros::Publisher pub_denoise;

void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& lidar){
    ros::Time begin = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZI>::Ptr noisy_lidar(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*lidar, *noisy_lidar);
    denoiser.filter(noisy_lidar, *noisy_lidar);
    
    sensor_msgs::PointCloud2 restored_ros;
    pcl::toROSMsg(*noisy_lidar, restored_ros);
    restored_ros.header = lidar->header;
    pub_denoise.publish(restored_ros);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "test_lidar_denoise");
    ros::NodeHandle nh;
    pub_denoise = nh.advertise<sensor_msgs::PointCloud2>("restored", 1);
    ros::Subscriber sub_lidar = nh.subscribe("points_degen", 1, lidarCallback);

    ros::spin();
}