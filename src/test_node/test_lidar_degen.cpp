#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <random>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
using namespace std;

double range = 10.0;
random_device rd;
double degen_rate = 0.3;
ros::Publisher pub_degen;

void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud){
    pcl::PointCloud<pcl::PointXYZI> raw_lidar;
    pcl::fromROSMsg(*cloud, raw_lidar);
    
    mt19937 gen(rd());

    uniform_real_distribution<> ud_r(0.0, range);
    uniform_real_distribution<> ud_bearing(-M_PI,  M_PI);
    uniform_real_distribution<> ud_alt(DEG2RAD(-15.0), DEG2RAD(15.0));

    uniform_int_distribution<> ud_id(0, raw_lidar.size() - 1);
    int N = degen_rate * raw_lidar.size();
    for(int i = 0; i < N; ++i){
        pcl::PointXYZI rand_pt;
        double r = ud_r(gen);
        double theta = ud_bearing(gen);
        double alt = ud_alt(gen);
        rand_pt.x = r*cos(alt)*cos(theta);
        rand_pt.y = r*cos(alt)*sin(theta);
        rand_pt.z = r*sin(alt);

        int rid = ud_id(gen);
        raw_lidar.erase(raw_lidar.begin() + rid);
        raw_lidar.push_back(rand_pt);
    }
    sensor_msgs::PointCloud2 degen_ros;
    pcl::toROSMsg(raw_lidar, degen_ros);
    degen_ros.header = cloud->header;
    pub_degen.publish(degen_ros);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "test_particle_filter");
    ros::NodeHandle nh;
    pub_degen = nh.advertise<sensor_msgs::PointCloud2>("points_degen", 1);
    ros::Subscriber sub_lidar = nh.subscribe("/velodyne_points", 1, lidarCallback);
    ros::spin();
}