#ifndef __MORIN_PARTICLE_FILTER_GRIDMAP_3D_H__
#define __MORIN_PARTICLE_FILTER_GRIDMAP_3D_H__
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <particle_filter_3d/data_types/particle.h>
using namespace std;
class GridMap3D{
    private:
        Eigen::Vector3d origin_;
        vector<bool> occupied_;
        double resolution_;
        size_t x_grids_, y_grids_, z_grids_;

        int toIndex(int xid, int yid, int zid);
    public:
        GridMap3D();

        ~GridMap3D();

        void initialize(double x_size, double y_size_, double z_size_, double resolution);

        void generateSubmap(const pcl::PointCloud<pcl::PointXYZI>& pcd_map, const Eigen::Matrix4d& robot_pose);

        void updateScore(const pcl::PointCloud<pcl::PointXYZI>& robot_tf_lidar, Particle& p);
};

#endif