#ifndef __MORIN_PARTICLE_FILTER_GRIDMAP_3D_H__
#define __MORIN_PARTICLE_FILTER_GRIDMAP_3D_H__
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <particle_filter_3d/data_types/particle.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>

using namespace std;
namespace PARTICLE_FILTER_3D{
    class GridMap3D{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        private:
            Eigen::Vector3d origin_;
            vector<uint8_t> occupied_;
            double resolution_;
            size_t x_grids_, y_grids_, z_grids_;

            int toIndex(int xid, int yid, int zid);
            mutex lock_;
        public:
            GridMap3D();

            GridMap3D(const GridMap3D& map);

            ~GridMap3D();

            void initialize(double x_size, double y_size, double z_size, double resolution);

            void generateSubmap(const pcl::PointCloud<pcl::PointXYZI>& pcd_map, const Eigen::Matrix4d& robot_pose);

            void setSemanticLabel(const pcl::PointCloud<pcl::PointXYZI>& object_cloud, const uint8_t& label);

            void updateScore(const pcl::PointCloud<pcl::PointXYZI>& robot_tf_lidar, Particle& p);
    };
}

#endif