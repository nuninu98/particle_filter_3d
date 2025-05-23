#include <particle_filter_3d/data_types/gridmap_3d.h>
namespace PARTICLE_FILTER_3D{
    GridMap3D::GridMap3D(){
        origin_ = Eigen::Vector3d::Zero();
    }

    GridMap3D::GridMap3D(const GridMap3D& map): origin_(map.origin_), resolution_(map.resolution_),
    x_grids_(map.x_grids_), y_grids_(map.y_grids_), z_grids_(map.z_grids_){
        occupied_ = map.occupied_;
    }

    GridMap3D::~GridMap3D(){

    }

    int GridMap3D::toIndex(int xid, int yid, int zid){
        return xid + yid * x_grids_ + zid * (x_grids_*y_grids_);
    }

    void GridMap3D::initialize(double x_size, double y_size, double z_size, double resolution){
        unique_lock<mutex> lock(lock_);
        resolution_ = resolution;
        x_grids_ = x_size / resolution;
        y_grids_ = y_size / resolution;
        z_grids_ = z_size / resolution;
        occupied_.resize(x_grids_ * y_grids_ * z_grids_, false);
    }

    void GridMap3D::generateSubmap(const pcl::PointCloud<pcl::PointXYZI>& pcd_map, const Eigen::Matrix4d& robot_pose){
        unique_lock<mutex> lock(lock_);
        occupied_.resize(x_grids_ * y_grids_ * z_grids_, false);
        Eigen::Vector3d tvec = robot_pose.block<3, 1>(0, 3);
        double x_size = resolution_ * x_grids_;
        double y_size = resolution_ * y_grids_;
        double z_size = resolution_ * z_grids_;
        origin_ = tvec - Eigen::Vector3d(x_size/2.0, y_size/ 2.0, z_size/2.0);
        for(const auto& pt : pcd_map){
            int xid = (pt.x - origin_(0))/resolution_;
            int yid = (pt.y - origin_(1))/resolution_;
            int zid = (pt.z - origin_(2))/resolution_;
            if(xid < 0 || xid >= x_grids_ || yid < 0 || yid >= y_grids_ || zid < 0 || zid >= z_grids_){
                continue;
            }
            if(toIndex(xid, yid, zid) >= occupied_.size()){
                continue;
            }
            occupied_[toIndex(xid, yid, zid)] = true;
        } 
    }


    void GridMap3D::updateScore(const pcl::PointCloud<pcl::PointXYZI>& robot_tf_lidar, Particle& p){
        unique_lock<mutex> lock(lock_);
        pcl::PointCloud<pcl::PointXYZI> tf_cloud;
        pcl::transformPointCloud(robot_tf_lidar, tf_cloud, p.getPose());
        int weight = 0.0;
        for(size_t i = 0; i < tf_cloud.size(); ++i){
            int xid = (tf_cloud[i].x - origin_(0))/resolution_;
            int yid = (tf_cloud[i].y - origin_(1))/resolution_;
            int zid = (tf_cloud[i].z - origin_(2))/resolution_;
            if(xid < 0 || xid >= x_grids_ || yid < 0 || yid >= y_grids_ || zid < 0 || zid >= z_grids_){
                continue;
            }
            int id = toIndex(xid, yid, zid);
            if(id >= occupied_.size()){
                continue;
            }
            if(occupied_[id]){
                weight += 1.0;
                // if(occupied_[id] == (uint8_t)tf_cloud[i].intensity){
                //     weight += 1.0;
                // }
            }
        }
        p.setWeight(pow(weight, 4));
    }
       
}