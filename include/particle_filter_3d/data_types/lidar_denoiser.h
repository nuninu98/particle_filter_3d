#ifndef __PARTICLE_FILTER_3D_LIDAR_DENOISER_H__
#define __PARTICLE_FILTER_3D_LIDAR_DENOISER_H__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace PARTICLE_FILTER_3D{
    class LidarDenoiser{
        private:
            size_t vertical_rays_;
            size_t horizontal_rays_;

            double min_bearing_;
            double max_bearing_;

            double max_altitude_;
            double min_altitude_;

            int kernel_;
        public:
            LidarDenoiser();

            ~LidarDenoiser();

            void setHorizontalRays(size_t N);

            void setVerticalRays(size_t N);

            void setBearingRange(double min_bearing, double max_bearing);

            void setAltitudeRange(double min_altitude, double max_altitude);

            void setKernelSize(int size);

            void filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, pcl::PointCloud<pcl::PointXYZI>& output);
    };
}

#endif