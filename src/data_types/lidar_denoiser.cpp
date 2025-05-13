#include <particle_filter_3d/data_types/lidar_denoiser.h>

namespace PARTICLE_FILTER_3D{
    LidarDenoiser::LidarDenoiser(): min_bearing_(-M_PI), max_bearing_(M_PI), vertical_rays_(32), horizontal_rays_(1024),
    min_altitude_(-15.0*M_PI/180.0), max_altitude_(15.0*M_PI/180.0), kernel_(5)
    {

    }

    LidarDenoiser::~LidarDenoiser(){

    }

    void LidarDenoiser::setHorizontalRays(size_t N){
        horizontal_rays_ = N;
        return;
    }

    void LidarDenoiser::setVerticalRays(size_t N){
        vertical_rays_ = N;
        return;
    }

    void LidarDenoiser::setBearingRange(double min_bearing, double max_bearing){
        min_bearing_ = min_bearing;
        max_bearing_ = max_bearing;
        return;
    }

    void LidarDenoiser::setAltitudeRange(double min_altitude, double max_altitude){
        min_altitude_ = min_altitude;
        max_altitude_ = max_altitude;
        return;
    }

    void LidarDenoiser::filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, pcl::PointCloud<pcl::PointXYZI>& output){
        cv::Mat lidar_img = cv::Mat::zeros(cv::Size(horizontal_rays_, vertical_rays_), CV_32F);
        double res_bear = (max_bearing_ - min_bearing_) / horizontal_rays_;
        double res_alt = (max_altitude_ - min_altitude_) / vertical_rays_;
        for(const auto& pt : input->points){
            double bearing = atan2(pt.y, pt.x);
            double altitude = atan2(pt.z, sqrt(pt.x*pt.x + pt.y*pt.y));
            float range = sqrt(pt.x*pt.x + pt.y*pt.y+ pt.z*pt.z);
            int col = (bearing - min_bearing_) / (res_bear);
            int row =  (altitude - min_altitude_) / (res_alt);
            lidar_img.at<float>(row, col) = range;
        }

        cv::Mat lidar_denoise;
        cv::medianBlur(lidar_img, lidar_denoise, kernel_);
        output.clear();
        for(int r = 0; r < vertical_rays_; ++r){
            for(int c = 0; c < horizontal_rays_; ++c){
                float range = lidar_denoise.at<float>(r, c);
                double bear = min_bearing_ + c * res_bear;
                double alt = min_altitude_ + r * res_alt;
                if(range < 1.0e-2){
                    continue;
                }
                
                pcl::PointXYZI pt;
                pt.x = range*cos(alt) *cos(bear);
                pt.y = range*cos(alt)*sin(bear);
                pt.z = range * sin(alt);
                output.push_back(pt);
            }
        }

        // cv::Mat lidar_img_scaled, lidar_denoise_scaled;
        // cv::Mat lidar_img_view, lidar_denoise_view;
        // cv::convertScaleAbs(lidar_img, lidar_img_scaled, 255/30.0);
        // cv::convertScaleAbs(lidar_denoise, lidar_denoise_scaled, 255/30.0);
        
        // cv::flip(lidar_img_scaled, lidar_img_view, 0);
        // cv::flip(lidar_denoise_scaled, lidar_denoise_view, 0);
        // cv::imshow("NOISY", lidar_img_view);
        // cv::imshow("DENOISED", lidar_denoise_view);
        // cv::waitKey(1);
    }

    void LidarDenoiser::setKernelSize(int size){
        kernel_ = size;
        return;
    }
}