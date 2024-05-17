#include <particle_filter_3d/api_class/gicp_localization.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "test_gicp_localization");
    GicpLocalization gloc;
    gloc.initialize(Eigen::Matrix4d::Identity());
    ros::spin();
}