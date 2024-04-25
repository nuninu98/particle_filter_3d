#include <particle_filter_3d/api_class/particle_filter.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "test_particle_filter");
    ParticleFilter pf;

    ros::spin();
}