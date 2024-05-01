#include <particle_filter_3d/data_types/particle.h>

Particle::Particle(): weight_(0.0), pose_(Eigen::Matrix4d::Identity()){

}

Particle::~Particle(){

}

void Particle::predict(const Eigen::Matrix4d& motion_gap){

}


double Particle::getWeight() const{

}

void Particle::setWeight(const double& weight){
    weight_ = weight;
}