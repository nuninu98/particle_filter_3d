#include <particle_filter_3d/data_types/particle.h>
using namespace std;
namespace PARTICLE_FILTER_3D{
    Particle::Particle(): weight_(1.0), pose_(Eigen::Matrix4d::Identity()){

    }

    Particle::Particle(const Eigen::Matrix4d& pose): weight_(1.0), pose_(pose){

    }

    Particle::~Particle(){

    }

    void Particle::predict(const Eigen::Matrix4d& motion_gap){
        pose_ = pose_ * motion_gap;
    }


    double Particle::getWeight() const{
        return weight_;
    }

    void Particle::setWeight(const double& weight){
        weight_ = weight;
    }

    Eigen::Matrix4d Particle::getPose() const{
        return pose_;
    }

    Particle::Particle(const Particle& p): weight_(p.weight_), pose_(p.pose_){

    }

    Particle& Particle::operator=(const Particle& p){
        pose_ = p.pose_;
        weight_ = p.weight_;
        return *this;
    }
}