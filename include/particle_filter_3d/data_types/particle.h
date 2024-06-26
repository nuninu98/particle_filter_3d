#ifndef __MORIN_PARTICLE_FILTER_PARTICLE_H__ 
#define __MORIN_PARTICLE_FILTER_PARTICLE_H__
#include <Eigen/Dense>
#include <Eigen/Core>
class Particle{
    private:
        Eigen::Matrix4d pose_;
        double weight_;
    public:
        Particle();

        Particle(const Particle& p);

        Particle(const Eigen::Matrix4d& pose);

        ~Particle();

        void predict(const Eigen::Matrix4d& motion_gap);

        double getWeight() const;

        void setWeight(const double& weight);

        Eigen::Matrix4d getPose() const;
};

#endif