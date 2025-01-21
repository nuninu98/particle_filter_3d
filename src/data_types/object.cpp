#include <particle_filter_3d/data_types/object.h>

namespace PARTICLE_FILTER_3D{
    Object::Object(){

    }

    Object::Object(const string& name, const gtsam_quadrics::ConstrainedDualQuadric& Q): name_(name), Q_(Q){

    }

    Object::~Object(){

    }

    gtsam_quadrics::ConstrainedDualQuadric Object::Q() const{
        return Q_;
    }

    string Object::name() const{
        return name_;
    }
}