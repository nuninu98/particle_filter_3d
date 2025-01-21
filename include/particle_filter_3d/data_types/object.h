#ifndef __MORIN_PARTICLE_FILTER_OBJECT_H__
#define __MORIN_PARTICLE_FILTER_OBJECT_H__
#include <gtsam_quadrics/geometry/ConstrainedDualQuadric.h>
#include <string>
using namespace std;
namespace PARTICLE_FILTER_3D{
    class Object{
        private:
            string name_;
            gtsam_quadrics::ConstrainedDualQuadric Q_;
        public:
            Object();

            Object(const string& name, const gtsam_quadrics::ConstrainedDualQuadric& Q);

            ~Object();

            gtsam_quadrics::ConstrainedDualQuadric Q() const;

            string name() const;
    };
}

#endif