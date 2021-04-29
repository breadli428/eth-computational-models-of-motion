#pragma once

#include <gui/model.h>
#include <utils/mathUtils.h>

#include <memory>

namespace crl {

/**
 * Contact points
 */
struct RBContactPoint {
    P3D localCoordinates;
};

/**
 * This class represents a container for the various properties of a rigid body
 */
class RBProperties {
public:
    // the mass
    double mass = 1.0;
    // we'll store the moment of inertia of the rigid body, in the local
    // coordinate frame
    Matrix3x3 MOI_local = Matrix3x3::Identity();

    // collision primitives that are relevant for this rigid body
    std::vector<RBContactPoint> contactPoints;

    // id of the rigid body
    int id = -1;

    // for selection via GUI
    bool selected = false;

    // for drawing abstract view
    V3D highlightColor = V3D(1.0, 0.5, 0.5);
    V3D colSphereDrawColor = V3D(0.75, 0.0, 0.0);

    // draw color for rb primitive
    V3D color = V3D(0.5, 0.5, 0.5);

    // is this body is fixed to world
    bool fixed = false;

    // physics related coefficients
    double restitutionCoeff = 0;
    double frictionCoeff = 0.8;

public:
    /**
     * default constructor.
     */
    RBProperties() {}

    /**
     * default destructor.
     */
    ~RBProperties() {}

    /**
     * set the moment of inertia of the rigid body - symmetric 3x3 matrix, so
     * we need the six values for it.
     */
    inline void setMOI(double moi00, double moi11, double moi22, double moi01,
                       double moi02, double moi12) {
        MOI_local << moi00, moi01, moi02, moi01, moi11, moi12, moi02, moi12,
            moi22;
    }
};

}  // namespace crl
