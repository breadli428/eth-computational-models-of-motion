#pragma once

#include <sim/RBProperties.h>
#include <sim/RBState.h>
#include <utils/geoms.h>
#include <utils/mathUtils.h>

namespace crl {

/**
 * Rigidbody class.
 */
class RB {
public:
    // state of the rigid body
    RBState state;
    // a list of properties for the rigid body
    RBProperties rbProps;
    // name of the rigid body
    std::string name;

public:
    /**
     * Default constructor
     */
    RB(void) {}

    /**
     * Default destructor
     */
    virtual ~RB(void) {}

    /**
     *  returns true if it is hit, false otherwise.
     */
    bool getRayIntersectionPoint(const Ray &ray, P3D &intersectionPoint);
};

}  // namespace crl
