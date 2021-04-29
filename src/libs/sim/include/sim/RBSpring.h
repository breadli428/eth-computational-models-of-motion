#pragma once

#include <sim/RB.h>

namespace crl {

class RBSpring
{
public:
    RBSpring() {};

    ~RBSpring() {};

public:
    // parent rigid body
    // if parent is nullptr, this spring is fixed to world
    RB *parent = nullptr;
    // this is the location of the joint on the parent - expressed in the
    // parent's local coordinates
    // if parent is nullptr, this spring is fixed to world:
    P3D pJPos = P3D(0, 0, 0);
    // this is the child link
    RB *child = nullptr;
    // this is the location of the joint on the child - expressed in the child's
    // local coordinates
    P3D cJPos = P3D(0, 0, 0);

    // spring constant
    double k = 1;
    // rest length
    double l0 = 1;

};

}