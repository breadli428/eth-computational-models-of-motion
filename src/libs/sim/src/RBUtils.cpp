#include <sim/RBUtils.h>
#include <utils/utils.h>

namespace crl {

// assume that the gravity is in the y-direction (this can easily be changed if
// need be), and this value gives its magnitude.
double RBGlobals::g = -9.8;
// this is the direction of the up-vector
V3D RBGlobals::worldUp = V3D(0, 1, 0);
// and the ground plane
Plane RBGlobals::groundPlane = Plane(P3D(0, 0, 0), RBGlobals::worldUp);

}  // namespace crl
