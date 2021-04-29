// #include <sim/RBUtils.h>
#include <sim/RB.h>
#include <utils/utils.h>

namespace crl {

#define UPDATE_RAY_INTERSECTION(P1, P2)                                        \
    if (ray.getDistanceToSegment(P1, P2, &tmpIntersectionPoint) < cylRadius) { \
        double t = ray.getRayParameterFor(tmpIntersectionPoint);               \
        if (t < tMin) {                                                        \
            intersectionPoint = ray.origin + ray.dir * t;                      \
            tMin = t;                                                          \
        }                                                                      \
    }

bool RB::getRayIntersectionPoint(const Ray &ray, P3D &intersectionPoint) {
    P3D tmpIntersectionPoint;
    double tMin = DBL_MAX;
    double t = tMin;

    // for convenience... consider rb as a cylinder with r = 0.12
    double cylRadius = 0.12;

    P3D startPos = state.getWorldCoordinates(P3D(0, -0.12, 0));
    P3D endPos = state.getWorldCoordinates(P3D(0, 0.12, 0));
    UPDATE_RAY_INTERSECTION(startPos, endPos);

    return tMin < DBL_MAX / 2.0;
}

}  // namespace crl
