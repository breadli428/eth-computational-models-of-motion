#include "Linkage.h"

using Eigen::Vector2d;
using Eigen::Matrix2d;
typedef std::array<Matrix2d, 2> Tensor2x2x2;

/* Returns the 3 points {p0, p1, p2} of the linkage
 * given two angles {a0, a1}
 * Note: p0 is given by the Linkage
 *
 *                  o < p2 = end-effector position
 *                 /
 *                / bar2
 *   a1    bar1  /
 *   o----------o < a2
 *   ^          ^
 *   p0         p1
 *
 */
std::array<Vector2d, 3> forwardKinematics(const Linkage &linkage, const Vector2d &angles) {
    // 1 - Forward Kinematics
    // put your code in this function
    Vector2d p0 = linkage.p0;
    Vector2d p1 = p0 + Vector2d(0, linkage.length[0]);
    Vector2d p2 = p1 + Vector2d(0, linkage.length[1]);
    return {p0, p1, p2};
}

Vector2d endEffectorPosition(const Linkage &linkage, const Vector2d &angles) {
    return forwardKinematics(linkage, angles)[2];
}

/* Return the (analytically computed) Jacobian of end-effector position
 * given the angles.
 *
 * Note: end-effector position = forwardKinematics(angles)[2]
 *
 *  d forwardKinematics[2]  <-- end effector position
 *  ----------------------
 *      d angles            <-- wrt. to both angles
 *
 * Hint: this might be useful for IK.
 */
Matrix2d dendEffector_dangles(const Linkage &linkage, const Vector2d &angles) {

    // 2 - Derivatives of Forward Kinematics
    // put your code in this function

    Matrix2d dp2_dangles = Matrix2d::Zero();

    return dp2_dangles;
}

/* Return the (analytically computed) 2nd order derivative of end-effector position
 * given the angles.
 *
 * Note: end-effector position = forwardKinematics(angles)[2]
 *
 *  d^2 forwardKinematics[2]  <-- end effector position
 *  ----------------------
 *      d angles^2            <-- wrt. to both angles
 *
 * Hint: this might be useful for IK.
 */
Tensor2x2x2 ddendEffector_ddangles(const Linkage &linkage, const Vector2d &angles) {

    // 2 - Derivatives of Forward Kinematics
    // put your code in this function
    Tensor2x2x2 tensor;
    tensor[0] = tensor[1] = Matrix2d::Zero();
    return tensor;
}
