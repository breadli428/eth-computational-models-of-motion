#include <InverseKinematics.h>

#include "TestResult.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>

using namespace tests;

TestResult test_1_forwardKinematics() {
    Linkage linkage;

    VectorXd x(2);
    x << 0.1, 0.2;

    std::array<Vector2d, 3> groundTruth;
    groundTruth[0] <<
               1,
            -1.2;
    groundTruth[1] <<
     1.995004165,
    -1.100166583;
    groundTruth[2] <<
      3.428008899,
    -0.6568862734;

    return SAME(groundTruth[0], forwardKinematics(linkage, x)[0], 1e-8)
          +SAME(groundTruth[1], forwardKinematics(linkage, x)[1], 1e-8)
          +SAME(groundTruth[2], forwardKinematics(linkage, x)[2], 1e-8)
            ;

}

TestResult test_2_dendeffector_dx() {
    Linkage linkage;

    VectorXd x(2);
    x << 0.1, 0.2;

    Matrix2d groundTruth;
    groundTruth << -0.5431137266, -0.44328031,
                   2.428008899, 1.433004734;

    return SAME(groundTruth, dendEffector_dangles(linkage, x), 1e-8);

}

TestResult test_3_ddendeffector_ddx() {
    Linkage linkage;

    VectorXd x(2);
    x << 0.1, 0.2;

    Tensor2x2x2 groundTruth;
    groundTruth[0] <<
                     -2.428008899, -1.433004734,
                    -0.5431137266, -0.44328031;

    groundTruth[1] <<
                    -1.433004734, -1.433004734,
                     -0.44328031,  -0.44328031;
    return SAME(groundTruth[0], ddendEffector_ddangles(linkage, x)[0], 1e-8)
          +SAME(groundTruth[1], ddendEffector_ddangles(linkage, x)[1], 1e-8);

}


TestResult test_4_IKEvaluate(){
    Linkage linkage;
    Vector2d target = {0.45, 1.57};
    InverseKinematics obj(linkage, target);

    VectorXd x(2);
    x << -0.1, 0.35;
    double f = obj.evaluate(x);

    return SAME(f, 7.61694, 1e-4);
}

TestResult test_5_IKGradient() {
    Linkage linkage;
    Vector2d target = {0.1, 0.2};

    VectorXd x(2);
    x << 0.1, 0.2;

    InverseKinematics ik(linkage, target);

    VectorXd groundTruth(2);
    groundTruth <<
            -3.888014813,
            -2.703162902;

    return SAME(groundTruth, ik.gradient(x), 1e-8);
}

TestResult test_6_IKHessian() {
    Linkage linkage;
    Vector2d target = {0.1, 0.2};

    VectorXd x(2);
    x << 0.1, 0.2;

    InverseKinematics ik(linkage, target);

    Matrix2d groundTruth;
    groundTruth <<  -1.424848792,  -0.6691118263,
                    -0.6691118263,  -2.139211693;

    return SAME(groundTruth, ik.hessian(x), 1e-8);

}

int main(int argc, char *argv[])
{

    // 1
    TEST(test_1_forwardKinematics);

    // 2
    TEST(test_2_dendeffector_dx);
    TEST(test_3_ddendeffector_ddx);

    // 3
    TEST(test_4_IKEvaluate);
    TEST(test_5_IKGradient);
    TEST(test_6_IKHessian);


    return (allTestsOk ? 0 : 1);
}

