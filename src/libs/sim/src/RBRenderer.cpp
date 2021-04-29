#include "sim/RBRenderer.h"

#include <gui/renderer.h>

#include <memory>

namespace crl {

void RBRenderer::drawCoordFrame(const RB *rb, const gui::Shader &shader) {
    drawArrow3d(rb->state.pos,
                V3D(rb->state.getWorldCoordinates(V3D(1, 0, 0)) * 0.1), 0.01,
                shader, V3D(1.0, 0.0, 0.0));
    drawArrow3d(rb->state.pos,
                V3D(rb->state.getWorldCoordinates(V3D(0, 1, 0)) * 0.1), 0.01,
                shader, V3D(0.0, 1.0, 0.0));
    drawArrow3d(rb->state.pos,
                V3D(rb->state.getWorldCoordinates(V3D(0, 0, 1)) * 0.1), 0.01,
                shader, V3D(0.0, 0.0, 1.0));
}

void RBRenderer::drawCollisionSpheres(const RB *rb, const gui::Shader &shader) {
    for (uint i = 0; i < rb->rbProps.contactPoints.size(); i++) {
        RBContactPoint cs = rb->rbProps.contactPoints[i];
        P3D pos = rb->state.getWorldCoordinates(cs.localCoordinates);
        drawSphere(pos, 0.02, shader, rb->rbProps.colSphereDrawColor);
    }
}

void RBRenderer::drawMOI(const RBState &rbState, const RBProperties &rbProps,
                         const gui::Shader &shader, bool wireFrame) {
    Eigen::EigenSolver<Matrix3x3> eigenvalueSolver(rbProps.MOI_local);

    Eigen::Vector3cd principleMomentsOfInertia = eigenvalueSolver.eigenvalues();

    assert(IS_ZERO(principleMomentsOfInertia[0].imag()) &&
           IS_ZERO(principleMomentsOfInertia[1].imag()) &&
           IS_ZERO(principleMomentsOfInertia[1].imag()));

    Eigen::Matrix3cd V = eigenvalueSolver.eigenvectors();

    double Ixx = principleMomentsOfInertia[0].real();  // = m(y2 + z2)/12
    double Iyy = principleMomentsOfInertia[1].real();  // = m(z2 + x2)/12
    double Izz = principleMomentsOfInertia[2].real();  // = m(y2 + x2)/12

    double x = sqrt((Iyy + Izz - Ixx) * 6 / rbProps.mass);
    double y = sqrt((Izz + Ixx - Iyy) * 6 / rbProps.mass);
    double z = sqrt((Ixx + Iyy - Izz) * 6 / rbProps.mass);

    P3D pmin(-x / 2, -y / 2, -z / 2), pmax(x / 2, y / 2, z / 2);

    if (V.determinant().real() < 0.0) {
        V(0, 2) *= -1;
        V(1, 2) *= -1;
        V(2, 2) *= -1;
    }
    assert(IS_ZERO(abs(V.determinant().real() - 1.0)) &&
           "Rotation matrices have a determinant which is equal to 1.0!");

    Quaternion q(V.real());

    if (wireFrame == false) {
        if (rbProps.selected)
            drawCuboid(rbState.pos, rbState.orientation * q, V3D(x, y, z),
                       shader, rbProps.highlightColor);
        else
            drawCuboid(rbState.pos, rbState.orientation * q, V3D(x, y, z),
                       shader, V3D(0.7, 0.7, 0.7));
    } else
        drawWireFrameCuboid(rbState.pos, rbState.orientation * q, V3D(x, y, z),
                            shader, V3D(0.7, 0.7, 0.7));
}

void RBRenderer::drawMOI(const RB *rb, const gui::Shader &shader) {
    drawMOI(rb->state, rb->rbProps, shader);
}

}  // namespace crl