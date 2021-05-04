#pragma once

#include <sim/RB.h>
#include <sim/RBRenderer.h>
#include <sim/RBSpring.h>
#include <sim/RBUtils.h>

namespace crl {

// subfunctions
Quaternion updateRotationGivenAngularVelocity(const Quaternion &q,
                                              const V3D &angularVelocity,
                                              double dt) {
    double angularVelocityMagnitude = angularVelocity.norm();
    // avoid divide by close to zero...
    if (angularVelocityMagnitude > 1e-10) {
        // TODO: Ex.1 Integration
        // implement quaternion update logic
        // q_p = rot(w, dt) * q
        
        // TODO: fix the following line. 
        Quaternion delta_q;
        delta_q.w() = cos(dt * angularVelocityMagnitude/ 2);
        delta_q.vec() = sin(dt * angularVelocityMagnitude/ 2) * angularVelocity / angularVelocityMagnitude;
        Quaternion q_p = delta_q * q;
        return q_p;
    }
    return q;
}

/**
 * our simulation world governed by the rigid-body dynamics
 */
class RBPhysicsEngine {
public:
    // constructor
    RBPhysicsEngine() {}

    // desctructor
    ~RBPhysicsEngine() {
        // we are going to delete every rigid body when simulation world is
        // destroyed.
        for (uint i = 0; i < rbs.size(); i++) delete rbs[i];
        rbs.clear();
        for (uint i = 0; i < springs.size(); i++) delete springs[i];
        springs.clear();
    }

    /**
     * add rigid body to simulation world.
     * note that we assume this is a cube block with approx. 0.24 x 0.24 x 0.24. 
     */
    RB *addRigidBodyToEngine() {
        rbs.push_back(new RB());
        // we use default mass = 100 kg
        rbs.back()->rbProps.mass = 100;
        rbs.back()->rbProps.id = rbs.size() - 1;
        // add force and torque
        f_ext.push_back(V3D());
        tau_ext.push_back(V3D());
        // add contact points (at vertices)
        rbs.back()->rbProps.contactPoints.push_back({P3D(0.125, 0.125, 0.125)});
        rbs.back()->rbProps.contactPoints.push_back(
            {P3D(0.125, 0.125, -0.125)});
        rbs.back()->rbProps.contactPoints.push_back(
            {P3D(0.125, -0.125, 0.125)});
        rbs.back()->rbProps.contactPoints.push_back(
            {P3D(0.125, -0.125, -0.125)});
        rbs.back()->rbProps.contactPoints.push_back(
            {P3D(-0.125, 0.125, 0.125)});
        rbs.back()->rbProps.contactPoints.push_back(
            {P3D(-0.125, 0.125, -0.125)});
        rbs.back()->rbProps.contactPoints.push_back(
            {P3D(-0.125, -0.125, 0.125)});
        rbs.back()->rbProps.contactPoints.push_back(
            {P3D(-0.125, -0.125, -0.125)});

        return rbs.back();
    }

    /**
     * add spring to simulation world.
     */
    RBSpring *addSpringToEngine(RB *parent, RB *child, P3D pJPos, P3D cJPos) {
        springs.push_back(new RBSpring());
        springs.back()->parent = parent;
        springs.back()->child = child;
        // local position of attached point from parent/child frames
        springs.back()->pJPos = pJPos;
        springs.back()->cJPos = cJPos;
        // we use default spring constant = 2000;
        springs.back()->k = 10000;

        // default rest length is the distance between attaching points when
        // the spring is added.
        if (parent == nullptr) {
            // TODO: Ex.2-1
            // implement your logic for a spring which is attached to world
            // 
            // TODO: Fix the following line
            P3D pJPos_world = pJPos;
            P3D cJPos_world = child->state.getWorldCoordinates(cJPos);
            springs.back()->l0 = V3D(pJPos_world - cJPos_world).norm();
        } else {
            // TODO: Ex.2-2
            // implement your logic for a spring where both ends are attached
            // to rigid bodies
            // 
            // TODO: Fix the following line
            P3D pJPos_world = parent->state.getWorldCoordinates(pJPos);
            P3D cJPos_world = child->state.getWorldCoordinates(cJPos);
            springs.back()->l0 = V3D(pJPos_world - cJPos_world).norm();
        }
        return springs.back();
    }

    /**
     * apply external force (no spring force, no gravity. Force comes from 
     * third sources) to rigid body.
     */
    void applyForceTo(RB *rb, const V3D &f, const P3D &p) {
        // add force only if rb is in rbs
        for (uint i = 0; i < rbs.size(); i++) {
            if (rbs[i] == rb) {
                f_ext[i] += f;
                V3D r = rb->state.getWorldCoordinates(V3D(p));
                tau_ext[i] += r.cross(f);
            }
        }
    }

    /**
     * simulation stepping logic. advance one simulation timestep with dt.
     * the unit of dt is second. 
     */
    void step(double dt) {
        // external force and torque
        for (uint i = 0; i < rbs.size(); i++) {
            // force and torque by gravity
            f_ext[i] += rbs[i]->rbProps.mass * V3D(0, RBGlobals::g, 0);
        }

        // force and torque by springs
        for (RBSpring *spring : springs) {
            // TODO: Ex.2 Spring force
            // compute spring force f_spring = -kx and torque tau_spring and
            // add them to f_ext and tau_ext
            //
            // Hint:
            // - spring->l0 is the rest length and spring->k is the spring contant
            // - you can retrieve index of rb in this->rbs list from rb->rbProps.id

            if (spring->parent == nullptr) {
                // TODO: Ex.2-1
                // implement your logic for a spring which is attached to world
                V3D pJPos_world = V3D(spring->pJPos);
                V3D cJPos_world = V3D(spring->child->state.getWorldCoordinates(spring->cJPos));
                V3D vPos = cJPos_world - pJPos_world;
                V3D f_spring = -spring->k * (vPos - vPos.normalized() * spring->l0);
                V3D tau_spring = (cJPos_world - V3D(spring->child->state.pos)).cross(f_spring);
                auto it = find(rbs.begin(), rbs.end(), spring->child);
                int idx = it - rbs.begin();
                f_ext[idx] += f_spring;
                tau_ext[idx] += tau_spring;
            } else {
                // TODO: Ex.2-2
                // implement your logic for a spring where both ends are attached
                // to rigid bodies.
                V3D pJPos_world = V3D(spring->parent->state.getWorldCoordinates(spring->pJPos));
                V3D cJPos_world = V3D(spring->child->state.getWorldCoordinates(spring->cJPos));
                V3D vPos = cJPos_world - pJPos_world;
                V3D f_spring = -spring->k * (vPos - vPos.normalized() * spring->l0);
                V3D tau_spring_p = (pJPos_world - V3D(spring->parent->state.pos)).cross(f_spring);
                V3D tau_spring_c = (cJPos_world - V3D(spring->child->state.pos)).cross(f_spring);
                auto p_it = find(rbs.begin(), rbs.end(), spring->parent);
                int p_idx = p_it - rbs.begin();
                f_ext[p_idx] -= f_spring;
                tau_ext[p_idx] -= tau_spring_p;
                auto c_it = find(rbs.begin(), rbs.end(), spring->child);
                int c_idx = c_it - rbs.begin();
                f_ext[c_idx] += f_spring;
                tau_ext[c_idx] += tau_spring_c;
            }
        }

        for (uint i = 0; i < rbs.size(); i++) {
            RB *rb = rbs[i];
            // retrieve saved force and tau
            V3D f = f_ext[i];
            V3D tau = tau_ext[i];

            // TODO: Ex.1 Integration
            // implement forward (explicit) Euler integration scheme for computing velocity and pose.
            //
            // Hint:
            // - recall, you need to compute 3x3 moment of inertia matrix expressed in world frame
            //
            // implement your logic here.

            // rb->state.pos = rb->state.pos + dt * rb->state.velocity;
            // Matrix3x3 R = rb->state.orientation.toRotationMatrix();
            // rb->state.orientation = updateRotationGivenAngularVelocity(rb->state.orientation, rb->state.angularVelocity, dt);
            // rb->state.velocity += dt * f / rb->rbProps.mass;
            // Matrix3x3 MOI_world = R * rb->rbProps.MOI_local * R.transpose();
            // rb->state.angularVelocity += dt * MOI_world.inverse() * (tau - rb->state.angularVelocity.cross(V3D(MOI_world * rb->state.angularVelocity)));

            // TODO: Ex.3 Stable Simulation
            // why our simulation is blown up? let's make it more stable!
            //
            // comment out (do not erase!) your logic for Ex.1 and implement Ex.3 here.

            rb->state.velocity += dt * f / rb->rbProps.mass;
            Matrix3x3 R = rb->state.orientation.toRotationMatrix();
            Matrix3x3 MOI_world = R * rb->rbProps.MOI_local * R.transpose();
            rb->state.angularVelocity += dt * MOI_world.inverse() * (tau - rb->state.angularVelocity.cross(V3D(MOI_world * rb->state.angularVelocity)));
            rb->state.pos = rb->state.pos + dt * rb->state.velocity;
            rb->state.orientation = updateRotationGivenAngularVelocity(rb->state.orientation, rb->state.angularVelocity, dt);

            // TODO: Ex.4 Impulse-based Collisions
            // we will simulate collisions between rigidbodies and the ground plane.
            // implement impulse-based collisions here. use coefficient of
            // restituation "epsilon" and coefficient of friction "mu".
            // note that we ignore collisions between rigidbodies.
            //
            // Hint:
            // - read the material "ImpulseBasedCollisions" on CMM21 website carefully.
            // - detect collision by checking if the y coordinate of predefined
            // contact points < 0 (under the ground) or not. each rb has 8 contact points.
            // we will assume that collisions only happen at the contact points.
            // - there could be multiple contact points under the ground at the sametime,
            // but we will assume there's only one contact between a single ground~rb pair
            // at once: choose the contact points which is the lowest in y coordinate.

            if (simulateCollisions) {
                // 
                // Ex.4 implementation here
                //
                for (uint i = 0; i < rbs.size(); i++) {
                    RB *rb = rbs[i];
                    std::vector<RBContactPoint> contactPoints = rb->rbProps.contactPoints;
                    double min_y = 1;
                    int min_j = -1;
                    P3D min_y_world = P3D(0, 0, 0);
                    for (uint j = 0; j < contactPoints.size(); j++)
                    {
                        P3D y_local = contactPoints[j].localCoordinates;
                        P3D y_world = rb->state.getWorldCoordinates(y_local);
                        if (y_world.y < min_y)
                        {
                            min_y = y_world.y;
                            min_j = j;
                            min_y_world = y_world;
                        }
                    }
                    if (min_y <= 0)
                    {
                        Matrix3x3 K_ground = Matrix3x3::Zero();
                        Matrix3x3 R = rb->state.orientation.toRotationMatrix();
                        Matrix3x3 MOI_world = R * rb->rbProps.MOI_local * R.transpose();
                        P3D offset = min_y_world - rb->state.pos;
                        Matrix3x3 offset_skew = getSkewSymmetricMatrix(V3D(offset));
                        Matrix3x3 K_rb = (Matrix3x3::Identity() / rb->rbProps.mass - offset_skew * MOI_world.inverse() * offset_skew);
                        Matrix3x3 K_T = K_ground + K_rb;
                        V3D N = V3D(0, 1, 0);
                        V3D u_rel = rb->state.getVelocityForPoint_global(min_y_world);
                        V3D u_rel_n = u_rel.dot(N) * N.normalized();
                        V3D u_rel_t = u_rel - u_rel_n;
                        V3D T = u_rel_t.normalized();
                        V3D J_inf = K_T.inverse() * (-u_rel - eps * u_rel.dot(N) * N.normalized());
                        V3D J;
                        if ((J_inf - J_inf.dot(N) * N.normalized()).norm() <= mu * (J_inf.dot(N) * N.normalized()).norm())
                        {
                            J = J_inf;
                        }
                        else
                        {
                            double j_n = -(eps + 1) * u_rel.dot(N) / (N.transpose() * K_T * (N - T * mu));
                            J = N * j_n - T * mu * j_n;
                        }
                        rb->state.velocity += J / rb->rbProps.mass;
                        rb->state.angularVelocity += MOI_world.inverse() * offset_skew * J;
                    }
                }
            }
        }

        // clean up
        for (uint i = 0; i < f_ext.size(); i++) {
            f_ext[i] = V3D();
            tau_ext[i] = V3D();
        }
    }

    /**
     * draw every rigid body belongs to world.
     */
    inline void draw(const gui::Shader &rbShader) {
        // draw moi boxes
        for (uint i = 0; i < this->rbs.size(); i++) {
            if (!this->rbs[i]->rbProps.fixed)
                crl::RBRenderer::drawMOI(this->rbs[i], rbShader);
        }

        // draw springs
        for (uint i = 0; i < this->springs.size(); i++) {
            P3D start, end;
            if (this->springs[i]->parent == nullptr) {
                start = this->springs[i]->pJPos;
                end = this->springs[i]->child->state.getWorldCoordinates(
                    this->springs[i]->cJPos);
            } else {
                start = this->springs[i]->parent->state.getWorldCoordinates(
                    this->springs[i]->pJPos);
                end = this->springs[i]->child->state.getWorldCoordinates(
                    this->springs[i]->cJPos);
            }
            drawCylinder(start, end, 0.05, rbShader);
        }

        // then draw collsion spheres
        if (showCollisionSpheres)
            for (uint i = 0; i < this->rbs.size(); i++)
                crl::RBRenderer::drawCollisionSpheres(this->rbs[i], rbShader);

        // and now coordinate frames
        if (showCoordFrame) {
            for (uint i = 0; i < this->rbs.size(); i++)
                crl::RBRenderer::drawCoordFrame(this->rbs[i], rbShader);
        }
    }

    /**
     * returns NULL if no RBs are hit by the ray...
     */
    RB *getFirstRBHitByRay(const Ray &ray, P3D &intersectionPoint) {
        RB *selectedRB = nullptr;
        double t = DBL_MAX;
        P3D tmpIntersectionPoint = P3D(0, 0, 0);

        for (uint i = 0; i < rbs.size(); i++) {
            if (rbs[i]->getRayIntersectionPoint(ray, tmpIntersectionPoint)) {
                double tTmp = ray.getRayParameterFor(tmpIntersectionPoint);
                if (tTmp < t) {
                    selectedRB = rbs[i];
                    t = tTmp;
                    intersectionPoint = tmpIntersectionPoint;
                }
            }
        }
        return selectedRB;
    }

public:
    // this is a list of all rigid bodies and springs belong to the world.
    std::vector<RB *> rbs;
    std::vector<RBSpring *> springs;

    // coefficients
    float eps = 0.0;  // restitution
    float mu = 0.8;   // friction

    // drawing flags
    bool showCollisionSpheres = false;
    bool showCoordFrame = false;

    // options
    bool simulateCollisions = false;

private:
    // list of force and tau applied to rigid bodies
    std::vector<V3D> f_ext;
    std::vector<V3D> tau_ext;
};
}  // namespace crl