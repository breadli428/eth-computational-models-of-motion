#pragma once

#include <gui/application.h>
#include <gui/camera.h>
#include <gui/light.h>
#include <gui/renderer.h>
#include <gui/shader.h>
#include <sim/RBPhysicsEngine.h>
#include <utils/logger.h>

#include "scenes.h"

using namespace crl::gui;

namespace crl {
namespace app {
namespace rigidbody {

class App : public Basic3DAppWithShadows {
public:
    App(const char *title = "CMM Assignment 5 - Rigid Bodies",
        std::string iconPath = CMM_DATA_FOLDER "/crl_icon_red.png")
        : Basic3DAppWithShadows(title, iconPath) {
        camera = TrackingCamera(5);
        camera.rotAboutUpAxis = 0;
        camera.rotAboutRightAxis = 0.25;
        light.s = 0.04f;
        shadowbias = 0.00001f;
        camera.aspectRatio = float(width) / height;
        glEnable(GL_DEPTH_TEST);

        showConsole = true;
        automanageConsole = true;
        Logger::maxConsoleLineCount = 10;
        consoleHeight = 225;

        this->targetFramerate = 30;
        this->limitFramerate = true;

        // setup world
        setupWorld();
    }

    virtual ~App() override {
        if (physicsEngine) {
            delete physicsEngine;
        }
    }

    virtual void resizeWindow(int width, int height) override {
        camera.aspectRatio = float(width) / height;
        return Application::resizeWindow(width, height);
    }

    bool mouseMove(double xpos, double ypos) override {
        P3D rayOrigin;
        V3D rayDirection;
        camera.getRayFromScreenCoordinates(xpos, ypos, rayOrigin, rayDirection);
        Ray mouseRay(rayOrigin, rayDirection);

        if (mouseState.dragging) {
            if (selectedRB != NULL && mouseState.lButtonPressed == true) {
                mouseRay.getDistanceToPoint(selectedPoint,
                                            &targetForSelectedPoint);
                return true;
            }
        }

        camera.processMouseMove(mouseState, keyboardState);
        return true;
    }

    bool mouseButtonReleased(int button, int mods) override {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            if (selectedRB) {
                selectedRB->rbProps.selected = false;
                selectedRB = nullptr;
            }
        }

        return true;
    }

    virtual bool mouseButtonPressed(int button, int mods) override {
        if (button == GLFW_MOUSE_BUTTON_LEFT ||
            button == GLFW_MOUSE_BUTTON_RIGHT) {
            if (this->selectedRB != NULL)
                this->selectedRB->rbProps.selected = false;
            P3D rayOrigin;
            V3D rayDirection;
            camera.getRayFromScreenCoordinates(mouseState.lastMouseX,
                                               mouseState.lastMouseY, rayOrigin,
                                               rayDirection);
            Ray mouseRay(rayOrigin, rayDirection);

            // ray tracing and check selection
            crl::RB *selectedRB = NULL;

            if (!selectedRB)
                selectedRB =
                    physicsEngine->getFirstRBHitByRay(mouseRay, selectedPoint);

            if (button == GLFW_MOUSE_BUTTON_LEFT) {
                // select RB by left click
                if (selectedRB != NULL) {
                    selectedRB->rbProps.selected = true;
                    selectedPointInLocalCoords =
                        selectedRB->state.getLocalCoordinates(selectedPoint);
                    targetForSelectedPoint = selectedPoint;
                }
                this->selectedRB = selectedRB;
            }
        }
        return true;
    }

    virtual bool scrollWheel(double xoffset, double yoffset) override {
        camera.processMouseScroll(xoffset, yoffset);
        return true;
    }

    void process() override {
        if (appIsRunning == false) return;

        // substeps (simulation)
        double simTime = 0;
        while (simTime < 1.0 / targetFramerate) {
            simTime += dt;

            // apply force
            if (selectedRB)
                physicsEngine->applyForceTo(
                    selectedRB,
                    V3D(selectedRB->state.getWorldCoordinates(
                            selectedPointInLocalCoords),
                        targetForSelectedPoint) *
                        500.0,
                    selectedPointInLocalCoords);

            physicsEngine->step(dt);

            if (slowMo) break;
        }

        // traj data
        if (selectedScene == SimulationScene::Projectile) {
            if (projectileTraj.size() < 1.0 / dt) {
                P3D p = physicsEngine->rbs[0]->state.pos;
                projectileTraj.push_back(p);
            }
        }
    }

    virtual void drawAuxiliaryInfo() override {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        drawFPS();
        drawConsole();
        drawImGui();

        ImGui::EndFrame();
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }

    // objects drawn with a shadowMapRenderer (during shadow pass) will cast a
    // shadow
    virtual void drawShadowCastingObjects() override {
        physicsEngine->draw(shadowMapRenderer);
    }

    // objects drawn with a shadowShader (during the render pass) will have
    // shadows cast on them
    virtual void drawObjectsWithShadows() override {
        ground.draw(shadowShader, V3D(0.6, 0.6, 0.8));
        physicsEngine->draw(shadowShader);
    }

    // objects drawn with basic shadowShader (during the render pass) will not
    // have shadows cast on them
    virtual void drawObjectsWithoutShadows() override {
        if (selectedScene == SimulationScene::Projectile) {
            // draw analytic trajectory
            double timestep = 0.01;
            for (uint i = 0; i < 100; i++) {
                double t = i * timestep;
                P3D p = projectileP0 + projectileV0 * t +
                        0.5 * V3D(0, RBGlobals::g, 0) * t * t;
                drawSphere(p, 0.01, basicShader);
            }

            // draw real trajectory
            for (uint i = 0; i < projectileTraj.size(); i++) {
                drawSphere(projectileTraj[i], 0.02, basicShader, V3D(0, 1, 0));
            }
        }

        if (selectedRB != NULL) {
            drawArrow3d(selectedRB->state.getWorldCoordinates(
                            selectedPointInLocalCoords),
                        V3D(selectedRB->state.getWorldCoordinates(
                                selectedPointInLocalCoords),
                            targetForSelectedPoint),
                        0.025, basicShader);
        }
    }

    virtual bool keyPressed(int key, int mods) override {
        bool dirty = false;
        if (key == GLFW_KEY_SPACE) {
            appIsRunning = !appIsRunning;
        }
        return false;
    }

    virtual void drawImGui() {
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Once);
        ImGui::Begin("Main Menu");

        ImGui::Text("Simulation Scene:");
        if (ImGui::BeginCombo("##scene",
                              simulationSceneNames[selectedScene].c_str())) {
            for (uint n = 0; n < simulationSceneNames.size(); n++) {
                bool is_selected = (selectedScene == n);
                if (ImGui::Selectable(simulationSceneNames[n].c_str(),
                                      is_selected)) {
                    selectedScene = (SimulationScene)n;
                    setupWorld();
                }
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        ImGui::Text("Play:");
        ImGui::SameLine();
        PlayPauseButton("Play App", &appIsRunning);

        if (appIsRunning == false) {
            ImGui::SameLine();
            if (ImGui::ArrowButton("tmp", ImGuiDir_Right)) {
                appIsRunning = true;
                process();
                appIsRunning = false;
            }
        }

        ImGui::Text("Slow Motion Mode:");
        ImGui::SameLine();
        ToggleButton("SlowMo", &slowMo);

        static int simRate = 30;
        ImGui::Text("Simulation Rate:");
        if (ImGui::SliderInt("Rate", &simRate, 30, 300)) dt = 1.0 / simRate;
        ImGui::InputDouble("dt", &dt, 0.0, 0.0, "%.5f",
                           ImGuiInputTextFlags_ReadOnly);

        ImGui::Text("Simulation Settings:");
        ImGui::Checkbox("Collision", &physicsEngine->simulateCollisions);
        ImGui::SliderFloat("epsilon", &physicsEngine->eps, 0.0, 1.0);
        ImGui::SliderFloat("mu", &physicsEngine->mu, 0.0, 2.0);

        if (ImGui::TreeNode("Draw options...")) {
            ImGui::Checkbox("Draw Coordinate Frames",
                            &physicsEngine->showCoordFrame);
            ImGui::Checkbox("Draw Collision Spheres",
                            &physicsEngine->showCollisionSpheres);
            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Debug options...")) {
            ImGui::Checkbox("Draw debug info", &drawDebugInfo);
            ImGui::TreePop();
        }

        ImGui::End();
    }

private:
    void setupWorld() {
        // reset physics engine first
        resetPhysicsEngine();

        // clean data
        projectileTraj.clear();

        // setup scene now
        switch (selectedScene) {
            case SimulationScene::Projectile: {
                camera.distanceToTarget = 5;
                camera.rotAboutUpAxis = 0;
                camera.rotAboutRightAxis = 0.25;
                auto *rb = physicsEngine->addRigidBodyToEngine();
                rb->state.pos = projectileP0;
                rb->state.orientation = Quaternion(0.1, 0.5, 0, 0).normalized();
                rb->state.velocity = projectileV0;
                rb->state.angularVelocity = V3D(0, 10, 0);
                break;
            }
            case SimulationScene::FixedSprings: {
                camera.distanceToTarget = 6;
                camera.rotAboutUpAxis = 0;
                camera.rotAboutRightAxis = 0.25;
                auto *rb = physicsEngine->addRigidBodyToEngine();
                rb->state.pos = P3D(0, 0.5, 0);
                physicsEngine->addSpringToEngine(
                    nullptr, rb, P3D(0.12, 2, 0.12), P3D(0.12, 0.12, 0.12));
                break;
            }
            case SimulationScene::RigidBodySprings: {
                camera.distanceToTarget = 6;
                camera.rotAboutUpAxis = 0;
                camera.rotAboutRightAxis = 0.25;
                auto *rb1 = physicsEngine->addRigidBodyToEngine();
                rb1->state.pos = P3D(0, 1.5, 0);
                auto *rb2 = physicsEngine->addRigidBodyToEngine();
                rb2->state.pos = P3D(0, 0.5, 0);
                // world~rb1
                physicsEngine->addSpringToEngine(
                    nullptr, rb1, P3D(0.12, 2, 0.12), P3D(0.12, 0.12, 0.12));
                physicsEngine->addSpringToEngine(
                    nullptr, rb1, P3D(0.12, 2, -0.12), P3D(0.12, 0.12, -0.12));
                // rb1~rb2
                physicsEngine->addSpringToEngine(
                    rb1, rb2, P3D(-0.12, -0.12, 0.12), P3D(-0.12, 0.12, 0.12));
                physicsEngine->addSpringToEngine(rb1, rb2,
                                                 P3D(-0.12, -0.12, -0.12),
                                                 P3D(-0.12, 0.12, -0.12));
                break;
            }
            default:
                break;
        }
    }

    void resetPhysicsEngine() {
        if (physicsEngine) delete physicsEngine;
        physicsEngine = new RBPhysicsEngine();
    }

public:
    SimpleGroundModel ground;
    
    // simulation conf
    double dt = 1 / 30.0;

    // simulation engine
    RBPhysicsEngine *physicsEngine = nullptr;
    // simulation scene
    SimulationScene selectedScene = SimulationScene::Projectile;

    // interaction
    crl::RB *selectedRB = NULL;
    P3D selectedPoint;
    P3D selectedPointInLocalCoords;
    P3D targetForSelectedPoint;

    // data
    std::vector<P3D> projectileTraj;

    // some constants
    const P3D projectileP0 = P3D(RBGlobals::g * 0.25, 0.5, 0);
    const V3D projectileV0 = V3D(-RBGlobals::g * 0.5, -RBGlobals::g * 0.5, 0);

    // flags
    bool slowMo = false;
    bool appIsRunning = false;
    bool drawDebugInfo = true;
};

}  // namespace rigidbody
}  // namespace app
}  // namespace crl