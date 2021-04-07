#include "application.h"
#include <imgui.h>

#include <iostream>
#include <math.h>
#include <deque>
#include <chrono>

#include "../boids/boids.h"

#define T float
#define dim 2

class TestApp : public Application
{
#define COLOR_OUT    nvgRGBA(220,50,50,255)
#define COLOR_IN     nvgRGBA(50,50,220,255)
#define COLOR_SOLVED nvgRGBA(50,220,50,255)

typedef Matrix<T, Eigen::Dynamic, 1> VectorXT;
typedef Matrix<T, dim, Eigen::Dynamic> TVStack;
typedef Vector<T, dim> TV;

public:

    TestApp(int w, int h, const char * title) : Application(title, w, h) {

        ImGui::StyleColorsClassic();

        const char* name = IMGUI_FONT_FOLDER"/Cousine-Regular.ttf";
        nvgCreateFont(vg, "sans", name);
        
    }

    void process() override {
        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::microseconds>(now-lastFrame).count() >= 10./60. * 1.e6)
        {
            if (keyDown[GLFW_KEY_R])
            {
                boids = Boids<T, dim>(40);
                boids.initializePositions(initialVel);
            }
            if (keyDown[GLFW_KEY_SPACE])
                boids.pause();
            if (keyDown[GLFW_KEY_ESCAPE])
                exit(0);
            lastFrame = now;
        }
    }

    void drawImGui() override {

        using namespace ImGui;

        const char* init_vel[] = {"Zero", "Orthogonal", "Random", "Random Positive"};
        const char* names[] = {"Free Fall", "Separation", "Alignment", "Cohesion", "Leading", "Circular", "Collision Avoidance", "Collaboration and Adversary"};
        const char* update_names[] = {"Explicit Euler", "Symplectic Euler", "Explicit Midpoint"};
        Begin("Menu");
        Combo("Initial Velocities", (int*)&initialVel, init_vel, 4);
        if (initialVel != initialVel_temp)
        {
            boids.initializePositions(initialVel);
            initialVel_temp = initialVel;
        }

        float hmin = 0.001f;
        float hmax = 0.05f;
        SliderScalar("Step Size", ImGuiDataType_Float, &h_read, &hmin, &hmax);
        float fsmin = 0.5f;
        float fsmax = 5.0f;
        SliderScalar("Force Scaler", ImGuiDataType_Float, &f_scaler_read, &fsmin, &fsmax);

        Combo("Boids Behavior", (int*)&currentMethod, names, 8);
        if (currentMethod == SEPARATION || currentMethod == ALIGNMENT || currentMethod == COHESION)
        {
            float rmin = 0.01f;
            float rmax = 2.0f;
            SliderScalar("Range", ImGuiDataType_Float, &range_read, &rmin, &rmax);
            drawCircles = false;
        }

        if (currentMethod == LEADING)
        {
            cursorDetect = true;
            float rmin = 0.01f;
            float rmax = 2.0f;
            SliderScalar("Range", ImGuiDataType_Float, &range_read, &rmin, &rmax); SameLine();
            Checkbox("Obstacle", &drawCircles);
            if(drawCircles)
            {
                float omin = 0.0f;
                float omax = 1.0f;
                SliderScalarN("Obstacle Position and Radius", ImGuiDataType_Float, &obstacle_read, 3, &omin, &omax);
            }
        }
        else cursorDetect = false;

        if (currentMethod == COLLISION_AVOIDANCE)
        {
            float omin = 0.0f;
            float omax = 1.0f;
            SliderScalarN("Obstacle Position and Radius", ImGuiDataType_Float, &obstacle_read, 3, &omin, &omax);
            drawCircles = true;
        }

        if (currentMethod == COLLABORATION_ADVERSARY)
        {
            float rmin = 0.01f;
            float rmax = 0.2f;
            SliderScalar("Range", ImGuiDataType_Float, &range_read, &rmin, &rmax);
            divideGroup = true;
            drawCircles = false;
            const char* control_group_list[] = {"Red Only", "Blue Only", "Both"};
            Combo("Control Group", (int*)&control_group_read, control_group_list, 3);
        }
        else divideGroup = false;


        Combo("Update Method", (int*)&updateRule, update_names, 3);
        End();
    }

    void drawNanoVG() override {
        
        boids.read_parameters(h_read, f_scaler_read, range_read, obstacle_read, drawCircles, control_group_read);
        boids.updateBehavior(currentMethod, updateRule);
        
        TVStack boids_pos = boids.getPositions();

        auto shift_01_to_screen = [](TV pos_01, T scale, T width, T height)
        {
            return TV(0.5 * (0.8 - scale) * width + scale * pos_01[0] * width, 0.5 * (0.8 - scale) * height + scale * pos_01[1] * height);
        };

        auto shift_screen_to_01 = [](TV screen_pos, T scale, T width, T height)
        {
            return TV((screen_pos[0] - 0.5 * (0.8 - scale) * width) / (scale * width), (screen_pos[1] - 0.5 * (0.8 - scale) * height) / (scale * height));
        };

        if (drawCircles)
        {
            T scale = 0.3f;
            circle_obstacle_start = shift_01_to_screen(obstacle_read.pos, scale, width, height);
            circle_obstacle = {circle_obstacle_start, scale * obstacle_read.radius * width, COLOR_SOLVED, nvgRGBA(10, 10, 10, 255)};
            auto drawCircle = [this](const Circle &circle)
            {
                nvgBeginPath(vg);
                nvgCircle(vg, circle.pos[0], circle.pos[1], circle.radius);
                nvgFillColor(vg, circle.colorFill);
                nvgFill(vg);
                nvgStrokeColor(vg, circle.colorStroke);
                nvgStrokeWidth(vg, 3.0f);
                nvgStroke(vg);
            };
            drawCircle(circle_obstacle);
        }

        if (divideGroup)
        {
            for (int i = 0; i < boids_pos.cols(); i++)
            {
                Eigen::MatrixXi group_label = boids.getGroupLabel();
                TV pos = boids_pos.col(i);
                nvgBeginPath(vg);
                T scale = 0.3f;
                TV screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
                nvgCircle(vg, screen_pos[0], screen_pos[1], 2.f);
                if (group_label(0, i) == 0)
                {
                    nvgFillColor(vg, COLOR_OUT);
                }
                else nvgFillColor(vg, COLOR_IN);
                nvgFill(vg);
            }
        }
        else
        {
            for (int i = 0; i < boids.getParticleNumber(); i++)
            {
                TV pos = boids_pos.col(i);
                nvgBeginPath(vg);
        
                // just map position from 01 simulation space to scree space
                // feel free to make changes
                // the only thing that matters is you have pos computed correctly from your simulation
                T scale = 0.3f;
                TV screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
                nvgCircle(vg, screen_pos[0], screen_pos[1], 2.f);
                if (i == 0 && cursorDetect)
                {
                    nvgFillColor(vg, COLOR_IN);
                }
                else nvgFillColor(vg, COLOR_OUT);
                nvgFill(vg);
            }
        }


        if (cursorDetect)
        {
            if (mouseState.lButtonPressed && mouseState.lastMouseX < width)
            {
            cursorPosDown = TV(mouseState.lastMouseX, mouseState.lastMouseY);
            }
            T scale = 0.3f;
            TV target_pos_read = shift_screen_to_01(cursorPosDown, scale, width, height);
            boids.read_cursor(target_pos_read);
        }
    }

protected:
    void mouseButtonPressed(int button, int mods) override {

    }

    void mouseButtonReleased(int button, int mods) override {
        
    }

private:
    int loadFonts(NVGcontext* vg)
    {
        int font;
        font = nvgCreateFont(vg, "sans", "../example/Roboto-Regular.ttf");
        if (font == -1) {
            printf("Could not add font regular.\n");
            return -1;
        }
        font = nvgCreateFont(vg, "sans-bold", "../example/Roboto-Bold.ttf");
        if (font == -1) {
            printf("Could not add font bold.\n");
            return -1;
        }
        return 0;
    }

private:

    MethodTypes currentMethod = FREEFALL;
    UpdateTypes updateRule = EXPLICIT_EULER;
    InitTypes initialVel = ZERO;
    InitTypes initialVel_temp = ZERO;
    Boids<T, dim> boids = Boids<T, dim>(40);
    std::chrono::high_resolution_clock::time_point lastFrame;
    float h_read = 0.02f;
    float f_scaler_read = 2.0f;
    float range_read = 0.5f;

    bool drawCircles = false;
    TV circle_obstacle_start;
    struct Circle
    {
        TV pos;
        float radius;
        NVGcolor colorFill, colorStroke;
    } circle_obstacle;

    Boids<T, dim>::Obstacle obstacle_read = {TV(0.5f, 0.5f), 0.2f};

    bool cursorDetect = false;
    TV cursorPosDown = TV(360.0f, 360.0f);

    bool divideGroup = false;
    ControlGroup control_group_read = BOTH;
};

int main(int, char**)
{
    int width = 720;
    int height = 720;
    TestApp app(width, height, "Assignment 3 Boids");
    app.run();

    return 0;
}
