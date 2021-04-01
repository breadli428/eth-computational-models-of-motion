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
        if(std::chrono::duration_cast<std::chrono::microseconds>(now-lastFrame).count() >= 10./60. * 1.e6)
        {
            if(keyDown[GLFW_KEY_R])
                boids.initializePositions();
            if(keyDown[GLFW_KEY_SPACE])
                boids.pause();
            if(keyDown[GLFW_KEY_ESCAPE])
                exit(0);
            lastFrame = now;
        }
    }

    void drawImGui() override {

        using namespace ImGui;

       const char* names[] = {"FreeFall", "Separation", "Alignment", "Cohesion", "Leading"};
       Combo("Boids Behavior", (int*)&currentMethod, names, 5);
       End();
    }

    void drawNanoVG() override {
        
        boids.updateBehavior(currentMethod);
        
        TVStack boids_pos = boids.getPositions();

        auto shift_01_to_screen = [](TV pos_01, T scale, T width, T height)
        {
            return TV(0.5 * (0.5 - scale) * width + scale * pos_01[0] * width, 0.5 * (0.5 - scale) * height + scale * pos_01[1] * height);
        };

        for(int i = 0; i < boids.getParticleNumber(); i++)
        {
            TV pos = boids_pos.col(i);
            nvgBeginPath(vg);
    
            // just map position from 01 simulation space to scree space
            // feel free to make changes
            // the only thing that matters is you have pos computed correctly from your simulation
            T scale = 0.3;
            TV screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
            nvgCircle(vg, screen_pos[0], screen_pos[1], 2.f);
            nvgFillColor(vg, COLOR_OUT);
            nvgFill(vg);

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

    Boids<T, dim> boids = Boids<T, dim>(40);
    std::chrono::high_resolution_clock::time_point lastFrame;
};

int main(int, char**)
{
    int width = 720;
    int height = 720;
    TestApp app(width, height, "Assignment 3 Boids");
    app.run();

    return 0;
}
