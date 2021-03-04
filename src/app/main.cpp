#include "application.h"
#include <imgui.h>

#include <iostream>
#include <math.h>
#include <deque>
#include <chrono>

#include <add.h>
using namespace math;

class TestApp : public Application
{
#define COLOR_OUT    nvgRGBA(220,50,50,255)
#define COLOR_IN     nvgRGBA(50,50,220,255)
#define COLOR_SOLVED nvgRGBA(50,220,50,255)

public:
    TestApp(int w, int h, const char * title) : Application(title, w, h) {

        ImGui::StyleColorsClassic();

        const char* name = IMGUI_FONT_FOLDER"/Cousine-Regular.ttf";
        nvgCreateFont(vg, "sans", name);

        lastFrame = std::chrono::high_resolution_clock::now();

        Vector2f center = {w/2, 130 + (h-130)/2};
        circleKeyStart = center + Vector2f{-200, -100};
        circleMouseStart = center + Vector2f{-200,  100};
        circleKey   = {circleKeyStart,   30, COLOR_OUT, nvgRGBA(10, 10, 10, 255)};
        circleMouse = {circleMouseStart, 20, COLOR_OUT, nvgRGBA(10, 10, 10, 255)};
        rect = Box{center + Vector2f{200, 0}, Vector2f(150, 200), nvgRGBA(150, 150, 150, 250), nvgRGBA(10, 10, 10, 200)};

    }

    void process() override {
        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
#ifdef SINGLE_BUFFER
        if(std::chrono::duration_cast<std::chrono::microseconds>(now-lastFrame).count() >= 1./60. * 1.e6)
#endif
        {
            Vector2f vel(0, 0);
            if(keyDown[GLFW_KEY_LEFT])
                vel[0] -= 1;
            if(keyDown[GLFW_KEY_RIGHT])
                vel[0] += 1;
            if(keyDown[GLFW_KEY_UP])
                vel[1] -= 1;
            if(keyDown[GLFW_KEY_DOWN])
                vel[1] += 1;
            circleKey.pos = math::add(circleKey.pos, vel/(vel.norm()+1e-10) * 10);

            bool isInsideKey = rect.isInside(circleKey.pos, circleKey.radius);
            bool isInsideMouse = rect.isInside(circleMouse.pos, circleMouse.radius);
            circleKey.colorFill = (isInsideKey) ? (isInsideMouse ? COLOR_SOLVED : COLOR_IN) : COLOR_OUT;
            circleMouse.colorFill = (isInsideMouse) ? (isInsideKey ? COLOR_SOLVED : COLOR_IN) : COLOR_OUT;

            if(draggingCircle)
                circleMouse.pos = math::add(Vector2f(mouseState.lastMouseX, mouseState.lastMouseY), - draggingCircleOffset);

            if(isInsideKey && isInsideMouse){
                auto make_circle = [&](Vector2f origin)
                {
                    float randX = (5.f*((float) rand() / (RAND_MAX)) + .1f) * ((rand()%2) ? -1 : 1);
                    float randY = (5.f*((float) rand() / (RAND_MAX)) + .1f) * ((rand()%2) ? -1 : 1);
                    float randR = 30.f*((float) rand() / (RAND_MAX)) + 10.f;
                    unsigned char r = rand() % 255;
                    unsigned char g = rand() % 255;
                    unsigned char b = rand() % 255;
                    unsigned char a = rand() % 255;
                    circles.push_back({Vector2f(randX, randY), Circle{origin, randR, nvgRGBA(r, g, b, a), nvgRGBA(r, g, b, (2*a)%255)}});
                };

                make_circle(circleKey.pos);
                make_circle(circleMouse.pos);

                if(circles.size() > 500)
                    circles.erase(circles.begin(), circles.begin()+10);

                for(auto &c : circles)
                    c.second.pos += c.first;
            }
            else{
                circles.clear();
            }
            lastFrame = now;
        }
    }

    void drawImGui() override {

        using namespace ImGui;

        SetNextWindowSize({width - 40.f, 200.f}, ImGuiCond_FirstUseEver);
        SetNextWindowPos({20.f, 60.f}, ImGuiCond_FirstUseEver);
        Begin("Hello World!");
        TextWrapped("Use the arrow keys to move the first circle.");
        TextWrapped("Drag the other circle with the mouse.");
        TextWrapped("Put them both in the Box!");
        End();

        BeginMainMenuBar();
        if(BeginMenu("debug")){
            Checkbox("draw cursor", &drawCursor);
            Checkbox("draw circles", &drawCircles);
            ImGui::EndMenu();
        }
        Text("| window size: %d x %d", width, height);
        Text("| pixel ratio: %f", pixelRatio);
        EndMainMenuBar();
    }

    void drawNanoVG() override {

        // draw Box
        nvgBeginPath(vg);
        nvgRoundedRect(vg, rect.center[0]-rect.size[0]/2.f, rect.center[1]-rect.size[1]/2.f, rect.size[0], rect.size[1], 10.f);
        nvgFillColor(vg, rect.colorFill);
        nvgFill(vg);
        nvgStrokeColor(vg, rect.colorStroke);
        nvgStrokeWidth(vg, 4.0f);
        nvgStroke(vg);

        if(drawCircles)
        {
            auto drawCircle = [this](const Circle &circle){
                nvgBeginPath(vg);
                nvgCircle(vg, circle.pos[0], circle.pos[1], circle.radius);
                nvgFillColor(vg, circle.colorFill);
                nvgFill(vg);
                nvgStrokeColor(vg, circle.colorStroke);
                nvgStrokeWidth(vg, 4.0f);
                nvgStroke(vg);
            };

            for(const auto &c : circles)
                drawCircle(c.second);

            // draw circle key
            drawCircle(circleKey);
            {
                nvgBeginPath(vg);
                nvgFontFace(vg, "sans");
                nvgFontSize(vg, 16.f);
                float d = (circleKey.pos - circleKeyStart).squaredNorm();
                nvgFillColor(vg, nvgRGBAf(0, 0, 0, std::max(0.f, 1.f-0.0001f*d)));
                nvgFontBlur(vg, std::min(1000.f, d));
                nvgTextAlign(vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
                nvgText(vg, circleKey.pos[0] + circleKey.radius + 5, circleKey.pos[1], "move me with arrow keys", nullptr);
                nvgFill(vg);
            }

            drawCircle(circleMouse);
            {
                nvgBeginPath(vg);
                nvgFontFace(vg, "sans");
                nvgFontSize(vg, 16.f);
                float d = (circleMouse.pos - circleMouseStart).squaredNorm();
                nvgFillColor(vg, nvgRGBAf(0, 0, 0, std::max(0.f, 1.f-0.0001f*d)));
                nvgFontBlur(vg, std::min(1000.f, 0.3f*d));
                nvgTextAlign(vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
                nvgText(vg, circleMouse.pos[0] + circleMouse.radius + 5, circleMouse.pos[1], "drag me with the cursor", nullptr);
                nvgFill(vg);
            }
        }

        if(drawCursor){
            nvgBeginPath(vg);
            nvgFillColor(vg, rect.colorStroke);
            nvgCircle(vg, mouseState.lastMouseX, mouseState.lastMouseY, 10.f);
            nvgFill(vg);
        }
    }

protected:
    void mouseButtonPressed(int button, int mods) override {
        Vector2f x = Vector2f(mouseState.lastMouseX, mouseState.lastMouseY);
        if(button == GLFW_MOUSE_BUTTON_LEFT && circleMouse.isInside(x)) {
            draggingCircle = true;
            draggingCircleOffset = x - circleMouse.pos;
        }
    }

    void mouseButtonReleased(int button, int mods) override {
        draggingCircle = false;
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
    bool drawCursor = false;
    bool drawCircles = true;
    Vector2f circleKeyStart, circleMouseStart;
    struct Circle
    {
        Vector2f pos;
        float radius;
        NVGcolor colorFill, colorStroke;

        bool isInside(const Vector2f &x){
            return (x-pos).squaredNorm() <= radius*radius;
        }
    } circleKey, circleMouse;
    struct Box {
        bool isInside(const Vector2f &x, float slack = 0.f) {
            for (int i = 0; i < 2; ++i)
                if(x[i]-slack < center[i]-size[i]/2 || x[i]+slack > center[i]+size[i]/2)
                    return false;
            return true;
        }

        Vector2f center;
        Vector2f size;
        NVGcolor colorFill, colorStroke;
    } rect;

    bool draggingCircle = false;
    Vector2f draggingCircleOffset;
    std::deque<std::pair<Vector2f, Circle>> circles;
    std::chrono::high_resolution_clock::time_point lastFrame;
};

int main(int, char**)
{
    TestApp app(720, 560, "Assignment 0");
    app.run();

    return 0;
}
