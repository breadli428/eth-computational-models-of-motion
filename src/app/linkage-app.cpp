#ifdef WIN32
#define NOMINMAX
#define _USE_MATH_DEFINES
#include <cmath>
#endif

#include <application.h>
#include <imgui.h>
#include <imgui_multiplot.h>
#include "ContourPlot.h"

#include <ObjectiveFunction.h>
#include <RandomMinimizer.h>
#include <GradientDescentMinimizer.h>
#include <NewtonFunctionMinimizer.h>
#include <InverseKinematics.h>

#include <iostream>
#include <math.h>
#include <deque>
#include <chrono>
#include <algorithm>
#include <math.h>

using Eigen::Vector2f;
using Eigen::Vector2d;
using Eigen::VectorXd;

#include <Linkage.h>

class LinkageApp : public Application
{
public:
    LinkageApp(int w, int h, const char * title)
        : Application(title, w, h) {

        ImGui::StyleColorsClassic();

        clearColor[0] = clearColor[1] = clearColor[2] = 0.15f;

        lastFrame = std::chrono::high_resolution_clock::now();

        const char* name = IMGUI_FONT_FOLDER"/Cousine-Regular.ttf";
        nvgCreateFont(vg, "sans", name);

        // create solvers
        methods[RANDOM] = {&random, "Random Minimizer", nvgRGBA(100, 255, 255, 150)};
        methods[GD_FIXED] = {&gdFixed, "Gradient Descent Fixed Step Size", nvgRGBA(255, 100, 100, 150)};
        methods[GD_LS] = {&gdLineSearch, "Gradient Descent w/ Line Search", nvgRGBA(255, 255, 100, 150)};
        methods[NEWTON] = {&newton, "Newton's method",nvgRGBA(255, 100, 255, 150)};
        resetMethods();

//        // set up `functionValues`, used for plots
//        functionValues = new float*[minimizers.size()];
//        for (int i = 0; i < minimizers.size(); ++i) {
//            functionValues[i] = new float[plotN];
//            for (int j = 0; j < plotN; ++j) {
//                functionValues[i][j] = 0.f;
//            }
//        }

        // create iso contours
        fixedImgRes = getImgSize();
        fixedImgRes.y() += 30;
        plot.from = {-M_PI, -M_PI};
        plot.to = {M_PI, M_PI};
        generatePlot(true);
    }

    void process() override {

        Vector2i imgSize = getImgSize();

        // move image if right mouse button is pressed
        if(mouseState.rButtonPressed && mouseState.lastMouseX < imgSize.x()){
            Vector2i d = {
                (mouseState.lastMouseX - cursorPosDown[0]),
                (mouseState.lastMouseY - cursorPosDown[1])
            };
            cursorPosDown[0] = mouseState.lastMouseX;
            cursorPosDown[1] = mouseState.lastMouseY;

            Vector2i res = getImgRes();
            d.x() = (float)res.x()/(float)imgSize.x() * (float)d.x();
            d.y() = (float)res.y()/(float)imgSize.y() * (float)d.y();

            InverseKinematics ik(linkage, target);
            plot.translate(&ik, res, d.x(), d.y());
            if(img != -1)
                nvgDeleteImage(vg, img);
            img = nvgCreateImageRGBA(vg, res.x(), res.y(), 0, plot.imgData);

        }

        bool settingAlpha = false;
        if(mouseState.lButtonPressed){

            if(mouseState.lastMouseX < imgSize.x()){
                settingAlpha = true;
                auto ijImg = fromScreenToImg({mouseState.lastMouseX, mouseState.lastMouseY});
                angles = plot.fromImgtoPlot(ijImg);
                anglesHistory.clear();
                anglesHistory.push_back(angles);
            }
            else {
                float w = 20;
                Vector2d x0 = {.5f*width/pixelRatio + w, .5f*height/pixelRatio};
                target = (Vector2d(mouseState.lastMouseX, mouseState.lastMouseY) - x0)/linkageScale;
                generatePlot(true);
            }
        }

        // run at 60fps, or in slow mo
        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::milliseconds>(now-lastFrame).count() > ((slowMo) ? 320 : 16)){

            // do IK
            if(ikOn && !settingAlpha){
                VectorXd x = angles;
                angles = inverseKinematics(linkage, target, angles, methods[currentMethod].method);
                //                for(auto &a : angles)
                //                    if(a > M_PI) a -= 2*M_PI;
                //                    else if(a < -M_PI) a+= 2*M_PI;

                anglesHistory.push_back(angles);
                while(anglesHistory.size() > 30)
                    anglesHistory.pop_front();
            }

            lastFrame = now;
        }
    }

    void drawImGui() override {

        using namespace ImGui;

        BeginMainMenuBar();
        menubarHeight = GetWindowSize().y;
        if(BeginMenu("View")){

            Text("Contour Plot");

            if(Checkbox("fixed contour plot resolution?", &fixImgSize))
                generatePlot(true);
            if(fixImgSize){
                if(InputInt2("resultion", &fixedImgRes[0]))
                    generatePlot(true);
            }

            if(InputInt("# of contours", &plot.nContours)){
                generatePlot();
            }

            if(Button("Recompute contours")){
                generatePlot(true);
            }

            if(Checkbox("show contour edges", &plot.showContourEdges)){
                generatePlot();
            }
            ImGui::EndMenu();
        }
        if(BeginMenu("Method Settings")){
            const double smin = 0.0, smax = 1.0;
            SliderScalar("GD Fixed: step size", ImGuiDataType_Double, &gdFixed.stepSize, &smin, &smax);
            InputDouble("Newton: regularizer", &newton.reg);
            ImGui::EndMenu();
        }
        EndMainMenuBar();

        Begin("IK");

        TextColored(ImVec4(1.f, 1.f, 1.f, 0.5f), "left mouse:  start optimization");
        TextColored(ImVec4(1.f, 1.f, 1.f, 0.5f), "right mouse: move function landscape");
        TextColored(ImVec4(1.f, 1.f, 1.f, 0.5f), "mouse wheel: zoom function landscape");
        TextColored(ImVec4(1.f, 1.f, 1.f, 0.5f), "space bar:   play/pause optimization");

        Separator();

        Text("Linkage:");

        float angle0 = angles[0];
        if(SliderAngle("angle 0", &angle0))
            angles[0] = angle0;

        float angle1 = angles[1];
        if(SliderAngle("angle 1", &angle1))
            angles[1] = angle1;

        const double min = 0., max = 4;
        if(SliderScalarN("target", ImGuiDataType_Double, target.data(), 2, &min, &max))
            generatePlot(true);
        InputFloat("link scale", &linkageScale);

        Text("Inverse Kinematics:");

        Checkbox("play [space]", &ikOn);
        Checkbox("Slow motion", &slowMo);

        const char* names[] = {"Random", "GD fixed step size", "GD w/ Line Search", "Newton"};
        Combo("active minimizer", (int*)&currentMethod, names, 4);

        End();
    }

    void drawNanoVG() override {

        // draw contour plot
        {

            nvgReset(vg);

            // draw contour lines
            nvgBeginPath(vg);
            auto res = getImgRes();
            auto size = getImgSize();
            NVGpaint imgPaint = nvgImagePattern(vg, 0.f, 0.f, size.x(), size.y(), 0.f, img, .8f);
            nvgRect(vg, 0.f, 0.f, size.x(), size.y());
            nvgFillPaint(vg, imgPaint);
            nvgFill(vg);

            nvgReset(vg);
            int i=0;
            for(const auto &angle : anglesHistory){
                float x = ((float)i+1)/(float)anglesHistory.size();

                nvgBeginPath(vg);
                auto a = fromImgToScreen( plot.fromPlotToImg(angle) );
                nvgCircle(vg, a[0], a[1], 5);
                nvgStrokeColor(vg, nvgRGBAf(.6f, .5f, 1.f, .8f*x));
                nvgStrokeWidth(vg, 2.f);
                nvgStroke(vg);
                nvgFillColor(vg, nvgRGBAf(.6f, .5f, 1.f, .4f*x));
                nvgFill(vg);

                if(i < anglesHistory.size()-1){
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, a[0], a[1]);
                    auto b = fromImgToScreen( plot.fromPlotToImg(anglesHistory[i+1]) );
                    nvgLineTo(vg, b[0], b[1]);
                    nvgStrokeColor(vg, nvgRGBAf(.6f, .5f, 1.f, .8f*x));
                    nvgStrokeWidth(vg, 10.f);
                    nvgStroke(vg);
                }

                i++;
            }

            nvgBeginPath(vg);
            nvgFontFace(vg, "sans");
            nvgFontSize(vg, 12.f);
            nvgFillColor(vg, nvgRGBAf(0, 0, 0, 1));
            char buf[100];
            sprintf(buf, "[%g,%g]", plot.from[0], plot.from[1]);
            nvgTextAlign(vg, NVG_ALIGN_LEFT | NVG_ALIGN_TOP);
            nvgText(vg, 0, 25, buf, nullptr);
            nvgFill(vg);
            sprintf(buf, "[%g,%g]", plot.to[0], plot.to[1]);
            nvgTextAlign(vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
            nvgText(vg, size.x(), size.y(), buf, nullptr);
            nvgFill(vg);
        }

        // draw linkage
        {
            std::array<Vector2d, 3> points = forwardKinematics(linkage, angles);
            Vector2f pts[3];
            float w = 20;
            Vector2f x0 = {.5f*width/pixelRatio + w, .5f*height/pixelRatio};
            int i=0;
            for (auto p : points)
                pts[i++] = x0 + Vector2f(p.x(), p.y())*linkageScale;

            double angle = 0;
            for (int i=0; i<2; i++) {
                angle += angles[i];
                nvgReset(vg);
                nvgBeginPath(vg);
                nvgTranslate(vg, pts[i].x(), pts[i].y());
                nvgRotate(vg, angle);
                nvgRoundedRect(vg, -w/2, -w/2, linkage.length[i]*linkageScale + w, w, w/3);
                nvgStrokeColor(vg, nvgRGBAf(1.f, 1.f, 1.f, 0.8f));
                nvgStrokeWidth(vg, 2.f);
                nvgStroke(vg);
                nvgFillColor(vg, nvgRGBAf(.6f, .6f, .6f, .4f));
                nvgFill(vg);
            }

            // points
            nvgReset(vg);
            int pt_i = 0;
            for (auto p : pts){
                nvgBeginPath(vg);
                nvgCircle(vg, p.x(), p.y(), 5.f);
                nvgStrokeColor(vg, nvgRGBAf(.6f, .6f, 1.f, 1.f));
                nvgStrokeWidth(vg, 2.f);
                nvgStroke(vg);
                nvgFillColor(vg, nvgRGBAf(.6f, .6f, 1.f, .8f));
                nvgFill(vg);

                nvgBeginPath(vg);
                nvgFontFace(vg, "sans");
                nvgFontSize(vg, 14.f);
                nvgFillColor(vg, nvgRGBAf(.6f, .6f, 1.f, .5f));
                char buf[100];
                sprintf(buf, "p%d", pt_i);
                nvgTextAlign(vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
                nvgText(vg, p.x() + 30, p.y(), buf, nullptr);
                nvgFill(vg);
                pt_i++;
            }

            nvgReset(vg);
            nvgBeginPath(vg);
            nvgCircle(vg, x0.x() + target.x()*linkageScale, x0.y() + target.y()*linkageScale, 5.f);
            nvgStrokeColor(vg, nvgRGBf(0, .8f, 0));
            nvgStrokeWidth(vg, 2.f);
            nvgStroke(vg);
            nvgFillColor(vg, nvgRGBf(0, .8f, 0));
            nvgFill(vg);

        }

    }

protected:
    void keyPressed(int key, int mods) override {
        // play / pause with space bar
        if(key == GLFW_KEY_SPACE)
            ikOn = !ikOn;
    }

    void mouseButtonPressed(int button, int mods) override {
        cursorPosDown[0] = mouseState.lastMouseX;
        cursorPosDown[1] = mouseState.lastMouseY;
    }

    void scrollWheel(double xoffset, double yoffset) override {
        double zoom = std::pow(1.1, yoffset);
        plot.to = plot.from + zoom*(plot.to - plot.from);

        generatePlot();
    }

    void resizeWindow(int w, int h) override {
        Application::resizeWindow(w, h);
//        if(!fixedContourPlotResolution)
        generatePlot(true);
    }


private:

    void resetMethods(const VectorXd &x = VectorXd()){
        // reset random minimizer
        random.searchDomainMax = plot.fromImgtoPlot({0,0});
        random.searchDomainMin = plot.fromImgtoPlot({width/pixelRatio,height/pixelRatio});
        InverseKinematics ik(linkage, target);
        random.fBest = (x.size() == 0) ? HUGE_VAL : ik.evaluate(x);
        random.xBest = x;
    }

    Vector2i getImgRes() const {
        return (fixImgSize)
                ? fixedImgRes
                : getImgSize();
    }

    Vector2i getImgSize() const {
        return {width/2 / pixelRatio, height / pixelRatio};
    }

    Vector2i fromScreenToImg(const Vector2i &ijScreen){
        Vector2i ijImg;
        auto res = getImgRes();
        auto size = getImgSize();
        ijImg[0] = (float)res[0] / (float)size[0] * (float)ijScreen[0];
        ijImg[1] = (float)res[1] / (float)size[1] * (float)ijScreen[1];
        return ijImg;
    }
    Vector2i fromImgToScreen(const Vector2f &ijImg){
        Vector2i ijScreen;
        auto res = getImgRes();
        auto size = getImgSize();
        ijScreen[0] = (float)size[0] / (float)res[0] * ijImg[0];
        ijScreen[1] = (float)size[1] / (float)res[1] * ijImg[1];
        return ijScreen;
    }
    Vector2i fromImgToScreen(const Vector2i &ijImg){
        Vector2f ijImgf = {ijImg[0], ijImg[1]};
        return fromImgToScreen(ijImgf);
    }

    void generatePlot(bool recomputeContours = false) {
        auto imgSize = getImgRes();
        InverseKinematics ik = {linkage, target};
        plot.generate(&ik, imgSize, recomputeContours);
        if(img != -1)
            nvgDeleteImage(vg, img);
        img = nvgCreateImageRGBA(vg, imgSize.x(), imgSize.y(), 0, plot.imgData);
    }

private:
    // linkage and angles
    Linkage linkage;
    Vector2d angles = {M_PI*0.5,0};
    Vector2d target = {0.1, 0.2};

    // for visualization
    std::deque<Vector2d> anglesHistory;
    float linkageScale = 200.f;
    bool fixImgSize = true;
    Vector2i fixedImgRes;
    float menubarHeight = 20;

    // optimization & IK
    RandomMinimizer random;
    GradientDescentFixedStep gdFixed;
    GradientDescentLineSearch gdLineSearch;
    NewtonFunctionMinimizer newton;
    bool ikOn = false;
    bool slowMo = false;
    enum MethodTypes {
        RANDOM=0, GD_FIXED=1, GD_LS=2, NEWTON=3
    };
    MethodTypes currentMethod = GD_FIXED;
    struct MethodState
    {
        MinimizationMethod *method;
        std::string name;
        NVGcolor color;
        std::deque<Vector2d> path = {};
        bool show = true;
    };
    std::map<MethodTypes, MethodState> methods;

    int img;
    ContourPlot plot;

    // user interface
    double cursorPosDown[2];
    std::chrono::high_resolution_clock::time_point lastFrame;
};

int main(int, char**)
{
    LinkageApp app(1080, 720, "CMM'21 - Linkage App");
    app.run();

    return 0;
}
