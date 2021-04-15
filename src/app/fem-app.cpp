#ifdef WIN32
#define NOMINMAX
#endif
#include <application.h>
#include <imgui.h>
#include <imgui_multiplot.h>

#include <Simulation.h>
#include <iostream>
#include <chrono>
#include <Eigen/Core>
#include <BernHelpers.h>

bool I_AM_HAVING_SCREEN_ISSUES = false; 
bool DRAW_MOUSE_POSITION = false; 
bool THE_SIM_RUNS_TOO_SLOWLY = false; 
double globalTime = 0.; 

class FEMApp : public Application { 

public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public: 
	Simulation sim = Simulation(1, THE_SIM_RUNS_TOO_SLOWLY);
    bool SIMULATE = false;
    bool SLOMO = false; 
public:
	const vector<Vector2d> T_handles0 = { Vector2d(0., .5), Vector2d(1., .5) };
	const vector<double> theta_handles0 = { PI, 0 };
	double eps_handle = .1;
	vector<Vector2d> T_handles = T_handles0;
	vector<double>   theta_handles = theta_handles0;
	vector<Vector2d> get_handleTranslations() {
		vector<Vector2d> ret;
		for (int i = 0; i < T_handles.size(); ++i) { ret.push_back(T_handles[i] - T_handles0[i]); }
		return ret;
	}
	vector<double> get_handleRotations() {
		vector<double> ret;
		for (int i = 0; i < theta_handles.size(); ++i) { ret.push_back(theta_handles[i] - theta_handles0[i]); }
		return ret;
	}

public:
    FEMApp(int w, int h, const char *title, float pixelRatio = 2.f) : Application(title, w, h) { } 

public: 
    void process() override { 
		globalTime += .01;
        static std::chrono::high_resolution_clock::time_point lastFrame = std::chrono::high_resolution_clock::now(); 
        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::milliseconds>(now - lastFrame).count() > ((SLOMO) ? 320 : 16)){ 
			if (SIMULATE) {
				auto u = sim.stackControl(get_handleTranslations(), get_handleRotations());
				auto xv = sim.solvePhysics(u);
				sim.storePositionAndVelocity(xv);
			}
		lastFrame = now; }
    } 

public: 
	void drawNanoVG() override { 
		strokeRect(Vector2d( 0.,  0.), Vector2d(1., 1.), GRAY);
		for (const auto &tri : sim.triangles) { drawTriangle(tri, sim.x); }
		for (const auto &pin : sim.pins     ) { drawPin(pin, sim.x); }
		for (int i = 0; i < T_handles.size(); ++i) {
			fillCircle(T_handles[i], RED); 
			drawVector(T_handles[i], eps_handle * e_theta(theta_handles[i]), RED); 
		}
		if (DRAW_MOUSE_POSITION) { auto push = CIRCLE_RADIUS; CIRCLE_RADIUS *= 3; NVGcolor c = GREEN; c.a = .7f; fillCircle(x_mouse, c); CIRCLE_RADIUS = push; } 
    }

    void drawImGui() override { using namespace ImGui; 
        Begin(""); 
        Checkbox("SIMULATE // Keyboard Shortcut: [Space]", &SIMULATE);
		Checkbox("sim.CHECK_dEdx", &sim.CHECK_dEdx);
		Checkbox("sim.CHECK_d2Edx2", &sim.CHECK_d2Edx2);
        Checkbox("sim.APPLY_GRAVITY  // Keyboard Shortcut: g", &sim.APPLY_GRAVITY);
        Checkbox("sim.SOLVE_DYNAMICS // Keyboard Shortcut: d", &sim.SOLVE_DYNAMICS);
		SliderScalar("ZOOM", ImGuiDataType_Double, &ZOOM_, &ZOOM_MIN_, &ZOOM_MAX_); 
        Checkbox("SLOMO // Keyboard Shortcut: s",    &SLOMO   ); 
        End(); 
    } 

protected: 
	int DRAG_T_i = -1;
	int DRAG_theta_i = -1;
	Vector2d x_mouse = { 0., 0. };
	void mouseButtonPressed(int, int) override { 
		DRAG_T_i = -1;
		DRAG_theta_i = -1;
		const double thresh = 0.05;

		{
			double minDist = INFINITY;
			int minIndex = -1;
			for (int i = 0; i < T_handles.size(); ++i) {
				double dist = (x_mouse - T_handles[i]).norm();
				if (dist < minDist) {
					minDist = dist;
					minIndex = i;
				}
			}
			if (minDist < thresh) { DRAG_T_i = minIndex; return; }
		} 

		{
			double minDist = INFINITY;
			int minIndex = -1;
			for (int i = 0; i < T_handles.size(); ++i) {
				double dist = (x_mouse - (T_handles[i] + eps_handle*e_theta(theta_handles[i]))).norm();
				if (dist < minDist) {
					minDist = dist;
					minIndex = i;
				}
			}
			if (minDist < thresh) { DRAG_theta_i = minIndex; }
		} 
	}
	void mouseButtonReleased(int, int) override { DRAG_T_i = DRAG_theta_i = -1; }
	void mouseMove(double, double) override {
	    double f = (I_AM_HAVING_SCREEN_ISSUES) ? .5 : 1.; x_mouse = _2xy(Vector2d(f*mouseState.lastMouseX, f*mouseState.lastMouseY));
		// --
		if (DRAG_T_i != -1) { T_handles[DRAG_T_i] = x_mouse; }
		if (DRAG_theta_i != -1) { theta_handles[DRAG_theta_i] = atan2_wrapper(from_(T_handles[DRAG_theta_i], x_mouse)); }
	}

    void keyPressed(int key, int mods) override {
		if (key == GLFW_KEY_SPACE) { toggle(SIMULATE); }
		if (key == GLFW_KEY_D    ) { toggle(sim.SOLVE_DYNAMICS); }
		if (key == GLFW_KEY_G    ) { toggle(sim.APPLY_GRAVITY); }
		if (key == GLFW_KEY_S    ) { toggle(SLOMO); }
    } 
	void keyRepeated(int key, int mods) override { }

public: 
	double eps = .25; // * otherwise stuff doesn't show up
	double get_L_() { return double(std::min(height, width)); }
	double get_f_() { return get_L_() / (1. + 2 * eps); }
	double ZOOM_ = 1.f;
	double ZOOM_MIN_ = .1f;
	double ZOOM_MAX_ = 2.f;
	double ZOOM() { return 1. / ZOOM_; }
	Vector2d get_o_() { return Vector2d(0.5, 1.0); }
	Vector2f _2nvg(Vector2d xy) {
		// ZOOM->(-eps, 1. + eps) -x-> (0, L)
		// ""                     -y-> (L, 0)
		xy = (xy - get_o_()) / ZOOM() + get_o_();
		Vector2f ret = (get_f_() * (xy + eps*Vector2d::Ones())).cast<float>();
		ret.y() = get_L_() - ret.y();
		return ret;
	} 
	Vector2d _2xy(const Vector2f uv_) { return _2xy(Vector2d(uv_.cast<double>())); }
	Vector2d _2xy(Vector2d uv) {
		// ZOOM->(-eps, 1. + eps) <-x- (0, L)
		//                     "" <-y- (L, 0)
		uv.y() = get_L_() - uv.y();
		auto tmp = (((1. / get_f_()) * uv) - eps*Vector2d::Ones()); 
		return (tmp - get_o_()) * ZOOM() + get_o_();
	}

	void strokeRect(const Vector2d &lr_, const Vector2d &LR_, const NVGcolor &COLOR) {
		auto lr = _2nvg(lr_);
		auto LR = _2nvg(LR_);
		auto wh = LR - lr;
		// --
		nvgReset(vg);
		nvgBeginPath(vg);
		nvgRect(vg, lr.x(), lr.y(), wh.x(), wh.y());
		nvgStrokeColor(vg, COLOR);
		nvgStrokeWidth(vg, pixelRatio);
		nvgStroke(vg); 
	} 

	float CIRCLE_RADIUS = 3.f;
	void fillCircle(const Vector2d &s_, const NVGcolor &COLOR) {
		auto s = _2nvg(s_);
		// --
		nvgReset(vg);
		nvgBeginPath(vg);
		nvgCircle(vg, s.x(), s.y(), pixelRatio * CIRCLE_RADIUS);
		nvgFillColor(vg, COLOR);
		nvgFill(vg); 
	} 
 
	void drawTriangle(const Triangle &triangle, const VectorXd &x) {
		nvgReset(vg);
		nvgBeginPath(vg);
		auto a = _2nvg(triangle.nodePos(0, x));
		auto b = _2nvg(triangle.nodePos(1, x));
		auto c = _2nvg(triangle.nodePos(2, x));
		nvgMove2(vg, a);
		nvgLine2(vg, b);
		nvgLine2(vg, c);
		nvgLine2(vg, a);
		nvgStrokeColor(vg, WHITE);
		nvgStrokeWidth(vg, pixelRatio * 2);
		nvgStroke(vg);
		/*/ nvgFillColor(vg, RED); /*/ nvgFillColor(vg, colorMap(16.*triangle.energy(x))); /**/
		nvgFill(vg);
	}

	void drawVector(const Vector2d &s_, const Vector2d &F_, const NVGcolor &COLOR) {
		Vector2f s = _2nvg(s_);
		Vector2f t = _2nvg(s_ + F_);
		Vector2f st = (t - s);
		Vector2f e = pixelRatio * CIRCLE_RADIUS * Vector2f(-st.y(), st.x()).normalized();
		Vector2f sP = s + e;
		Vector2f sM = s - e;
		// --
		nvgReset(vg);
		nvgBeginPath(vg);
		nvgMoveTo(vg, t.x(), t.y());
		nvgLineTo(vg, sP.x(), sP.y());
		nvgLineTo(vg, sM.x(), sM.y());
		nvgLineTo(vg, t.x(), t.y());
		nvgFillColor(vg, COLOR);
		nvgFill(vg); 
	}; 

	void drawPin(const Pin &pin, const VectorXd &x) { fillCircle(pin.nodePos(0, x), WHITE); } 
	void nvgMove2(NVGcontext *vg, Vector2f v) { nvgMoveTo(vg, v.x(), v.y()); }
	void nvgLine2(NVGcontext *vg, Vector2f v) { nvgLineTo(vg, v.x(), v.y()); } 
}; 

int main(int, char**) {
    FEMApp app(720, 720, "FEMApp", !I_AM_HAVING_SCREEN_ISSUES ? 1.f : 2.f);
    app.run(); 
    return 0;
}
