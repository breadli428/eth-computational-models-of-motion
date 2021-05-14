#ifdef WIN32
#define NOMINMAX
#endif
#include <application.h>
#include <imgui.h>
#include <imgui_multiplot.h>
#include <chrono>

#include <BernHelpers.h>
#include <ObjectiveFunction.h>
#include <GradientDescentMinimizer.h>
#include <NewtonFunctionMinimizer.h>

bool I_AM_HAVING_SCREEN_ISSUES = false; 

//////////////////////////////////////////////////////////////////////////////// 
typedef vector<Vector2d> Traj;
VectorXd stackTraj(const Traj &u) { VectorXd u_stack; u_stack.setZero(2 * u.size()); for (int i = 0; i < u.size(); ++i) { u_stack.segment(2 * i, 2) = u[i]; } return u_stack; }
Traj unstackTraj(const VectorXd &u_stack) { Traj u; for (int i = 0; i < u_stack.size() / 2; ++i) { u.push_back(u_stack.segment(2 * i, 2)); } return u; } 
////////////////////////////////////////////////////////////////////////////////

Vector2d x0 = Vector2d(0., 1.);
Vector2d v0 = Vector2d(1., 0.);
const double h = .033;
const double m = 1.; 
Vector2d x_sun = Vector2d::Zero();
double r_prime = .5f;

Vector2d get_Fk(const Vector2d &uk, const Vector2d &xk) { return uk + from_(xk, x_sun).normalized() / pow(1e-2 + (from_(xk, x_sun).norm()), 2.); };
pair<Vector2d, Vector2d> stepPhysicsSemiImplicitEuler(const Vector2d &xk, const Vector2d &vk, const Vector2d &uk) {
	Vector2d vkp1 = vk + h * get_Fk(uk, xk) / m; 
	Vector2d xkp1 = xk + h * vkp1;
	return std::make_pair(xkp1, vkp1);
} 

pair<Traj, Traj> get_xv(const Traj &u) {
	// (x0, v0) --(u0, ..., uKm1)--> (x0, x1, ..., xK)
	Traj x = { x0 };
	Traj v = { v0 };
	for (int k = 0; k < u.size(); ++k) {
		auto xv_kp1 = stepPhysicsSemiImplicitEuler(x[k], v[k], u[k]);
		x.push_back(xv_kp1.first);
		v.push_back(xv_kp1.second);
	} 
	return make_pair(x, v);
}
Traj get_x(const Traj &u) { return get_xv(u).first;  } 
Traj get_v(const Traj &u) { return get_xv(u).second; }

////////////////////////////////////////////////////////////////////////////////

// NOTE: The control inputs supplied to the ship when you simulate are
//       (u_curr[0], ... u_curr[K-1], 0, 0, ... )
//       i.e. the contents of u_curr in order, followed by zeros forever.
const int K = 32;

// BEG YOUR CODE HERE ONLY
class ChallengeObjective : public ObjectiveFunction {
public: 
	virtual double evaluate(const VectorXd &u_stack) const { 
		auto u = unstackTraj(u_stack);
		double O = 0.;
		// TODO: O += ...
		
		return O;
	}
};
// END YOUR CODE HERE ONLY

class ChallengeApp : public Application { 

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private: 
	ChallengeObjective objective = ChallengeObjective();
	Traj u_curr; // Current best guess of solution.
	vector<double> O_history; // Objective value trace.

public:
    bool OPTIMIZE = false;
    bool SIMULATE = false;
	int k_simulate = -1;
	Vector2d u_simulate = Vector2d::Zero();
	Vector2d x_simulate = Vector2d::Zero();
	Vector2d v_simulate = Vector2d::Zero();
	void resetOptimization() {
		u_curr.clear(); {
			for (int _ = 0; _ < K; ++_) { u_curr.push_back(Vector2d::Zero()); }
		}
	}
	void resetPlot(){
		O_history.clear(); 
		auto O0 = objective.evaluate(stackTraj(u_curr));
		for (int _ = 0; _ < 128; ++_) { O_history.push_back(O0); }		
	}
	void resetSimulation() {
		k_simulate = -1;
		x_simulate = x0;
		v_simulate = v0;
		u_simulate.setZero(2);
	}
    bool SLOMO = false; 
	bool PRINT_O = false;
	bool PRINT_magG = false;

public:
    ChallengeApp(int width, int height, const char * title, float pixelRatio = 2.f) : Application(title, width, height) { 
		ImGui::StyleColorsDark();
		
		resetOptimization();
		// --
		resetPlot();
		// --
		resetSimulation();
    } 

public: 
    void process() override {
		static std::chrono::high_resolution_clock::time_point lastFrame = std::chrono::high_resolution_clock::now(); 
        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::milliseconds>(now - lastFrame).count() > ((SLOMO) ? 320 : 16)){ 

            if (OPTIMIZE) {
				auto u_stack = stackTraj(u_curr); {
					GradientDescentLineSearch minimizer = GradientDescentLineSearch(1, 1e-10);
					minimizer.minimize(&objective, u_stack);
					push_back_pop_front(O_history, objective.evaluate(u_stack));
				} u_curr = unstackTraj(u_stack); 
				if (PRINT_O) { cout << " O  = " << objective.evaluate(u_stack) << endl; }
				if (PRINT_magG) { cout << "|G| = " << objective.getGradient(u_stack).norm() << endl; }
            } 

			if (SIMULATE) {
				k_simulate++;
				u_simulate = (k_simulate < u_curr.size()) ? u_curr[k_simulate] : Vector2d::Zero();
				auto xv_next = stepPhysicsSemiImplicitEuler(x_simulate, v_simulate, u_simulate);
				x_simulate = xv_next.first;
				v_simulate = xv_next.second;
			} else {
				resetSimulation();
			}


		lastFrame = now; }
    } 

private: 
	bool DRAW_THRUSTER = true;
	bool DRAW_VELOCITY = false;
	void drawNanoVG() override {
		auto x = get_x(u_curr);
		auto v = get_v(u_curr);
		// --
		strokeRect(Vector2d(-1., -1.), Vector2d(1., 1.), GRAY);
		strokeRect(Vector2d( 0.,  0.), Vector2d(1., 1.), GRAY);
		fillCircle(x_sun, YELLOW, 3.);
		if (!SIMULATE) {
			for (int k = 0; k <= K; ++k) {
				if (DRAW_VELOCITY) { drawVelocity(x[k], v[k], BLUE); }
				if (DRAW_THRUSTER && k < K) { drawThruster(x[k], u_curr[k], WHITE); }
				drawShip(x[k], 0., colorMap(.5*v[k].squaredNorm()));
			}
			drawCircle(x_sun, r_prime, PURPLE);
		} else {
			drawCircle(x_sun, r_prime, PURPLE);
			if (DRAW_VELOCITY) { drawVelocity(x_simulate, v_simulate, BLUE); }
			if (DRAW_THRUSTER) { drawThruster(x_simulate, u_simulate, WHITE); }
			drawShip(x_simulate, 0., colorMap(v_simulate.squaredNorm()));
		} 
    }

	ImColor PLOT_COLOR = ImColor(249, 38, 114);
    void drawImGui() override { using namespace ImGui; 
        Begin("NOTE: Press r to reset (optimization or simulation)."); 
        Checkbox("OPTIMIZE // Keyboard Shortcut: o", &OPTIMIZE);
        Checkbox("SIMULATE // Keyboard Shortcut: s",  &SIMULATE );
        Checkbox("SLOMO",     &SLOMO   ); 
		Checkbox("PRINT  O",  &PRINT_O);
		Checkbox("PRINT |G|", &PRINT_magG);
		Checkbox("DRAW_THRUSTER", &DRAW_THRUSTER);
        Checkbox("DRAW_VELOCITY", &DRAW_VELOCITY);
		SliderScalar("ZOOM", ImGuiDataType_Double, &ZOOM_, &ZOOM_MIN_, &ZOOM_MAX_); 
		{ // D:
            const char ** names = new const char*[1]; names[0] = "...";
			ImColor *colors = { &PLOT_COLOR };
			float **functionValues = new float*[1]; float *singleton = new float[O_history.size()]; for (int i = 0; i < O_history.size(); ++i) { singleton[i] = O_history[i]; } functionValues[0] = singleton;
            PlotMultiLines("O(u_curr)", 1, names, colors, [](const void *data, int idx)->float { return ((const float*)data)[idx]; }, ((const void *const *)functionValues), O_history.size(), O_history.size(), 0, 0., max_element(O_history), ImVec2(0, 150)); 
        } 
        End(); 
    }

protected: 
    void mouseButtonPressed (int, int) override { }
    void mouseButtonReleased(int, int) override { } 
	void mouseMove(double, double) override { } 
	void keyPressed(int key, int mods) override {
		if (key == GLFW_KEY_O) { toggle(OPTIMIZE); SIMULATE = false; }
		if (key == GLFW_KEY_S) { toggle(SIMULATE); OPTIMIZE = false; }
		if (key == GLFW_KEY_R) { resetOptimization(); resetPlot(); resetSimulation();  } // *
	}

private: 
	double eps = .25; // * otherwise stuff doesn't show up
	double get_L_() { return double(std::min(height, width)); }
	double get_f_() { return get_L_() / (2. + 2 * eps); }
	double ZOOM_ = 1.f;
	double ZOOM_MIN_ = .1f;
	double ZOOM_MAX_ = 2.f;
	double ZOOM() { return 1. / ZOOM_; }
	Vector2f _2nvg(Vector2d xy) {
		// zoom*(-1. - eps, 1. + eps) -x-> (0, L)
		// ""                         -y-> (L, 0)
		xy /= ZOOM();
		#if defined __APPLE__ 
		Vector2f ret = (get_f_() * (xy + (1. + eps)*Vector2d(1, 1.5))).cast<float>();
		#else
		Vector2f ret = (get_f_() * (xy + (1. + eps)*Vector2d::Ones())).cast<float>();
		#endif
		ret.y() = get_L_() - ret.y();
		return ret;
	} 
	Vector2d _2xy(const Vector2f uv_) { return _2xy(Vector2d(uv_.cast<double>())); }
	Vector2d _2xy(Vector2d uv) {
		// zoom*(-1. - eps, 1. + eps) <-x- (0, L)
		//                    "" <-y- (L, ))
		uv.y() = get_L_() - uv.y();
		double _1of = 1. / get_f_();
		return ZOOM()*((_1of * uv) - (1. + eps) * Vector2d::Ones()); 
	}

	void drawShip(const Vector2d &s_, const double &theta, const NVGcolor &COLOR) { Vector2d o = .02*Vector2d::Ones(); fillRect(s_ - o, s_ + o, COLOR); }
	void drawThruster(const Vector2d &s_, const Vector2d &F_, const NVGcolor &COLOR) { drawVector(s_, -.1*F_, COLOR); } 
	void drawVelocity(const Vector2d &s_, const Vector2d &v_, const NVGcolor &COLOR) { drawVector(s_, .1*v_, COLOR); } 

	void strokeRect(const Vector2d &lr_, const Vector2d &LR_, const NVGcolor &COLOR) {
		Vector2f lr = _2nvg(lr_);
		Vector2f LR = _2nvg(LR_);
		Vector2f wh = LR - lr;
		// --
		nvgReset(vg);
		nvgBeginPath(vg);
		nvgRect(vg, lr.x(), lr.y(), wh.x(), wh.y());
		nvgStrokeColor(vg, COLOR);
		nvgStroke(vg); 
	} 

	void fillRect(const Vector2d &lr_, const Vector2d &LR_, const NVGcolor &COLOR) {
		Vector2f lr = _2nvg(lr_);
		Vector2f LR = _2nvg(LR_);
		Vector2f wh = LR - lr;
		// --
		nvgReset(vg);
		nvgBeginPath(vg);
		nvgRect(vg, lr.x(), lr.y(), wh.x(), wh.y());
		nvgFillColor(vg, COLOR);
		nvgFill(vg); 
	} 

	float CIRCLE_RADIUS = 4.f;
	void fillCircle(const Vector2d &s_, const NVGcolor &COLOR, double SCALE=1.) {
		Vector2f s = _2nvg(s_);
		// --
		nvgReset(vg);
		nvgBeginPath(vg);
		nvgCircle(vg, s.x(), s.y(), SCALE * CIRCLE_RADIUS);
		nvgFillColor(vg, COLOR);
		nvgFill(vg); 
	} 

	void loopPoints(const vector<Vector2d> &P_, const NVGcolor &COLOR, float STROKE_WIDTH=1.f) {
		vector<Vector2f> P; for (const auto &p_ : P_) { P.push_back(_2nvg(p_)); }
		nvgReset(vg);
		nvgBeginPath(vg);
		nvgMove2(vg, P[0]);
		for (int i = 1; i <= P.size(); ++i) { nvgLine2(vg, loop_access(P, i)); }
		nvgStrokeColor(vg, COLOR);
		nvgStrokeWidth(vg, pixelRatio * STROKE_WIDTH);
		nvgStroke(vg); 
	}
	void drawCircle(const Vector2d &s_, const double &r_, const NVGcolor &COLOR) {
		vector<Vector2d> C;
		int N = 64;
		for (int i = 0; i < N; ++i) { C.push_back(s_ + r_*e_theta(2. * PI * double(i) / double(N))); }
		loopPoints(C, COLOR);
	} 

	void drawVector(const Vector2d &s_, const Vector2d &F_, const NVGcolor &COLOR) {
		Vector2f s = _2nvg(s_);
		Vector2f t = _2nvg(s_ + F_);
		Vector2f st = t - s;
		Vector2f e = CIRCLE_RADIUS * Vector2f(-st.y(), st.x()).normalized();
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
	}

	void nvgMove2(NVGcontext *vg, Vector2f v) { nvgMoveTo(vg, v.x(), v.y()); }
	void nvgLine2(NVGcontext *vg, Vector2f v) { nvgLineTo(vg, v.x(), v.y()); } 
}; 

int main(int, char**) {
    ChallengeApp app(720, 720, "ChallengeApp", !I_AM_HAVING_SCREEN_ISSUES ? 1.f : 2.f);
    app.run(); 
    return 0;
}

