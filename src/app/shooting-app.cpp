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
bool USE_ZERO_INITIAL_GUESS = false; // If you would rather just use a zero initial guess (not a bad idea) :)
const int K = 64; // Number of timesteps (feel free to make smaller if code is running too slowly)

//////////////////////////////////////////////////////////////////////////////// 
typedef vector<Vector2d> Traj;
VectorXd stackTraj(const Traj &u) { VectorXd u_stack; u_stack.setZero(2 * u.size()); for (int i = 0; i < u.size(); ++i) { u_stack.segment(2 * i, 2) = u[i]; } return u_stack; }
Traj unstackTraj(const VectorXd &u_stack) { Traj u; for (int i = 0; i < u_stack.size() / 2; ++i) { u.push_back(u_stack.segment(2 * i, 2)); } return u; } 
////////////////////////////////////////////////////////////////////////////////

Vector2d x0 = Vector2d(0., 0.);
Vector2d v0 = Vector2d(1., 0.);
Vector2d x_prime = Vector2d(0., 1.);
const double h = .033;
const double m = 1.; 
bool SECOND_ORDER_SOLVER = false; 
bool PLANET = false;
Vector2d x_planet = .5 * (x0 + x_prime);

Vector2d get_Fk(const Vector2d &uk, const Vector2d &xk) {
	Vector2d Fk = Vector2d::Zero();
	// TODO: Fk += ...

	return Fk;
} 

pair<Vector2d, Vector2d> stepPhysicsExplicitEuler(const Vector2d &xk, const Vector2d &vk, const Vector2d &uk) {
	// NOTE: I use xkp1 to mean $x_{k+1}$.
	Vector2d xkp1 = xk;
	Vector2d vkp1 = vk;
	// TODO: xkp1 = ...
	// TODO: vkp1 = ...
	
	return std::make_pair(xkp1, vkp1);
} 

pair<Traj, Traj> get_xv(const Traj &u) {
	// (x0, v0) --(u0, ..., uKm1)--> (x0, x1, ..., xK)
	Traj x = { x0 };
	Traj v = { v0 };
	for (int k = 0; k < K; ++k) {
		auto xv_kp1 = stepPhysicsExplicitEuler(x[k], v[k], u[k]);
		x.push_back(xv_kp1.first);
		v.push_back(xv_kp1.second);
	} 
	return make_pair(x, v);
}
Traj get_x(const Traj &u) { return get_xv(u).first;  } 
Traj get_v(const Traj &u) { return get_xv(u).second; }

////////////////////////////////////////////////////////////////////////////////

class ShootingObjective : public ObjectiveFunction {
public:

	// Please feel free to use this in the coefficient of a regularizer as pow(10., log_c_reg).
	// It's already in your GUI.
	double log_c_reg = -3.; double log_c_reg_min = -6.; double log_c_reg_max = 6.;

	virtual double evaluate(const VectorXd &u_stack) const { 
		auto u = unstackTraj(u_stack);
		auto x = get_x(u);
		auto v = get_v(u);
		// --
		double O = 0.;
		// TODO: O += ...
		
		return O;
	}
};

class ShootingApp : public Application { 

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private: 
	ShootingObjective objective = ShootingObjective();
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
		u_curr.clear();
		Vector2d uk_temp = Vector2d::Zero();
		for (int _ = 0; _ < K; ++_) {
			if (!USE_ZERO_INITIAL_GUESS) {
				uk_temp = lerp<Vector2d>(double(_) / double(K - 1),
										Vector2d(-1, 1), Vector2d::Zero());
			}
			u_curr.push_back(uk_temp);
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
    ShootingApp(int width, int height, const char * title, float pixelRatio = 2.f) : Application(title, width, height) { 
		ImGui::StyleColorsDark();
		// --
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
					if (!SECOND_ORDER_SOLVER) {
						GradientDescentLineSearch minimizer = GradientDescentLineSearch(1, 1e-10);
						minimizer.minimize(&objective, u_stack);
					} else {
						NewtonFunctionMinimizer minimizer = NewtonFunctionMinimizer(1, 1e-10);
						minimizer.minimize(&objective, u_stack);
					}
					push_back_pop_front(O_history, objective.evaluate(u_stack));
				} u_curr = unstackTraj(u_stack); 
				if (PRINT_O) { cout << " O  = " << objective.evaluate(u_stack) << endl; }
				if (PRINT_magG) { cout << "|G| = " << objective.getGradient(u_stack).norm() << endl; }
            } 

			if (SIMULATE) {
				k_simulate++;
				u_simulate = (k_simulate < u_curr.size()) ? u_curr[k_simulate] : Vector2d::Zero();
				auto xv_next = stepPhysicsExplicitEuler(x_simulate, v_simulate, u_simulate);
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
		strokeRect(Vector2d( 0.,  0.), Vector2d(1., 1.), GRAY);
		if (PLANET) { fillCircle(x_planet, GREEN, 3.); }
		if (!SIMULATE) {
			for (int k = 0; k <= K; ++k) {
				if (DRAW_VELOCITY) { drawVelocity(x[k], v[k], BLUE); }
				if (DRAW_THRUSTER && k < K) { drawThruster(x[k], u_curr[k], WHITE); }
				drawShip(x[k], 0., colorMap(.5*v[k].squaredNorm()));
			}
			fillCircle(x_prime, PURPLE); 
		} else {
			fillCircle(x_prime, PURPLE); 
			if (DRAW_VELOCITY) { drawVelocity(x_simulate, v_simulate, BLUE); }
			if (DRAW_THRUSTER) { drawThruster(x_simulate, u_simulate, WHITE); }
			drawShip(x_simulate, 0., colorMap(v_simulate.squaredNorm()));
		} 
    }

	ImColor PLOT_COLOR = ImColor(249, 38, 114);
    void drawImGui() override { using namespace ImGui; 
        Begin("NOTE: Press r to reset (optimization or simulation)."); 
        Checkbox("PLANET", &PLANET);
        Checkbox("OPTIMIZE // Keyboard Shortcut: o", &OPTIMIZE);
        Checkbox("SECOND_ORDER_SOVLER", &SECOND_ORDER_SOLVER);
		SliderScalar("log_c_reg", ImGuiDataType_Double, &objective.log_c_reg, &objective.log_c_reg_min, &objective.log_c_reg_max); 
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
	const vector<Vector2d *> DRAGGABLE_POINTS_ = { &x_prime, &x_planet };
	bool DRAGGING = false; Vector2d *DRAG_POINT;
	Vector2d x_mouse = { 0., 0. };
    void mouseButtonPressed (int, int) override {
		vector<double> distances; { for (auto _x : DRAGGABLE_POINTS_) { distances.push_back((x_mouse - *_x).norm()); } }
		DRAGGING = false; {
			if (min_element(distances) < .1) { DRAGGING = true; DRAG_POINT = DRAGGABLE_POINTS_[min_element_index(distances)]; }
		}
	}
    void mouseButtonReleased(int, int) override { DRAGGING = false; } 
	void mouseMove(double, double) override {
		double f = (I_AM_HAVING_SCREEN_ISSUES) ? .5 : 1.; x_mouse = _2xy(Vector2d(f*mouseState.lastMouseX, f*mouseState.lastMouseY));
		if (DRAGGING) { *DRAG_POINT = x_mouse; }
	} 

	void keyPressed(int key, int mods) override {
		if (key == GLFW_KEY_O) { toggle(OPTIMIZE); SIMULATE = false; }
		if (key == GLFW_KEY_S) { toggle(SIMULATE); OPTIMIZE = false; }
		if (key == GLFW_KEY_R) {  resetOptimization(); resetPlot(); resetSimulation();    } // *
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

}; 

int main(int, char**) {
    ShootingApp app(720, 720, "ShootingApp", !I_AM_HAVING_SCREEN_ISSUES ? 1.f : 2.f);
    app.run(); 
    return 0;
}

