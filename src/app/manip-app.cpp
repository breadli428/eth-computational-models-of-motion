#ifdef WIN32
#define NOMINMAX
#endif
#include <application.h>
#include <imgui.h>
#include <imgui_multiplot.h>

#include <iostream>
#include <chrono>
#include <BernHelpers.h>
#include <Simulation.h>
#include <NewtonFunctionMinimizer.h>

bool I_AM_HAVING_SCREEN_ISSUES = false; 
bool DRAW_MOUSE_POSITION = false; 
bool THE_SIM_RUNS_TOO_SLOWLY = false; 
double globalTime = 0.; 

class ManipObjective : public ObjectiveFunction {

public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
	Simulation *sim = nullptr;
	VectorXd get_x(const VectorXd &u) const { return sim->solvePhysics(u).first; } // Wrapper function to compute x(u)
	bool QUESTION_12 = false;
	
	virtual double evaluate(const VectorXd &u) const {
		// Given u, compute O(x(u))
		auto x = get_x(u);
		double O = get_O(x);

        if (QUESTION_12) {
            
        }

        return O;
	}

	double get_O(const VectorXd &x) const {
		// // Given x, compute O(x).

		// Our objective tries to bring a few specific nodes of the mesh as close as possible to corresponding target positions.
		// We call these specific nodes "feature points." 
		// For convenience, here are their indices as a std::vector of ints.
		auto I = sim->getFeaturePointsNodalIndices();

		// We can define now define our objective succintly as
		// + O = sum_{i \in I} 0.5 * || x_i - x'_i || ^ 2,
		// ++ where x_i is the deformed position of the i-th node 
		// ++ and x'_i is the target position of the i-th node.
		// ++ NOTE: Both x_i and x'_i are size 2 vectors (containing x and y position) 
 
		// Also for convenience, here is x' as a column vector (NOTE: x' is implicitly chosen by you aa you move around the red spline)
		// + x' is size 2 * N, just like x(u)
		// + The i-th 2-segment of x' is either...
		// ++ The target position of the i-th node IF        i \in I
		// ++ 0                                    OTHERWISE
		auto x_prime = get_x_prime();

		double O = 0.;
		for (const int &i : I) {
			O += .5 * (seg2(x, i) - seg2(x_prime, i)).squaredNorm();
		} 

		return O;
	}

	bool CHECK_GRADIENT = false;
	virtual void addGradientTo(const VectorXd &u, VectorXd &grad) const {
		grad += get_DODu(u);
	}

	enum STEP { FD_O_of_u=0, FD_BIG_PIECES=1, ANALYTIC_dOdx=2, SOLVE_dxdu=3, ANALYTIC_dFdu=4 };
	STEP step = FD_O_of_u;
	VectorXd get_DODu(const VectorXd &u) const {
		// // Given u, compute DOdu|(u, x(u)) 
		// Please recall DODu = dxdu^T * dOdx
		// + dxdu is a |x| by |u| matrix
		// + dOdx is a |x| column 
 
		// NOTE: These are used to estimate derivatives with finite differences.
		auto x_of_u_wrapper = [&](const VectorXd &u) -> VectorXd { return get_x(u); };
		auto O_of_x_wrapper = [&](const VectorXd &x) -> double   { return get_O(x); }; 

		VectorXd DODu; DODu.setZero(u.size()); {

			auto x = get_x(u); 
			auto F_of_u_NOT_x_of_u_wrapper = [&](const VectorXd &u) -> VectorXd {return -get_G(u, x); }; // Another wrapper for finite differencing.

			if (step == FD_O_of_u) {
				addFiniteDifferenceGradientTo(u, DODu);
			} else if (step == FD_BIG_PIECES) {
				// Goal: Make sure you wrote down the chain rule correctly.
				VectorXd dOdx_FD = vecFD(x, O_of_x_wrapper);
				MatrixXd dxdu_FD = matFD(u, x_of_u_wrapper);
				// --
				// TODO: DODu += ...
			} else if (step == ANALYTIC_dOdx) {
				// Goal: Compute dOdx analytically.
				VectorXd dOdx = get_dOdx(x); // TODO: Implement get_dOdx(...)
				MatrixXd dxdu_FD = matFD(u, x_of_u_wrapper);
				// --
				// TODO: DODu += ...
			} else if (step == SOLVE_dxdu) {
				// Goal: Solve dxdu.
				VectorXd dOdx = get_dOdx(x);
				SparseMatrixd dFdu = dense2sparse(matFD(u, F_of_u_NOT_x_of_u_wrapper));
				// TODO: MatrixXd dxdu = ...
				// --
				// TODO: DODu += ...
				
			} else if (step == ANALYTIC_dFdu) {
				// Goal: Analytic everything :)
				VectorXd dOdx = get_dOdx(x);
				// TODO: SparseMatrixd dFdx = ...
				SparseMatrixd dFdu = get_dFdu(u, x); // TODO: Finish implementing get_dOdx(...)
				// TODO: MatrixXd dxdu = ...
				// --
				// TODO: DODu += ...
				
			}

			if (CHECK_GRADIENT) { 
				VectorXd DODu_FD; DODu_FD.setZero(DODu.size());
				addFiniteDifferenceGradientTo(u, DODu_FD);
				if (checkVectorXd(DODu_FD, DODu)) {
					cout << string_format("[STEP%d] DODu checks.", step) << endl;
				}
			}
		} 

		return DODu;
    }

	VectorXd get_dOdx(const VectorXd &x) const {
		cout << "NotImplementedWarning:get_dOdx" << endl;
		auto I = sim->getFeaturePointsNodalIndices(); 
		auto x_prime = get_x_prime();
		// --
		VectorXd dOdx; dOdx.setZero(x.size()); 
		return dOdx;
	} 

	SparseMatrixd get_dFdu(const VectorXd &u, const VectorXd &x) const {
		// cout << "NotImplementedWarning:get_dFdu" << endl;
		
		SparseMatrixd ret; ret.resize(x.size(), u.size()); {
			for (const auto &pin : sim->pins) { 
				auto F_DSI = 2 * pin.nodeIndex;
				const auto &k = pin.k;
				auto p = pin.getPosition(x);
				const auto &p_prime = pin.anchorPosition;
				// --
				// Recall F = -k * (p - p')
				// => dFdu = dFdp' dp'du
				// dFdp' = k
				MatrixXd dFdp_prime; dFdp_prime.setZero(2, 2); {
					dFdp_prime = k * Matrix2d::Identity();
				}
				// dp'du = ...
				MatrixXd dp_primedu; dp_primedu.setZero(2, u.size()); {
					// Assume: u = [ x_L y_L x_R y_R theta_L theta_R ]
					// e.g. x_L is x-component of translation of left handle
					//      theta_R is rotation of right handle
					bool BELONGS_TO_LEFT_HANDLE = pin.handleIndex == 0;
					int xy_dataStartIndex    = (BELONGS_TO_LEFT_HANDLE) ? 0 : 2;
					int theta_dataIndex      = (BELONGS_TO_LEFT_HANDLE) ? 4 : 5;
					auto theta = u[theta_dataIndex];
					auto o = sim->handleCenters[pin.handleIndex];
					auto s_X = pin.nodePos(0, sim->X);
					auto f = from_(o, s_X);
					// --
					// From updatePinAnchorsAccordingToHandles(...): pin.anchorPosition = o + rotate_(from_(o, s_X), theta) + T;
					// --> p' = o + R_theta{f} + xy; 
					
					// => dp'dxy = ???
					// TODO: dp_primedu.block<2, 2>(0, xy_dataStartIndex) = ...
					
					// => dp'dtheta = ???
					// TODO: dp_primedu.col(theta_dataIndex) = ...

				} 
				MatrixXd dFdu = dFdp_prime * dp_primedu;
				writeSparseMatrixDenseBlockAdd(ret, F_DSI, 0, dFdu, false); 
				// // NOTE: You can use this to check the single block computed above.
				// auto F_wrapper = [&](const VectorXd &u) {
				// 	VectorXd ret;
				// 	sim->pushSimulationState(); {
				// 		sim->updatePinAnchorsAccordingToHandles(u);
				// 		ret = -pin.gradient(x);
				// 	} sim->popSimulationState();
				// 	return ret;
				// };
				// cout << "BEG:CHECK_dFdu" << endl;
				// checkMatrixXd(matFD(u, F_wrapper), dFdu);
				// cout << "END:CHECK_dFdu" << endl; 
			}
		}
		return ret; 
	} 

public:
	MatrixXd solve_AX_EQUALS_B(const SparseMatrixd &A, const SparseMatrixd &B) const { 
		// Solve AX = B.
		// NOTE: Much faster than explicitly computing A^{-1} and then calculating A^{-1}*B
		Solver solver;
		solver.compute(A);
		return solver.solve(B); 
	}

public:
	vector<Vector2d> targetFeaturePoints;
	VectorXd get_x_prime() const { // Wrapper function to assemble x'
		VectorXd x_prime; x_prime.setZero(sim->x.size());
		auto I = sim->getFeaturePointsNodalIndices(); 
		for (int ii = 0; ii < targetFeaturePoints.size(); ++ii) {
			seg2(x_prime, I[ii]) = targetFeaturePoints[ii];
		}
		return x_prime;
	}

public: 
	// Wrapper function to compute hessian of entire simulation.
	SparseMatrixd get_H(const VectorXd &u, const VectorXd &x) const {
		SparseMatrixd H;
		sim->pushSimulationState(); {
			auto energy = sim->buildEnergyFunction(); 
			sim->updatePinAnchorsAccordingToHandles(u);
			energy.getHessian(x, H);
		} sim->popSimulationState();
		return H; 
	} 

	// Wrapper function to compute gradient of entire simulation.
	VectorXd get_G(const VectorXd &u, const VectorXd &x) const {
		VectorXd G;
		sim->pushSimulationState(); {
			auto energy = sim->buildEnergyFunction(); 
			sim->updatePinAnchorsAccordingToHandles(u);
			G = energy.getGradient(x);
		} sim->popSimulationState();
		return G; 
	} 

};


class ManipApp : public Application { 

public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public: 
	Simulation sim = Simulation(1, THE_SIM_RUNS_TOO_SLOWLY);
	ManipObjective obj;

public:
    bool OPTIMIZE = false;
	bool PRINT_u_O_DODu = false;
	vector<Vector2d> handleTranslations = { Vector2d(0., 0.), Vector2d(0., 0.) };
	vector<double>   handleRotations = { 0., 0. };
    bool SIMULATE = true;
    bool SLOMO = false; 
	bool DRAW_CORRESPONDENCE = true;
	bool BORING_COLORS = false;

public:
	vector<Vector2d> targetFeaturePoints;

public:
    ManipApp(int w, int h, const char *title, float pixelRatio = 2.f) : Application(title, w, h) {
		sim.APPLY_GRAVITY = sim.SOLVE_DYNAMICS = false;
		targetFeaturePoints = sim.getFeaturePoints(sim.X);
	} 

public: 
    void process() override { 
		globalTime += .01;
        static std::chrono::high_resolution_clock::time_point lastFrame = std::chrono::high_resolution_clock::now(); 
        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::milliseconds>(now - lastFrame).count() > ((SLOMO) ? 320 : 16)){ 

			if (OPTIMIZE) { 
				// 1) Specify objective
				obj.sim = &sim;
				obj.targetFeaturePoints = targetFeaturePoints;
				// 2) Take a single step of gradient descent.
				auto u = sim.stackControl(handleTranslations, handleRotations);
				{
					if (PRINT_u_O_DODu) {
						cout << "   u = " << u.transpose() << endl;
						cout << "   O = " << obj.evaluate(u) << endl;
						cout << "DODu = " << obj.getGradient(u).transpose() << endl;
					}
					GradientDescentLineSearch minimizer(1, 1e-6, 15);
					minimizer.minimize(&obj, u);
				}
				auto tmp = sim.unstackControl(u); 
				handleTranslations = tmp.first;
				handleRotations = tmp.second;
			}

			if (SIMULATE) {
				sim.storePositionAndVelocity(sim.solvePhysics(sim.stackControl(handleTranslations, handleRotations)));
			}

		lastFrame = now; }
    } 

////////////////////////////////////////////////////////////////////////////////

public: 
	void drawNanoVG() override { 
		strokeRect(Vector2d( 0.,  0.), Vector2d(1., 1.), GRAY);
		for (const auto &tri : sim.triangles) { drawTriangle(tri, sim.x); }
		for (const auto &pin : sim.pins     ) { drawPin(pin, sim.x); }

		{ // Ew.
			double eps_handle = .1;
			const vector<double> theta_handles0 = { PI, 0 };
			for (int i = 0; i < handleTranslations.size(); ++i) {
				auto T = sim.handleCenters[i] + handleTranslations[i];
				auto theta = handleRotations[i] + theta_handles0[i];
				fillCircle(T, DARK_RED);
				drawVector(T, eps_handle * e_theta(theta), DARK_RED);
			}
		}

		{
			auto c = sim.getFeaturePoints(sim.x);
			auto c_prime = targetFeaturePoints;
			// --
			if (DRAW_CORRESPONDENCE) {
				for (int i = 0; i < c.size(); ++i) {
					connectDots({ c[i], c_prime[i] }, BLUE, 3.f);
				}
			}
			drawDots(c, GREEN);
			drawDots(c_prime, RED);
		}

		if (DRAW_MOUSE_POSITION) { auto push = CIRCLE_RADIUS; CIRCLE_RADIUS *= 3; NVGcolor c = GREEN; c.a = .7f; fillCircle(x_mouse, c); CIRCLE_RADIUS = push; } 
    }

    void drawImGui() override { using namespace ImGui; 
        Begin(""); 
        Checkbox("OPTIMIZE // Keyboard Shortcut: [Space]", &OPTIMIZE);
        Checkbox("obj.CHECK_GRADIENT", &obj.CHECK_GRADIENT);
        Checkbox("obj.QUESTION_12", &obj.QUESTION_12);
		const char * items[5] = { "FD_O_of_u", "FD_BIG_PIECES", "ANALYTIC_dOdx", "SOLVE_dxdu", "ANALYTIC_dFdu" };
		Combo("step", (int*)&obj.step, items, 5); 
        Checkbox("PRINT_u_O_DODu", &PRINT_u_O_DODu);
        Checkbox("SIMULATE", &SIMULATE);
        Checkbox("sim.APPLY_GRAVITY  // Keyboard Shortcut: g", &sim.APPLY_GRAVITY);
        Checkbox("sim.SOLVE_DYNAMICS // Keyboard Shortcut: d", &sim.SOLVE_DYNAMICS);
		SliderScalar("ZOOM", ImGuiDataType_Double, &ZOOM_, &ZOOM_MIN_, &ZOOM_MAX_); 
        Checkbox("SLOMO // Keyboard Shortcut: s",    &SLOMO   ); 
		Checkbox("DRAW_CORRESPONDENCE", &DRAW_CORRESPONDENCE);
		Checkbox("BORING_COLORS", &BORING_COLORS);
        End(); 
    } 

protected: 
	int DRAG_i = -1;
	Vector2d x_mouse = { 0., 0. };
	void mouseButtonPressed(int, int) override { 
		DRAG_i = -1; {
			const double thresh = 0.05;
			double minDist = INFINITY;
			int minIndex = -1;
			for (int i = 0; i < targetFeaturePoints.size(); ++i) {
				double dist = (x_mouse - targetFeaturePoints[i]).norm();
				if (dist < minDist) {
					minDist = dist;
					minIndex = i;
				}
			}
			if (minDist < thresh) { DRAG_i = minIndex; }
		} 
	}
	void mouseButtonReleased(int, int) override { DRAG_i = -1; }
	void mouseMove(double, double) override {
	    double f = (I_AM_HAVING_SCREEN_ISSUES) ? .5 : 1.; x_mouse = _2xy(Vector2d(f*mouseState.lastMouseX, f*mouseState.lastMouseY));
		// --
		if (DRAG_i != -1) { targetFeaturePoints[DRAG_i] = x_mouse; }
	}

    void keyPressed(int key, int mods) override {
		if (key == GLFW_KEY_SPACE) { toggle(OPTIMIZE); }
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

	void drawDots(const vector<Vector2d> &P_, NVGcolor &COLOR) {
		for (const auto &p_ : P_) { fillCircle(p_, COLOR); }
	}

	void connectDots(const vector<Vector2d> &P_, NVGcolor &COLOR, float STROKE_WIDTH=1.f) {
		vector<Vector2f> P; for (const auto &p_ : P_) { P.push_back(_2nvg(p_)); }
		nvgReset(vg);
		nvgBeginPath(vg);
		nvgMove2(vg, P[0]);
		for (int i = 1; i < P.size(); ++i) { nvgLine2(vg, P[i]); }
		nvgStrokeColor(vg, COLOR);
		nvgStrokeWidth(vg, pixelRatio * STROKE_WIDTH);
		nvgStroke(vg); 
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
		if (BORING_COLORS) { nvgFillColor(vg, GRAY); } else { nvgFillColor(vg, colorMap(16.*triangle.energy(x))); }
		nvgFill(vg);
	}

	void drawPin(const Pin &pin, const VectorXd &x) { fillCircle(pin.nodePos(0, x), WHITE); } 
	void nvgMove2(NVGcontext *vg, Vector2f v) { nvgMoveTo(vg, v.x(), v.y()); }
	void nvgLine2(NVGcontext *vg, Vector2f v) { nvgLineTo(vg, v.x(), v.y()); }

}; 

int main(int, char**) {
    ManipApp app(720, 720, "ManipApp", !I_AM_HAVING_SCREEN_ISSUES ? 1.f : 2.f);
    app.run(); 
    return 0;
}
