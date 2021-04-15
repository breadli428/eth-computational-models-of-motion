#pragma once

#include "EnergyFunction.h"
#include "Element.h"
#include "Triangle.h"
#include "Pin.h"
#include <NewtonFunctionMinimizer.h>

class Simulation {

public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
	vector<Triangle> triangles;
	mutable vector<Pin> pins;
	VectorXd x, X; // deformed and undeformed nodal positions
	VectorXd v;    // nodal velocities

	bool SOLVE_DYNAMICS = true;
	double h = .01; // timeStep
	VectorXd M;     // mass matrix (diagonal)

	bool APPLY_GRAVITY = true;
	VectorXd f_ext; // external force (gravity)

	vector<Vector2d> handleCenters; // * These are set permanently in the constructor

public:
	int C;
	int R;
	int i_(int r, int c) const { return r * C + c; }; // index of node at row R and col C 
	int R_mid_() const { return (R - 1) / 2; }
	Simulation(int TEST_CASE = 0, bool SPARSE_BAR = false) { 
		double L = 1.; C = (SPARSE_BAR) ? 15 : 30;
		// C = 2   // Uncomment for a reallllly sparse bar (useful for debugging).
		// C = 100 // Uncomment to stress test.
		double H = .1; R = max(3, int(H/L*C));
		double s_C = L / (C - 1);
		double s_R = H / (R - 1);
		// :
		// C ...
		// 0 1 2 ...
		vector<VectorXd> X; // *
		vector<array<int, 3>> triangles; 
		for (int r = 0; r < R; ++r) {
			for (int c = 0; c < C; ++c) {
				X.push_back(Vector2d(c*s_C, .5 - .5*H + r*s_R));
			}
		}
		for (int r = 0; r < R - 1; ++r) {
			for (int c = 0; c < C - 1; ++c) {
				triangles.push_back({ i_(r,     c    ), i_(r + 1, c    ), i_(r + 1, c + 1) });
				triangles.push_back({ i_(r,     c    ), i_(r    , c + 1), i_(r + 1, c + 1) });
			}
		} 
		this->x = this->X = stack_(X); 
		this->v.setZero(this->X.size());
		for (const auto &nodeIndices : triangles) {
			this->triangles.push_back(Triangle(nodeIndices, this->X));
		} 
		this->f_ext.setZero(this->X.size());
		this->M.setZero(this->X.size());
		Vector2d g = Vector2d(0., -10.);
		double massPerArea = 1.;
		for (const auto &tri : this->triangles) {
			auto A = tri.area(this->X);
			for (const auto &i : tri.nodeIndices) {
				double m = (A / 3.) * massPerArea;
				seg2(this->M,     i) += m * Vector2d::Ones();
				seg2(this->f_ext, i) += m * g;
			}
		} 
		for (int r = 0; r < R; ++r) {
			for (const auto &c : (TEST_CASE == 0) ? vector<int>({ 0 }) : vector<int>({0, C - 1})) {
				this->pins.push_back(Pin(i_(r, c), seg2(this->X, i_(r, c)), 1e6));
				this->pins.back().handleIndex = (c == 0) ? 0 : 1;
			}
		}
		handleCenters = { seg2(this->X, i_(R_mid_(), 0)), seg2(this->X, i_(R_mid_(), C - 1)) };
	}

	vector<int> getFeaturePointsNodalIndices() const {
		vector<int> ret;
		for (const int &c : { int(C/4), int(3*C/4) }) { ret.push_back(i_(R_mid_(), c)); }
		return ret;
	}

	vector<Vector2d> getFeaturePoints(const VectorXd &x) const {
		vector<Vector2d> ret;
		for (const auto &i : getFeaturePointsNodalIndices()) { ret.push_back(x.segment(2 * i, 2)); }
		return ret;
	}

public: 
	VectorXd stackControl(const vector<Vector2d> &handleTranslations, const vector<double> &handleRotations) const {
		vector<VectorXd> _2stack;
		for (const auto &handleTranslation : handleTranslations) { _2stack.push_back(VectorXd(handleTranslation)); }
		_2stack.push_back(vecDub2VectorXd(handleRotations));
		return stack_(_2stack);
	}

	pair<vector<Vector2d>, vector<double>> unstackControl(const VectorXd &u) const {
		vector<Vector2d> handleTranslations;
		vector<double> handleRotations;
		int H = u.size() / 3;
		for (int i = 0; i < H; ++i) {
			handleTranslations.push_back(Vector2d(u.segment<2>(2 * i)));
			handleRotations.push_back(u[2 * H + i]);
		}
		return std::make_pair(handleTranslations, handleRotations);
	} 

	void updatePinAnchorsAccordingToHandles(const VectorXd &u) const {
		auto tmp = unstackControl(u);
		auto handleTranslations = tmp.first;
		auto handleRotations = tmp.second;
		for (auto &pin : pins) {
			auto T = handleTranslations[pin.handleIndex];
			auto theta = handleRotations[pin.handleIndex];
			auto o = handleCenters[pin.handleIndex];
			auto s_X = pin.nodePos(0, X);
			// --
			// theta = 0.;
			// T.setZero();
			// --
			pin.anchorPosition = o + rotate_(from_(o, s_X), theta) + T;
		}
	} 

private:
	mutable bool PUSHED_STATE_ = false; 
	mutable vector<Vector2d> pinAnchorPositions_PUSH_;

public:
	void pushSimulationState() const {
		if (setTrueButReturnOriginalValue(PUSHED_STATE_)) { throw std::runtime_error("double push"); }
		pinAnchorPositions_PUSH_.clear(); {
			for (const auto &pin : pins) { pinAnchorPositions_PUSH_.push_back(pin.anchorPosition); }
		}
	}
	void popSimulationState() const {
		if (!setFalseButReturnOriginalValue(PUSHED_STATE_)) { throw std::runtime_error("double pop");  }
		for (int p = 0; p < pins.size(); ++p) { pins[p].anchorPosition = pinAnchorPositions_PUSH_[p]; }
	}

public:

	bool CHECK_dEdx   = false;
	bool CHECK_d2Edx2 = false;
	EnergyFunction buildEnergyFunction(VectorXd x_prev=VectorXd(), VectorXd v_prev=VectorXd()) const {
		EnergyFunction energy;
		energy.checkGradient = CHECK_dEdx;
		energy.checkHessian = CHECK_d2Edx2;
		// --
		for (const auto &_ : triangles) { energy.elements.push_back(&_); }
		for (const auto &_ : pins) { energy.elements.push_back(&_); }
		if (energy.APPLY_GRAVITY = APPLY_GRAVITY) {
			energy.f_ext = f_ext;
		}
		if (energy.SOLVE_DYNAMICS = SOLVE_DYNAMICS) {
			energy.x_prev = (x_prev.size() != 0) ? x_prev : this->x;
			energy.v_prev = (v_prev.size() != 0) ? v_prev : this->v;
			energy.h = h;
			energy.M = M;
		}
		return energy;
	}

	pair<VectorXd, VectorXd> solvePhysics(const VectorXd &u, VectorXd x_prev=VectorXd(), VectorXd v_prev=VectorXd()) const { 
		// Get x, v resulting from applying u @ x_prev, v_prev
		// NOTE: This function returns with the state of simulation unchanged.
		VectorXd x; x.setZero(this->x.size());
		VectorXd v; v.setZero(this->x.size()); 
		if (x_prev.size() == 0) { x_prev = this->x; }
		if (v_prev.size() == 0) { v_prev = this->v; }
		pushSimulationState(); {
			// 1) Build E(x; u, x_prev, v_prev) // NOTE: x_prev, v_prev used for dynamics.
			auto energy = buildEnergyFunction(x_prev, v_prev); 
			updatePinAnchorsAccordingToHandles(u); // NOTE: Set pin anchor positions according to u.
			// 2) Solve x(u) = arg min E(x; u);
			VectorXd x_solve = x_prev; { // NOTE: warm-start optimiztion @ x_prev
				NewtonFunctionMinimizer solver(25, 1e-6, 15); solver.reg = (SOLVE_DYNAMICS) ? 0. : .001; 
				solver.minimize(&energy, x_solve);
			}
			x = x_solve;
			// 3) Calculate v(x, x_prev)
			// NOTE: for statics                 v = 0
			// NOTE: for implicit euler dynamics x = x_prev + h * v
			if (!SOLVE_DYNAMICS) { v.setZero(); } else { v = (x - x_prev) / h; } 
		} popSimulationState();
		return std::make_pair(x, v);
	}

	void storePositionAndVelocity(pair<VectorXd, VectorXd> xv) {
		x = xv.first;
		v = xv.second;
	} 
};

