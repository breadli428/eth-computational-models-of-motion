#pragma once

#include "ObjectiveFunction.h"
#include "GradientDescentMinimizer.h"
#include <Eigen/Geometry>

class NewtonFunctionMinimizer : public GradientDescentLineSearch {

public:
    NewtonFunctionMinimizer(int maxIterations = 100, double solveResidual = 0.0001, int maxLineSearchIterations = 15)
        : GradientDescentLineSearch(maxIterations, solveResidual, maxLineSearchIterations) {	}

    virtual ~NewtonFunctionMinimizer() {}

public: 
	virtual void computeSearchDirection(const ObjectiveFunction *function, const VectorXd &x, VectorXd& dx) {
		Solver solver; 
		SparseMatrixd H; function->getHessian(x, H);
		VectorXd g = function->getGradient(x);
		solver.compute(H);
		dx = solver.solve(g); 
		// --
		if (abs(H.toDense().determinant()) < 1e-10) { cout << "Warning: det H ~ 0" << endl; }
    }

public:
    SparseMatrixd hessian;
    std::vector<Triplet<double>> hessianEntries;
    double reg = 1.0;
};
