    #pragma once

#include "ObjectiveFunction.h"
#include "GradientDescentMinimizer.h"

class NewtonFunctionMinimizer : public GradientDescentLineSearch {

public:
    NewtonFunctionMinimizer(int maxIterations = 100, double solveResidual = 0.0001, int maxLineSearchIterations = 15)
        : GradientDescentLineSearch(maxIterations, solveResidual, maxLineSearchIterations) {	}

    virtual ~NewtonFunctionMinimizer() {}

public: 
	Solver solver; 
	int nMaxStabSteps = 10;
	double stabValue = 1e-4;
	virtual void computeSearchDirection(const ObjectiveFunction *function, const VectorXd &x, VectorXd& dx) {
		SparseMatrixd H; function->getHessian(x, H);
		VectorXd g = function->getGradient(x);
		solver.compute(H);
		dx = solver.solve(g); 
		if (dx.dot(g) < 0) {
			double currStabValue = stabValue;
			for (int i = 0; i < nMaxStabSteps; ++i) {
				for (int j = 0; j < x.size(); ++j) { H.coeffRef(j, j) += currStabValue; }
				currStabValue *= 10;
				solver.compute(H);
				dx = solver.solve(g); 
				if (dx.dot(g) > 0) { break; }
			}
		}
    }

public:
    SparseMatrixd hessian;
    std::vector<Triplet<double>> hessianEntries;
    double reg = 1.0;
};
