#pragma once

#include "ObjectiveFunction.h"
#include "Minimizer.h"

class GradientDescentFixedStep : public Minimizer {
public:
    GradientDescentFixedStep(int maxIterations=100, double solveResidual=1e-5)
        : maxIterations(maxIterations), solveResidual(solveResidual) {
    }

    int getLastIterations() { return lastIterations; }

    virtual bool minimize(const ObjectiveFunction *function, VectorXd &x) {

        bool optimizationConverged = false;

        VectorXd dx(x.size());

        int i=0;
        for(; i < maxIterations; i++) {
            dx.setZero();
            computeSearchDirection(function, x, dx);

            if (dx.norm() < solveResidual){
                optimizationConverged = true;
                break;
            }

            step(function, dx, x);
        }

        lastIterations = i;

        return optimizationConverged;
    }

public:
    virtual void computeSearchDirection(const ObjectiveFunction *function, const VectorXd &x, VectorXd& dx) {
        function->addGradientTo(x, dx);
    }

    // Given the objective `function` and the search direction `x`, update the candidate `x`
    virtual void step(const ObjectiveFunction *function, const VectorXd& dx, VectorXd& x) {
		x = x - stepSize * dx;
    }

public:
    double solveResidual = 1e-5;
    int maxIterations = 1;
    double stepSize = 0.001;

    // some stats about the last time `minimize` was called
    int lastIterations = -1;
};


class GradientDescentLineSearch : public GradientDescentFixedStep {
public:
    GradientDescentLineSearch(int maxIterations=100, double solveResidual=1e-5, int maxLineSearchIterations=15)
        : GradientDescentFixedStep (maxIterations, solveResidual), maxLineSearchIterations(maxLineSearchIterations){
    }

public:
    virtual void step(const ObjectiveFunction *function, const VectorXd& dx, VectorXd& x)
    {
		double alpha_nominal = 1.;
		double O0 = function->evaluate(x);
		VectorXd x_cand = x;
		for (int i = 0; i < maxLineSearchIterations; ++i) {
			double alpha = alpha_nominal * pow(.5, i);
			x_cand = x - alpha * dx;
			if (function->evaluate(x_cand) < O0) {
				x = x_cand;
				return;
			}
		}
    }

public:
    int maxLineSearchIterations = 15;
};
