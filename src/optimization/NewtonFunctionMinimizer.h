    #pragma once

#include "ObjectiveFunction.h"
#include "GradientDescentMinimizer.h"

class NewtonFunctionMinimizer : public GradientDescentLineSearch {
public:
    NewtonFunctionMinimizer(int maxIterations = 100, double solveResidual = 0.0001, int maxLineSearchIterations = 15)
        : GradientDescentLineSearch(maxIterations, solveResidual, maxLineSearchIterations) {	}

    virtual ~NewtonFunctionMinimizer() {}

protected:
    virtual void computeSearchDirection(const ObjectiveFunction *function, const VectorXd &x, VectorXd& dx) {

        // REMOVE FROM HERE ----------------------------------------------------

        // get hessian
        function->hessian(x, hessian);

        // add regularization
        VectorXd r(x.size());
        r.setOnes();
        hessian += r.asDiagonal() * reg;

        // get gradient
        VectorXd gradient(x.size());
        gradient.setZero();
        function->addGradientTo(x, gradient);

        //dp = Hes^-1 * grad
        Eigen::SimplicialLDLT<SparseMatrixd, Eigen::Lower> solver;
        solver.compute(hessian);
        dx = solver.solve(gradient);

        // TO HERE -------------------------------------------------------------
    }

public:
    SparseMatrixd hessian;
    std::vector<Triplet<double>> hessianEntries;
    double reg = 1.0;
};
