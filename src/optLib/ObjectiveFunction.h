#pragma once 
#include <BernHelpers.h>

class ObjectiveFunction{

public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    // this should always return the current value of the objective function
    virtual double evaluate(const VectorXd& x) const = 0;

	VectorXd u_reg; // You can use this to implement regularizers (nothing is handled for you)

	bool checkGradient = false;
    VectorXd getGradient(const VectorXd &x) const {
        VectorXd gradient; gradient.setZero(x.size());
        addGradientTo(x, gradient);
		if (checkGradient) {
			VectorXd gradient_FD; gradient_FD.setZero(x.size());
			addFiniteDifferenceGradientTo(x, gradient_FD);
			if (checkVectorXd(gradient_FD, gradient)) { cout << "Gradient checks." << endl; }
		} 
        return gradient;
    }
	
	bool checkHessian = false;
    void getHessian(const VectorXd &x, SparseMatrixd &hessian) const {
		vector<Triplet<double>> triplets;
		addHessianEntriesTo(x, triplets);
		hessian = hessianHelper_(triplets, x);
		if (checkHessian) {
			vector<Triplet<double>> triplets_FD;
			addFiniteDifferenceHessianEntriesTo(x, triplets_FD);
			SparseMatrixd hessian_FD = hessianHelper_(triplets_FD, x);
			if (checkMatrixXd(hessian_FD.toDense(), hessian.toDense())) { cout << "Hessian checks." << endl; }
		} 
    } 

    virtual void addGradientTo(const VectorXd& x, VectorXd& grad) const {
        addFiniteDifferenceGradientTo(x, grad);
    }

    virtual void addHessianEntriesTo(const VectorXd& x, std::vector<Triplet<double>>& hessianEntries) const {
        addFiniteDifferenceHessianEntriesTo(x, hessianEntries);
    }

    void addFiniteDifferenceGradientTo(const VectorXd& x, VectorXd& grad) const {
        const double h = 1e-5;
        for (int i = 0; i < x.size(); ++i) {
            VectorXd dx(x.size());
            dx.setZero();
            dx[i] = h;
            grad[i] += (evaluate(x + dx) - evaluate(x - dx)) / (2.*h);
        }
    }

    void addFiniteDifferenceHessianEntriesTo(const VectorXd& x, std::vector<Triplet<double>>& hessianEntries) const {
        const double h = 1e-6;
        for (int i = 0; i < x.size(); ++i) {
            VectorXd dx(x.size());
            dx.setZero();
            dx[i] = h;

            VectorXd gp(x.size()), gm(x.size());
            gp.setZero(); gm.setZero();
            addGradientTo(x + dx, gp);
            addGradientTo(x - dx, gm);

            VectorXd hess = (gp - gm) / (2.*h);
            for (int j = 0; j < x.size(); ++j) { // *
                if(abs(hess[j]) > 1e-12 || i == j)
                    hessianEntries.push_back(Triplet<double>(i, j, hess[j]));
            }
        }
    }
	
private: 
	SparseMatrixd hessianHelper_(const vector<Triplet<double>> &triplets, const VectorXd &x) const {
		SparseMatrixd hessian;
        hessian.resize(x.size(), x.size());
        hessian.setFromTriplets(triplets.begin(), triplets.end());
		return hessian; 
	}

};

