#pragma once

#include <Eigen/Eigen>
#include <BernHelpers.h>

/**
	This class implements the interface for an elementary energy unit. As a function of deformed, undeformed, 
	and other parameters, such as boundary conditions, each class that extends this one will define a potential energy.
	The deformed energy depends on a number of nodes.
*/

class Element {

public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
	Element() {}
	virtual ~Element() {}

	// Returns the number of nodes this unit depends on
	virtual int getNumNodes() const = 0;
	// Returns the global index of node `i`
	virtual int getNodeIndex(int i) const = 0;

	Vector2d nodePos(int i, const VectorXd &x) const {
		return x.segment<2>(2*getNodeIndex(i));
	}

	virtual double energy(const VectorXd& x) const = 0;
	virtual VectorXd gradient(const VectorXd& x) const = 0;
	virtual MatrixXd hessian(const VectorXd& x) const = 0;
};
