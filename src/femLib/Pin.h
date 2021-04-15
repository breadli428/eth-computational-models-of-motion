#pragma once

#include "Element.h"

class Pin : public Element {

public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
	int nodeIndex;           // Which node is being pinned
	Vector2d anchorPosition; // p': Where the node is anchored to
	double k;                // Spring constant

	Pin(int nodeIndex, const Vector2d &anchorPosition, double k = 1)
		: nodeIndex(nodeIndex), anchorPosition(anchorPosition), k(k) { } 

	// p = p(x)
	Vector2d getPosition(const VectorXd &x) const {
		// NOTE: x is the position of all nodes in the mesh stacked into a column.
		// + The i-th 2-segment of x(u) is the position of the i-th node 
		return seg2(x, nodeIndex); // NOTE: shorthand for x.segment<2>(2 * nodeIndex)
	}

	// E(p) = .5 k ||p - p'|| ^ 2
	// HINT: Vector2d has a method called squaredNorm()
	double energy(const VectorXd &x) const override {
		double E = 0.;
		// TODO: ...
		return E;
	}

	// dEdp = ???
	virtual VectorXd gradient(const VectorXd &x) const override {
	    VectorXd dEdp; dEdp.setZero(2);
		// TODO: ...
	    return dEdp;
	}

	// d2Edp2 = ???
	virtual MatrixXd hessian(const VectorXd &) const override {
	    MatrixXd d2Edp2; d2Edp2.setZero(2, 2);
		// TODO: ...
		return d2Edp2;
	}

public:
	int getNumNodes() const override { return 1; }
	int getNodeIndex(int) const override { return nodeIndex; }

public:
	int handleIndex = -1;

};
