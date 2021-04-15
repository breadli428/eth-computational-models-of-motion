#pragma once

#include "Element.h"

class Triangle : public Element {

public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
	// Triangle() {}
	Triangle(const std::array<int, 3> &nodeIndices, const VectorXd &X) {
		this->nodeIndices = nodeIndices;
		init(X);
	}

public:
	std::array<int, 3> nodeIndices; // the collection of nodes that define the triangle element
	int getNumNodes() const override { return 3; }
	int getNodeIndex(int i) const override { return nodeIndices[i]; }

public:
	Matrix2d dXInv; 
	double restShapeArea = 0.;
	void init(const VectorXd &X){
		//edge vectors
		Vector2d v1 = nodePos(1, X) - nodePos(0, X);
		Vector2d v2 = nodePos(2, X) - nodePos(0, X); 
		Matrix2d dX; //matrix that holds three edge vectors
		dX << v1[0], v2[0],
			  v1[1], v2[1]; 
		dXInv = dX.inverse(); 
		restShapeArea = area(v1, v2);
	}

public: 
	// material parameters
	double shearModulus = 1024.;
	double bulkModulus  = 1024.;

public:
	double energy(const VectorXd& x) const override {
		Matrix2d F = deformationGradient(x);
		double J = F.determinant();
		double mu = shearModulus;
		double kappa = bulkModulus;
		// --
		double energyDensity = 0.0;
		return energyDensity * restShapeArea;
	}

	VectorXd gradient(const VectorXd& x) const override { 
		Matrix2d F = deformationGradient(x);
		double detF = F.determinant();
		Matrix2d dEdF = F * shearModulus + F.inverse().transpose() * (-shearModulus + bulkModulus*log(detF));

		VectorXd gradient; gradient.setZero(6);
		//dF/dx is going to be some +/- Xinv terms. The forces on nodes 1,2 can be writen as: dE/dF * XInv', while the force on node 0 is -f1-f2;
		Matrix2d m = dEdF * dXInv.transpose() * restShapeArea;
		gradient.segment<2>(2) = m.block<2, 1>(0, 0);
		gradient.segment<2>(4) = m.block<2, 1>(0, 1);
		gradient.segment<2>(0) = -m.block<2, 1>(0, 0)-m.block<2, 1>(0, 1);

		return gradient;
	}

	MatrixXd hessian(const VectorXd& x) const override {
		MatrixXd hessian; hessian.setZero(6, 6);

		Matrix2d F = deformationGradient(x);

		Matrix2d Finv = F.inverse();
		Matrix2d FinvT = Finv.transpose();
		Matrix2d dF, dP, tmpM, dH;
		const double dDs[6][4] = { { -1,-1,0,0 },{ 0,0,-1,-1 },{ 1,0,0,0 },{ 0,0,1,0 },{ 0,1,0,0 },{ 0,0,0,1 } };
		for (int i = 0; i < 6; ++i)
		{
			for (int x = 0; x < 4; ++x)
				dF(x / 2, x % 2) = dDs[i][x];
			dF = dF * dXInv;
			double J = F.determinant();
			dP = shearModulus * dF + (shearModulus - bulkModulus * log(J)) * FinvT * dF.transpose() * FinvT;
			tmpM = Finv * dF;
			dP = dP + bulkModulus * (tmpM(0, 0) + tmpM(1, 1)) * FinvT;
			dH = restShapeArea * dP * dXInv.transpose();
			hessian.block<1,2>(i,2) = dH.block<2,1>(0,0).transpose();
			hessian.block<1,2>(i,4) = dH.block<2,1>(0,1).transpose();
			hessian(i, 0 + 0) = -(dH(0, 0) + dH(0, 1));
			hessian(i, 0 + 1) = -(dH(1, 0) + dH(1, 1));
		}

		return hessian;
	}

	double area(const VectorXd &x) const {
		return area(nodePos(1, x) - nodePos(0, x), nodePos(2, x) - nodePos(0, x));
	}

	static double area(const Vector2d &v1, const Vector2d &v2) {
		auto cross2d = [](const Vector2d &a, const Vector2d &b){
			return a[0]*b[1] - a[1]*b[0];
		};
		return 1 / 2.0 * std::abs(cross2d(v1, v2));
	} 

	// as a deformation measure, we need to compute the deformation gradient F. F maps deformed vectors dx to undeformed coords dX: dx = F*dX.
	// for linear basis functions, an easy way to compute it is by looking at the matrix that maps deformed traingle/tet edges to their underformed counterparts (F = dx * inv(dX)).
	Matrix2d deformationGradient(const VectorXd &x) const {
		//edge vectors
		Vector2d v1 = nodePos(1, x) - nodePos(0, x);
		Vector2d v2 = nodePos(2, x) - nodePos(0, x);
		Matrix2d dx;
		dx << v1[0], v2[0],
			  v1[1], v2[1];
		return dx * dXInv;
	}

};
