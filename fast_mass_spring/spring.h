#pragma once
#include "constraint.h"

struct Spring : Constraint {
public:
	const int end_node2;
	Spring(const int idx, const int n1, const int n2, const double k_in, const double r_in);

	virtual void Print() {
		printf("[%d]---[%d]\n", end_node1, end_node2);
	};
	// virtual functions
	virtual void Construct_dE(const VectorX &X, VectorX &F);
	virtual void Construct_ddE(const VectorX &Xi, std::vector<Eigen::Triplet<double, int>> &triplets);
	virtual void Construct_L(std::vector<Eigen::Triplet<double, int>> &triplets);
	virtual void Construct_J(std::vector<Eigen::Triplet<double, int>> &triplets);
	virtual void Construct_DVector(const VectorX &Xi, VectorX &d);
	
	// ---------- //
	// Non-iterative integration
	// varying dt
	virtual void ConstructNoniterative_Mlhs_F(const Integrator integrator, const double dt, const double mass, const double damping, const VectorX &X, MatrixX &Mlhs, VectorX &F);
	// fixed dt
	virtual void PreconstructNoniterative_Mlhs(const Integrator integrator, const double dt, const double mass, const double damping, const VectorX &X, MatrixX &Mlhs);
	virtual void ConstructNoniterative_F(const Integrator integrator, const double dt, const double mass, const double damping, const VectorX &X, VectorX &F);
};