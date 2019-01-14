#pragma once
#include "constraint.h"

struct SpringFixed : Constraint {
public:
	const Vector3 fixed_point;

	SpringFixed(const int idx, const int n1, const Vector3 &fp, const double k_in);

	virtual void Print() {
		printf("[%d] fixed\n", end_node1);
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

