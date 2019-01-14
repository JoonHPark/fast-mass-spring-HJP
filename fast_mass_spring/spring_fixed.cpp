#include "spring_fixed.h"

SpringFixed::SpringFixed(const int idx, const int n1, const Vector3 &fp, const double k_in)
	: Constraint(idx, n1, k_in, 0.0), fixed_point(fp) {
}
void SpringFixed::Construct_dE(const VectorX &X, VectorX &F) {
	// d vector
	const Vector3 force = -k * (X.segment<3>(end_node1) - fixed_point);

	// spring forces
	F.segment<3>(end_node1) += force;
}
void SpringFixed::Construct_ddE(const VectorX &Xi, std::vector<Eigen::Triplet<double, int>> &ddE_triplets) {
	ddE_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 0, end_node1 + 0, k));
	ddE_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 1, end_node1 + 1, k));
	ddE_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 2, end_node1 + 2, k));
}
void SpringFixed::Construct_L(std::vector<Eigen::Triplet<double, int>> &L_triplets) {
	// (1,1) block
	L_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 0, end_node1 + 0, k));
	L_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 1, end_node1 + 1, k));
	L_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 2, end_node1 + 2, k));
}
void SpringFixed::Construct_J(std::vector<Eigen::Triplet<double, int>> &J_triplets) {
	// (1,1) block
	J_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 0, 3 * constraint_index + 0, k));
	J_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 1, 3 * constraint_index + 1, k));
	J_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 2, 3 * constraint_index + 2, k));
}
void SpringFixed::Construct_DVector(const VectorX &Xi, VectorX &d) {
	d.segment<3>(3 * constraint_index) = fixed_point;
}

//
//
//
//
//
//
//
// non-iterative integration
void SpringFixed::ConstructNoniterative_Mlhs_F(const Integrator integrator, const double dt, const double mass, const double damping, const VectorX &X, MatrixX &Mlhs, VectorX &F) {
	// integrator-dependent variable
	double K = 0;

	if (integrator == PMI) {
		K = k * dt / 2.0;
	}
	else if (integrator == IEI) {
		K = k * dt;
	}
	// Mlhs
	Mlhs.coeffRef(end_node1 + 0, end_node1 + 0) += K;
	Mlhs.coeffRef(end_node1 + 1, end_node1 + 1) += K;
	Mlhs.coeffRef(end_node1 + 2, end_node1 + 2) += K;

	// F
	F.segment<3>(end_node1) += (k * (fixed_point - X.segment<3>(end_node1)));
}
void SpringFixed::PreconstructNoniterative_Mlhs(const Integrator integrator, const double dt, const double mass, const double damping, const VectorX &X, MatrixX &Mlhs) {
	// integrator-dependent variable
	double K = 0;

	if (integrator == PMI) {
		K = k * dt / 2.0;
	}
	else if (integrator == IEI) {
		K = k * dt;
	}
	// Mlhs
	Mlhs.coeffRef(end_node1 + 0, end_node1 + 0) += K;
	Mlhs.coeffRef(end_node1 + 1, end_node1 + 1) += K;
	Mlhs.coeffRef(end_node1 + 2, end_node1 + 2) += K;
}
void SpringFixed::ConstructNoniterative_F(const Integrator integrator, const double dt, const double mass, const double damping, const VectorX &X, VectorX &F) {
	// F
	F.segment<3>(end_node1) += (k * (fixed_point - X.segment<3>(end_node1)));
}