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


// PMI
void SpringFixed::ConstructPMI_Mlhs_F(const double dt, const double mass, const double damping, const VectorX &X, std::vector<Eigen::Triplet<double, int>> &Mlhs_triplets, VectorX &F) {
	// Mlhs
	double mhat = mass * 2.0 / dt + damping + k * dt / 2.0;
	Mlhs_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 0, end_node1 + 0, mhat));
	Mlhs_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 1, end_node1 + 1, mhat));
	Mlhs_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 2, end_node1 + 2, mhat));

	// F
	F.segment<3>(end_node1) += (k * (fixed_point - X.segment<3>(end_node1)));
}
