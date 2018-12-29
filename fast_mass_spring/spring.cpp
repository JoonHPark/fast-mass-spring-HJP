#include "spring.h"

Spring::Spring(const int idx, const int n1, const int n2, const double k_in, const double r_in) 
	: Constraint(idx, n1, k_in, r_in), end_node2(n2) {
}

void Spring::Construct_dE(const VectorX &X, VectorX &F) {
	// d vector
	const Vector3 x1 = X.segment<3>(end_node1);
	const Vector3 x2 = X.segment<3>(end_node2);
	const Vector3 x12 = x1 - x2;
	const Vector3 d12 = r * x12.normalized();
	const Vector3 force = -k * (x1 - x2 - d12);

	// spring forces
	F.segment<3>(end_node1) += force;
	F.segment<3>(end_node2) -= force;
}
void Spring::Construct_L(std::vector<Eigen::Triplet<double, int>> &L_triplets) {
	// (1,1) block
	L_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 0, end_node1 + 0, k));
	L_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 1, end_node1 + 1, k));
	L_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 2, end_node1 + 2, k));
	// (1,2) block
	L_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 0, end_node2 + 0, -k));
	L_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 1, end_node2 + 1, -k));
	L_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 2, end_node2 + 2, -k));
	// (2,1) block
	L_triplets.push_back(Eigen::Triplet<double, int>(end_node2 + 0, end_node1 + 0, -k));
	L_triplets.push_back(Eigen::Triplet<double, int>(end_node2 + 1, end_node1 + 1, -k));
	L_triplets.push_back(Eigen::Triplet<double, int>(end_node2 + 2, end_node1 + 2, -k));
	// (2,2) block
	L_triplets.push_back(Eigen::Triplet<double, int>(end_node2 + 0, end_node2 + 0, k));
	L_triplets.push_back(Eigen::Triplet<double, int>(end_node2 + 1, end_node2 + 1, k));
	L_triplets.push_back(Eigen::Triplet<double, int>(end_node2 + 2, end_node2 + 2, k));
}
void Spring::Construct_ddE(const VectorX &Xi, std::vector<Eigen::Triplet<double, int>> &ddE_triplets) {
	Vector3 x12 = Xi.segment<3>(end_node1) - Xi.segment<3>(end_node2);
	double x12_norm = x12.norm();
	
	Matrix3 I_3x3 = Matrix3::Identity();
	Matrix3 H = k * (I_3x3 - (r / x12_norm) * I_3x3 + r * x12*x12.transpose() / pow(x12_norm, 3));

	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			double h = H(r, c);
			// (1,1) block
			ddE_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + r, end_node1 + c, h));
			// (1,2) block
			ddE_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + r, end_node2 + c, -h));
			// (2,1) block
			ddE_triplets.push_back(Eigen::Triplet<double, int>(end_node2 + r, end_node1 + c, -h));
			// (2,2) block
			ddE_triplets.push_back(Eigen::Triplet<double, int>(end_node2 + r, end_node2 + c, h));
		}
	}
}

void Spring::Construct_J(std::vector<Eigen::Triplet<double, int>> &J_triplets) {
	// (1,1) block
	J_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 0, 3 * constraint_index + 0, k));
	J_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 1, 3 * constraint_index + 1, k));
	J_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 2, 3 * constraint_index + 2, k));
	// (2,2) block
	J_triplets.push_back(Eigen::Triplet<double, int>(end_node2 + 0, 3 * constraint_index + 0, -k));
	J_triplets.push_back(Eigen::Triplet<double, int>(end_node2 + 1, 3 * constraint_index + 1, -k));
	J_triplets.push_back(Eigen::Triplet<double, int>(end_node2 + 2, 3 * constraint_index + 2, -k));
}
void Spring::Construct_DVector(const VectorX &Xi, VectorX &d) {
	Vector3 d12 = r * (Xi.segment<3>(end_node1) - Xi.segment<3>(end_node2)).normalized();
	d.segment<3>(3 * constraint_index) = d12;
}