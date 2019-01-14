#include "spring.h"

using std::cout;
using std::endl;
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


//
//
//
//
//
//
// non-iterative integraion
void Spring::ConstructNoniterative_Mlhs_F(const Integrator integrator, const double dt, const double mass, const double damping, const VectorX &X, MatrixX &Mlhs, VectorX &F) {
	// integrator-dependent variable
	double K = 0;

	const Vector3 x21 = X.segment<3>(end_node2) - X.segment<3>(end_node1);
	const double x21_norm = sqrt(pow(x21(0), 2) + pow(x21(1), 2) + pow(x21(2), 2));

	// Khat
	const double Khat = k * (1 - r / x21_norm);
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

	Mlhs.coeffRef(end_node1 + 0, end_node2 + 0) += -K;
	Mlhs.coeffRef(end_node1 + 1, end_node2 + 1) += -K;
	Mlhs.coeffRef(end_node1 + 2, end_node2 + 2) += -K;

	Mlhs.coeffRef(end_node2 + 0, end_node1 + 0) += -K;
	Mlhs.coeffRef(end_node2 + 1, end_node1 + 1) += -K;
	Mlhs.coeffRef(end_node2 + 2, end_node1 + 2) += -K;

	Mlhs.coeffRef(end_node2 + 0, end_node2 + 0) += K;
	Mlhs.coeffRef(end_node2 + 1, end_node2 + 1) += K;
	Mlhs.coeffRef(end_node2 + 2, end_node2 + 2) += K;
	
	// F
	const Vector3 force = Khat * x21;
	F.segment<3>(end_node1) += force;
	F.segment<3>(end_node2) -= force;
}
void Spring::PreconstructNoniterative_Mlhs(const Integrator integrator, const double dt, const double mass, const double damping, const VectorX &X, MatrixX &Mlhs) {
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

	Mlhs.coeffRef(end_node1 + 0, end_node2 + 0) += -K;
	Mlhs.coeffRef(end_node1 + 1, end_node2 + 1) += -K;
	Mlhs.coeffRef(end_node1 + 2, end_node2 + 2) += -K;

	Mlhs.coeffRef(end_node2 + 0, end_node1 + 0) += -K;
	Mlhs.coeffRef(end_node2 + 1, end_node1 + 1) += -K;
	Mlhs.coeffRef(end_node2 + 2, end_node1 + 2) += -K;

	Mlhs.coeffRef(end_node2 + 0, end_node2 + 0) += K;
	Mlhs.coeffRef(end_node2 + 1, end_node2 + 1) += K;
	Mlhs.coeffRef(end_node2 + 2, end_node2 + 2) += K;
}
void Spring::ConstructNoniterative_F(const Integrator integrator, const double dt, const double mass, const double damping, const VectorX &X, VectorX &F) {
	// Khat
	const Vector3 x21 = X.segment<3>(end_node2) - X.segment<3>(end_node1);
	const double x21_norm = sqrt(pow(x21(0), 2) + pow(x21(1), 2) + pow(x21(2), 2));
	const double Khat = k * (1 - r / x21_norm);

	// F
	const Vector3 force = Khat * x21;
	F.segment<3>(end_node1) += force;
	F.segment<3>(end_node2) -= force;
}

