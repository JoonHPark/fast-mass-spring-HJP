#pragma once
#include <iostream>
#include "defines.h"

struct Constraint {
	// index
	const int constraint_index;
	// index of end-point node
	const int end_node1;
	// stiffness
	const double k;
	// rest length
	const double r;

	Constraint(const int idx_, const int en_, const double k_, const double r_) : constraint_index(idx_), end_node1(en_), k(k_), r(r_) {}

	// virtual functions
	virtual void Construct_dE(const VectorX &X, VectorX &F) { std::cout << "called virtual function." << std::endl; }
	virtual void Construct_ddE(const VectorX &Xi, std::vector<Eigen::Triplet<double, int>> &triplets) { std::cout << "called virtual function." << std::endl; }
	virtual void Construct_L(std::vector<Eigen::Triplet<double, int>> &triplets) { std::cout << "called virtual function." << std::endl; }
	virtual void Construct_J(std::vector<Eigen::Triplet<double, int>> &triplets) { std::cout << "called virtual function." << std::endl; }
	virtual void Construct_DVector(const VectorX &Xi, VectorX &d) { std::cout << "called virtual function." << std::endl; }

	// PMI
	virtual void ConstructPMI_Mlhs_F(const double dt, const double mass, const double damping, const VectorX &X, std::vector<Eigen::Triplet<double, int>> &Mlhs_triplets, VectorX &F) { std::cout << "called virtual function." << std::endl; }
	virtual void ConstructPMI_Mrhs(const double dt, const double mass, std::vector<Eigen::Triplet<double, int>> &Mrhs_triplets) { 
		double mhat = mass * 2.0 / dt;
		Mrhs_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 0, end_node1 + 0, mhat));
		Mrhs_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 1, end_node1 + 1, mhat));
		Mrhs_triplets.push_back(Eigen::Triplet<double, int>(end_node1 + 2, end_node1 + 2, mhat));
		printf("[%d, %d] = %f\n", end_node1, end_node1, mhat);
	}
};