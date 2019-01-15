#pragma once
#include <glm/glm.hpp>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Windows.h>
#include "defines.h"
#include "textfile.h"
#include "constraint.h"
#include "mesh.h"
#include "control.h"

enum ControlNodeIndex {
	one, two, control_node_count
};

class Simulation {
public:
	bool simulation_on;

	// frame rate
	unsigned int frame_count;
	double total_sec_elapsed;

	// total time elapsed
	double ctrl_sec_elpased;

	// initialization
	Simulation();
	~Simulation();
	
	// thread starter
	void StartSimulationThread();

	// textfile - for debugging
	Textfile *text;
	bool text_output;
	
	// PID gain
	double pid_P, pid_I, pid_D;

	// cout current settings
	void PrintCurrentSettings(bool print_node_count);
private:
	// =========================== //
	// MAIN SIMULATION FUNCTIONS
	// 1) gradient descent
	void GradientDescentUpdate(const double Tk, const VectorX &Y, VectorX &X_output);
	// 2) newton's method
	void NewtonsMethodUpdate(const double Tk, const VectorX &Y, VectorX &X_output);
	void FactorizeDirectSolverLDLT(const MatrixX& A, Eigen::SimplicialLDLT<MatrixX, Eigen::Upper>& ldlt_solver);
	// 3) block coordinate descent
	void LocalGlobalUpdate(const double Tk, const VectorX &Y, VectorX &X_output);
	void FactorizeDirectSolverLLT(const MatrixX& MhhL, Eigen::SimplicialLLT<MatrixX, Eigen::Upper> &llt_solver);
	void PrefactorizeLocalGlobal();
	MatrixX M_hhL, L, J;
	bool local_global_first_loop;
	Eigen::SimplicialLLT<MatrixX, Eigen::Upper> llt_solver_localglobal;
	// =========================== //
	// NON-ITERATIVE METHODS
	// 4) pmi, iei
	void NoniterativeUpdate_FixedTimestep(const double Tk, const VectorX &Xi);
	void PrefactorizeNoniterative(Integrator integrator);
	MatrixX Mlhs_pmi, Mrhs_pmi, Mlhs_iei, Mrhs_iei;
	bool pmi_first_loop, iei_first_loop;
	Eigen::SimplicialLLT<MatrixX, Eigen::Upper> llt_solver_pmi, llt_solver_iei;
	void NoniterativeUpdate_VaryingTimestep(const double Tk, const VectorX &Xi);
	// =========================== //


	// controller for end nodes
	Control *controller;
	void InitControlCommands();

	// reset simulation
	void Reset();

	// simulation thread
	HANDLE sim_thread;
	DWORD sim_thread_id;
	friend DWORD WINAPI MainSimulationThread(LPVOID lpParam);
	void UpdateSimulation();

	// utility
	const Matrix3 I_3x3;
	MatrixX I_3m3m;
	void ToggleIntegrator();
};