#include "stdafx.h"
#include "simulation.h"
#include <chrono>

// actual dt (currently fixed to h)
static double g_dt = 0.001;
static const double h = 1.0 / MIN_Hz;
DWORD WINAPI MainSimulationThread(LPVOID lpParam);

// externs
extern int g_fixed_index_1, g_fixed_index_2;
extern bool g_toggle_integrator;
extern bool g_do_control;
extern bool g_reset;
extern int g_iteration;
extern Mesh *g_mesh;
extern VectorX g_gravity_force;
extern Integrator g_integrator;

Simulation::Simulation() : I_3x3(Eigen::Matrix3d::Identity()) {
	local_global_first_loop = true;
	ctrl_sec_elpased = 0.0;
	total_sec_elapsed = 0.0;
	frame_count = 0;

	// default pid gains -> can be tuned w/ keyboard
	pid_P = 1.0;
	pid_I = 0.0;
	pid_D = 0.0;

	// gravity vector
	g_gravity_force.resize(g_mesh->sys_dim);
	g_gravity_force.setZero();
	for (int i = 0; i < g_mesh->node_count; i++) {
		g_gravity_force(3 * i + 2) = -MASS * GRAVITY;
	}

	// identity matrix (3m x 3m)
	I_3m3m.resize(g_mesh->sys_dim, g_mesh->sys_dim);
	std::vector<Eigen::Triplet<double, int>> i_triplets;
	for (int i = 0; i < g_mesh->sys_dim; i++) {
		i_triplets.push_back(Eigen::Triplet<double, int>(i, i, 1));
	}
	I_3m3m.setFromTriplets(i_triplets.begin(), i_triplets.end());

	// for node control
	controller = new Control(control_node_count);
	InitControlCommands();

	// only for debugging
	text_output = false;
	text = new Textfile("1_OUTPUT.txt");

	simulation_on = true;

	PrintCurrentSettings(true);
}

Simulation::~Simulation() {
	text->Close();
}

void Simulation::StartSimulationThread() {
	sim_thread = CreateThread(NULL, 0, MainSimulationThread, this, 0, &sim_thread_id);
	SetThreadPriority(&sim_thread, THREAD_PRIORITY_HIGHEST);
}

DWORD WINAPI MainSimulationThread(LPVOID simulation) {
	// pointer to self
	Simulation *self = (Simulation *)simulation;

	// main loop
	bool first_loop = true;
	bool textout = false;

	std::chrono::high_resolution_clock::time_point time_last = std::chrono::high_resolution_clock::now();
	while (1) {
		auto time_curr = std::chrono::high_resolution_clock::now();
		if (!first_loop)
			g_dt = 0.000001 * std::chrono::duration_cast<std::chrono::microseconds>(time_curr - time_last).count();
		else
			first_loop = false;

		time_last = time_curr;

		if (self->simulation_on) {
			if (g_toggle_integrator) {
				self->ToggleIntegrator();
				g_toggle_integrator = false;
				g_reset = true;
			}

			if (g_reset) {
				self->Reset();
				g_reset = false;
			}

			self->UpdateSimulation();
			g_mesh->ExportRenderData();
		}

		// Hz fixer
		double Tk_sim = 0.0;
		while (Tk_sim < MAX_dT_nano) {
			auto time_delta = std::chrono::high_resolution_clock::now();
			Tk_sim = std::chrono::duration_cast<std::chrono::microseconds>(time_delta - time_curr).count();
		}

		// save frame rate
		if (self->text_output) {
			self->text->txt << self->frame_count << "/" << g_dt << "/" << self->total_sec_elapsed / (self->frame_count + 1) << endl;
		}
		self->ctrl_sec_elpased += g_dt;
		self->total_sec_elapsed += g_dt;
		self->frame_count++;
	} // while ends

	return 0;
}

/***************************
main update
****************************/
void Simulation::UpdateSimulation() {
	g_mesh->X_last = g_mesh->X;

	if (g_integrator != PMI && g_integrator != IEI) {
		// iterative
		VectorX X_updated;

		// === 1. State estimate: Y = X + h*b*V ===================== //
		VectorX Y = g_mesh->X + h * (1.0 - DAMPING_NUMERICAL) * g_mesh->Vel;

		// === 2. State update ====================================== //
		switch (g_integrator) {
		case Integrator::GradientDescent:
			GradientDescentUpdate(Y, X_updated);
			break;
		case Integrator::NewtonsMethod:
			NewtonsMethodUpdate(Y, X_updated);
			break;
		case Integrator::LocalGlobal:
			LocalGlobalUpdate(Y, X_updated);
			break;
		}
		// === 3. Velocity update =================================== //
		g_mesh->UpdateState(X_updated, h);
	}
	else {
		// non-iterative
		switch (g_integrator) {
		case Integrator::PMI:
			PMIUpdate(g_mesh->X);
			break;
		case Integrator::IEI:
			break;
		}
	}

	// auxiliary for PID control
	if (g_do_control) {
		if (ctrl_sec_elpased > 0.5) {
			ctrl_sec_elpased = 0.0;
			controller->ToggleStageIndex(one);
			controller->ToggleStageIndex(two);
		}
	}
}

/***************************
1) gradient descent
****************************/
void Simulation::GradientDescentUpdate(const VectorX &Y, VectorX &X_output) {
	// initial condition
	VectorX Xi = Y;

	VectorX gradient(g_mesh->sys_dim);
	for (int n = 0; n < g_iteration; n++) {
		// *** initialize *** //
		gradient.setZero();

		// *** gradient ***** //
		g_mesh->ComputeGradient(Xi, Y, h, gradient);

		// *** controller ***** //
		if (g_do_control) {
			static const PID pid_gain = PID(pid_P, pid_I, pid_D);
			Vector3 f_ctrl_1 = controller->ComputeControlForce(one, pid_gain, h, Xi.segment<3>(g_fixed_index_1));
			Vector3 f_ctrl_2 = controller->ComputeControlForce(two, pid_gain, h, Xi.segment<3>(g_fixed_index_2));
			gradient.segment<3>(g_fixed_index_1) = f_ctrl_1;
			gradient.segment<3>(g_fixed_index_2) = f_ctrl_2;
		}

		// *** update ******* //
		Xi = Xi - STEP_SIZE * gradient;
	} // for ends

	X_output = Xi;
}

/***************************
2) Newton's method
****************************/
void Simulation::NewtonsMethodUpdate(const VectorX &Y, VectorX &X_output) {
	// initial condition
	VectorX Xi = Y;

	VectorX gradient(g_mesh->sys_dim);
	MatrixX hessian(g_mesh->sys_dim, g_mesh->sys_dim);
	for(int n = 0; n < g_iteration; n++) {
		// *** initialize *** //
		gradient.setZero();
		hessian.setZero();

		// *** gradient ***** //
		g_mesh->ComputeGradient(Xi, Y, h, gradient);

		// *** hessian ***** //
		g_mesh->ComputeHessian(Xi, h, hessian);

		// *** controller ***** //
		if (g_do_control) {
			static const PID pid_gain = PID(pid_P, pid_I, pid_D);
			Vector3 f_ctrl_1 = controller->ComputeControlForce(one, pid_gain, h, Xi.segment<3>(g_fixed_index_1));
			Vector3 f_ctrl_2 = controller->ComputeControlForce(two, pid_gain, h, Xi.segment<3>(g_fixed_index_2));
			gradient.segment<3>(g_fixed_index_1) = f_ctrl_1;
			gradient.segment<3>(g_fixed_index_2) = f_ctrl_2;
		}

		// *** inv(hessian) x gradient ***** //
		Eigen::SimplicialLDLT<MatrixX, Eigen::Upper> ldlt_solver;
		FactorizeDirectSolverLDLT(hessian, ldlt_solver);
		VectorX hessianInv_x_gradient = ldlt_solver.solve(gradient);

		// update
		Xi = Xi - STEP_SIZE * hessianInv_x_gradient;
	} // for ends

	X_output = Xi;
}

/***************************
3) block coordinate descent
****************************/
void Simulation::LocalGlobalUpdate(const VectorX &Y, VectorX &X_output) {
	// 1. Precompute & Prefactorize ========================= //
	if (local_global_first_loop) {
		PrefactorizeLocalGlobal();
		local_global_first_loop = false;
	}

	// 2. Fixed iteration =================================== //
	// initial condition
	VectorX Xi = Y;

	VectorX d(3 * g_mesh->constraint_count); // 3s
	VectorX dE(g_mesh->sys_dim); // 3m
	for (int n = 0; n < g_iteration; n++) {
		// *** initialize *** //
		d.setZero();
		dE.setZero();

		// *** d vector *** //
		g_mesh->ConstructDVector(Xi, d);

		// *** external force *** //
		dE += g_gravity_force;

		// *** controller ***** //
		if (g_do_control) {
			// rough
			static const PID pid_gain = PID(pid_P * 0.6, pid_I, pid_D);
			Vector3 f_ctrl_1 = controller->ComputeControlForce(one, pid_gain, h, Xi.segment<3>(g_fixed_index_1));
			Vector3 f_ctrl_2 = controller->ComputeControlForce(two, pid_gain, h, Xi.segment<3>(g_fixed_index_2));
			dE.segment<3>(g_fixed_index_1) = f_ctrl_1 / (h*h);
			dE.segment<3>(g_fixed_index_2) = f_ctrl_2 / (h*h);
		}

		// X = inv(A)*b -> X_i+1 = (M + hhL)^-1 * [My + h*h*(Jd + f_ext)]
		VectorX b = g_mesh->M * Y + h * h * (J * d + dE);

		// *** update *** //
		Xi = llt_solver.solve(b);
	} // for ends

	X_output = Xi;
}
/***************************
4) PMI
****************************/
void Simulation::PMIUpdate(const VectorX &Xi) {
	// *** external force *** //
	// spring force
	VectorX F(g_mesh->sys_dim);
	g_mesh->ComputeForces(Xi, F);

	// gravity
	F += g_gravity_force;

	// controller
	if (g_do_control) {
		static const PID pid_gain = PID(pid_P, pid_I, pid_D);
		Vector3 f_ctrl_1 = controller->ComputeControlForce(one, pid_gain, g_dt, Xi.segment<3>(g_fixed_index_1));
		Vector3 f_ctrl_2 = controller->ComputeControlForce(two, pid_gain, g_dt, Xi.segment<3>(g_fixed_index_2));
		F.segment<3>(g_fixed_index_1) = f_ctrl_1;
		F.segment<3>(g_fixed_index_2) = f_ctrl_2;
	}



	// solve V = inv(A)*B using Eigen 
	MatrixX hessian(g_mesh->sys_dim, g_mesh->sys_dim);
	Eigen::SimplicialLDLT<MatrixX, Eigen::Upper> ldlt_solver;
	FactorizeDirectSolverLDLT(hessian, ldlt_solver);
	VectorX hessianInv_x_gradient = ldlt_solver.solve(gradient);


	// v_{k+1}
	VectorX Vel_next = g_mesh->M_inv*g_dt*(F + g_mesh->M / g_dt * g_mesh->Vel);
	// x_{k+1}
	VectorX X_updated = g_mesh->X + (g_mesh->Vel + Vel_next) / 2.0 * g_dt;

	// update states here
	g_mesh->X = X_updated;
	g_mesh->Vel = Vel_next;
}
/***************************
5) IEI
****************************/
void Simulation::IeiUpdate(const VectorX &Y, VectorX &X_output) {
}


// =================================== //
// factorizations
void Simulation::FactorizeDirectSolverLDLT(const MatrixX& hessian, Eigen::SimplicialLDLT<MatrixX, Eigen::Upper>& ldlt_solver) {
	MatrixX hessian_prime = hessian;
	ldlt_solver.analyzePattern(hessian_prime);
	ldlt_solver.factorize(hessian_prime);
	double regularization =  0.00001;
	while (ldlt_solver.info() != Eigen::Success) {
		regularization *= 10;
		hessian_prime = hessian_prime + regularization * I_3m3m;
		ldlt_solver.factorize(hessian_prime);
	}
}
void Simulation::FactorizeDirectSolverLLT(const MatrixX& MhhL) {
	MatrixX MhhL_prime = MhhL;
	llt_solver.analyzePattern(MhhL_prime);
	llt_solver.factorize(MhhL_prime);
	double Regularization = 0.00001;

	MatrixX I_3m3m;
	I_3m3m.resize(g_mesh->sys_dim, g_mesh->sys_dim);
	I_3m3m.setIdentity();

	while (llt_solver.info() != Eigen::Success)
	{
		Regularization *= 10;
		MhhL_prime = MhhL_prime + Regularization * I_3m3m;
		llt_solver.factorize(MhhL_prime);
	}
}
void Simulation::PrefactorizeLocalGlobal() {
	// L: 3m x 3m
	L.resize(g_mesh->sys_dim, g_mesh->sys_dim);
	std::vector<Eigen::Triplet<double, int>> triplets_L;
	triplets_L.resize(12 * g_mesh->constraint_count);
	g_mesh->ConstructLTriplets(triplets_L);
	L.setFromTriplets(triplets_L.begin(), triplets_L.end());

	// J: 3m x 3s
	J.resize(g_mesh->sys_dim, 3 * g_mesh->constraint_count);
	std::vector<Eigen::Triplet<double, int>> triplets_J;
	triplets_J.resize(6 * g_mesh->constraint_count);
	g_mesh->ConstructJTriplets(triplets_J);
	J.setFromTriplets(triplets_J.begin(), triplets_J.end());

	// prefactorize: Cholesky
	M_hhL = g_mesh->M + h * h * L;
	FactorizeDirectSolverLLT(M_hhL);
}
void Simulation::InitControlCommands() {
	double dx = ROW * REST_LENGTH * 0.25;

	Vector3 initial_pos = g_mesh->X_default.segment<3>(g_fixed_index_1);
	controller->AddCommand(one, initial_pos + Vector3(0, dx, 0));
	controller->AddCommand(one, initial_pos);
	controller->AddCommand(one, initial_pos + Vector3(0, -dx, 0));
	controller->AddCommand(one, initial_pos);

	initial_pos = g_mesh->X_default.segment<3>(g_fixed_index_2);
	controller->AddCommand(two, initial_pos + Vector3(0, -dx, 0));
	controller->AddCommand(two, initial_pos);
	controller->AddCommand(two, initial_pos + Vector3(0, dx, 0));
	controller->AddCommand(two, initial_pos);
}
void Simulation::Reset() {
	local_global_first_loop = true;
	ctrl_sec_elpased = 0.0;
	controller->Reset();
	g_mesh->Reset();
	PrintCurrentSettings(true);
}
void Simulation::ToggleIntegrator() {
	if ((int)g_integrator == (int)LastIndex - 1) {
		g_integrator = (Integrator)0;
	}
	else {
		g_integrator = (Integrator)((int)g_integrator + 1);
	}
}
void Simulation::PrintCurrentSettings(bool print_node_count) {
	if (g_integrator == LocalGlobal) {
		cout << "[1. Local Global] " << endl << "  - " << g_iteration << " iterations." << endl;
	}
	else if (g_integrator == GradientDescent) {
		cout << "[2. Gradient Descent] " << endl << "  - " << g_iteration << " iterations." << endl;
	}
	else if (g_integrator == NewtonsMethod) {
		cout << "[3. Newton's Method] " << endl << "  - " << g_iteration << " iterations." << endl;
	}
	else if (g_integrator == PMI) {
		cout << "[4. Passive Midpoint Integrator] " << endl << "  - non-iterative." << endl;
	}
	else if (g_integrator == IEI) {
		cout << "[5. Implicit Euler Integrator] " << endl << "  - non-iterative." << endl;
	}

	if (print_node_count) {
		cout << "  - " << g_mesh->node_count << " nodes, " << g_mesh->constraint_count << " springs and constraints initialized." << endl;
	}
}