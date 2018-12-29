#include "stdafx.h"
#include "_opengl_files/opengl_manager.h"
#include "simulation.h"
#include "mesh.h"

// callbacks
void KeyboardCallback(GLFWwindow* window, int key, int scancode, int action, int mode);
void MouseCallback(GLFWwindow* window, double pos_x, double pos_y);

// on/off triggers
bool g_toggle_integrator;
bool g_process_mouse;
bool g_do_control;
bool g_mesh_fill;
bool g_reset;

// integrator type
Integrator g_integrator;

// number of iteartions
int g_iteration;

// PID controls
enum GainSelect { P, I, D };
const double d_gain = 0.01;
GainSelect g_gain;

// constraints
int g_fixed_index_1;
int g_fixed_index_2;

// gravity
VectorX g_gravity_force;

// global objects
opengl::OpenglManager *g_opengl_manager;
Simulation *g_sim;
Mesh *g_mesh;

int main(int argc, char** argv) {
	// init some values
	g_toggle_integrator = false;
	g_process_mouse = false;
	g_do_control = true;
	g_mesh_fill = true;
	g_reset = false;
	g_gain = P;
	g_iteration = MAX_ITER;
	g_integrator = PMI;

	g_fixed_index_1 = 0;
	g_fixed_index_2 = 3 * COL * (ROW - 1);

	// opengl setup
	g_opengl_manager = new opengl::OpenglManager();
	g_opengl_manager->SetKeyCallback(KeyboardCallback);
	g_opengl_manager->SetMouseCallback(MouseCallback);

	// important key menus
	cout << endl;
	cout << " --[ Keys ]-------------------------------------------------------------------------" << endl;
	cout << "   1: Toggle integrator mode: Gradient descent -> Newton's method -> Block coordinate descent." << endl;
	cout << "   i: Change number of iterations: 1 -> 10 -> 100" << endl;
	cout << "   e: PID control on/off." << endl;
	cout << "   f: OpenGL: wire <-> fill." << endl;
	cout << "   r: Reset." << endl;
	cout << "   space bar: Enable/disable camera gimbal." << endl;
	cout << "   arrow keys: Move camera." << endl;
	cout << "   0: save current camera/light settings." << endl;
	cout << " -----------------------------------------------------------------------------------" << endl;
	cout << endl;

	// initialization
	g_mesh = new Mesh();
	
	g_sim = new Simulation();
	g_sim->StartSimulationThread();

	// loop holder
	g_opengl_manager->StartRenderLoop();
	return 0;
}

void KeyboardCallback(GLFWwindow* window, int key, int scancode, int action, int mode) {
	if (action == GLFW_PRESS) {
		if (key == GLFW_KEY_SPACE) {
			// Cursor enable / disable
			g_process_mouse = !g_process_mouse;
			g_opengl_manager->ToggleCursorMode(g_process_mouse);

			if (g_process_mouse) {
				cout << ">> Mouse Movement: on" << endl;
			}
			else {
				cout << ">> Mouse Movement: off" << endl;
			}
		}
		else if (key == GLFW_KEY_0) {
			// save opengl cache
			g_opengl_manager->SaveCache();
		}
		else if (key == GLFW_KEY_S) {
		}
		else if (key == GLFW_KEY_R) {
			g_reset = true;
		}
		else if (key == GLFW_KEY_L) {
			g_opengl_manager->ToggleLightSelection();
		}
		else if (key == GLFW_KEY_1) {
			g_toggle_integrator = true;
		}
		else if (key == GLFW_KEY_I) {
			if (g_iteration == 1) {
				g_iteration = 10;
			}
			else if (g_iteration == 10) {
				g_iteration = 100;
			}
			else if (g_iteration == 100) {
				g_iteration = 1;
			}
			g_sim->PrintCurrentSettings(false);
		}
		else if (key == GLFW_KEY_Q) {
			g_opengl_manager->Exit();
			_exit(0);
		}
		else if (key == GLFW_KEY_E) {
			g_do_control = !g_do_control;
			if (g_do_control) {
				g_sim->ctrl_sec_elpased = 0.0;
				cout << ">> Control ON" << endl;
			}
			else {
				cout << ">> Control OFF" << endl;
			}
		}
		else if (key == GLFW_KEY_F) {
			g_mesh_fill = !g_mesh_fill;
			if (g_mesh_fill) {
				cout << ">> Mesh fill ON" << endl;
			}
			else {
				cout << ">> Mesh wire OFF" << endl;
			}
		}
		else if (key == GLFW_KEY_W) {
			g_sim->text_output = !g_sim->text_output;
			if (g_sim->text_output) {
				g_sim->frame_count = 0;
				g_sim->total_sec_elapsed = 0.0;
				cout << ">> Textfile ON" << endl;
			}
			else {
				cout << ">> Textfile OFF" << endl;
			}
		}
		else if (key == GLFW_KEY_U) {
			g_sim->simulation_on = !g_sim->simulation_on;
		}
		// below for debugging
		/*
		else if (key == GLFW_KEY_2) {
			g_gain = P;
			cout << "P selected" << endl;
		} 
		else if (key == GLFW_KEY_3) {
			g_gain = I;
			cout << "I selected" << endl;
		}
		else if (key == GLFW_KEY_4) {
			g_gain = D;
			cout << "D selected" << endl;
		}
		else if (key == GLFW_KEY_Y) {
			if (g_gain == P) {
				g_sim->pid_P += d_gain * 50.0;
				cout << "P = " << g_sim->pid_P << ", I = " << g_sim->pid_I << ", D = " << g_sim->pid_D << endl;
			}
			else if (g_gain == I) {
				g_sim->pid_I += d_gain * 50.0;
			}
			else {
				g_sim->pid_D += d_gain * 50.0;
			}
			cout << "P = " << g_sim->pid_P << ", I = " << g_sim->pid_I << ", D = " << g_sim->pid_D << endl;
		}
		else if (key == GLFW_KEY_T) {
			if (g_gain == P) {
				g_sim->pid_P -= d_gain * 50.0;
			}
			else if (g_gain == I) {
				g_sim->pid_I -= d_gain * 50.0;
			}
			else {
				g_sim->pid_D -= d_gain * 50.0;
			}
			cout << "P = " << g_sim->pid_P << ", I = " << g_sim->pid_I << ", D = " << g_sim->pid_D << endl;
		}
		else if (key == GLFW_KEY_H) {
			if (g_gain == P) {
				g_sim->pid_P += d_gain;
			}
			else if (g_gain == I) {
				g_sim->pid_I += d_gain;
			}
			else {
				g_sim->pid_D += d_gain;
			}
			cout << "P = " << g_sim->pid_P << ", I = " << g_sim->pid_I << ", D = " << g_sim->pid_D << endl;
		}
		else if (key == GLFW_KEY_G) {
			if (g_gain == P) {
				g_sim->pid_P -= d_gain;
			}
			else if (g_gain == I) {
				g_sim->pid_I -= d_gain;
			}
			else {
				g_sim->pid_D -= d_gain;
			}
			cout << "P = " << g_sim->pid_P << ", I = " << g_sim->pid_I << ", D = " << g_sim->pid_D << endl;
		}
		*/
		else if (key == GLFW_KEY_MINUS) {
		}
		else if (key == GLFW_KEY_EQUAL) {
		}
		else if (action == GLFW_RELEASE) {
		}
	}

	// exit
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		g_opengl_manager->Exit();
	}
	else {
		g_opengl_manager->ProcessKey(action, key);
	}
}
void MouseCallback(GLFWwindow* window, const double pos_x, const double pos_y) {
	if (g_process_mouse) g_opengl_manager->ProcessMouse(pos_x, pos_y);
}

