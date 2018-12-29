#include "control.h"
#include <iostream>

using std::cout;
using std::endl;
Control::Control(const size_t size) {
	for (int i = 0; i < size; i++) {
		commands.push_back(Command());
	}
}
void Control::AddCommand(const int stage, const Vector3 &target) {
	if (stage >= commands.size()) {
		std::cout << "command size error." << std::endl;
		return;
	}
	commands[stage].X_targets.push_back(target);
}

Vector3 Control::ComputeControlForce(const int cmd_idx, const PID gain, const double h, const Vector3 &Xi) {
	if (cmd_idx >= commands.size()) {
		std::cout << "command size error." << std::endl;
		return Vector3::Zero();
	}

	Command *command = &commands[cmd_idx];
	Vector3 X_target = command->X_targets[command->stage];
	Vector3 e = Xi - X_target;
	Vector3 de = (e - command->error_prev) / h;
	Vector3 e_sum = command->error_sum + e;
	Vector3 F_ctrl = gain.P * e + gain.I * e_sum + gain.D * de;

	// save error history
	command->error_prev = e;
	command->error_sum = e_sum;

	return F_ctrl;
}
void Control::Reset() {
	for (std::vector<Command>::iterator c = commands.begin(); c != commands.end(); c++) {
		c->Reset();
		c->stage = 0;
	}
}
void Control::ToggleStageIndex(const int cmd_idx) {
	if (cmd_idx >= commands.size()) {
		std::cout << "command size error." << std::endl;
		return;
	}
	int stage = commands[cmd_idx].stage;
	commands[cmd_idx].stage = (stage + 1) % commands[cmd_idx].X_targets.size();
	commands[cmd_idx].Reset();
}