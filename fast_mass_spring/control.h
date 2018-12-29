#pragma once
#include "defines.h"

struct PID {
	double P, I, D;
	PID(const double p_, const double i_, const double d_) : P(p_), I(i_), D(d_) {
	}
};
struct Command {
	int stage;
	std::vector<Vector3> X_targets;
	Vector3 error_prev;
	Vector3 error_sum;
	Command() {
		stage = 0;
		Reset();
	}
	void Reset() {
		error_prev.setZero();
		error_sum.setZero();
	}
};
class Control {
public:
	Control(const size_t size);
	void ToggleStageIndex(const int cmd_idx);
	void AddCommand(const int cmd_idx, const Vector3 &target);
	Vector3 ComputeControlForce(const int cmd_idx, const PID gain, const double h, const Vector3 &Xi);
	void Reset();
private:
	std::vector<Command> commands;
};