#pragma once
#include <Windows.h>
#include <Eigen/Sparse>
// GLEW
#define GLEW_STATIC
#include <GL/glew.h>
#include "simulation.h"
#include "spring_fixed.h"
#include "spring.h"

class Simulation;
enum SpringNodeType {
	vertex, edge, face, constraint,
};
struct DisplayData {
	glm::vec3 pos;
	SpringNodeType type;
	DisplayData(const Vector3 &p, const SpringNodeType t) : pos(p(0), p(1), p(2)), type(t) {
	}
};
class Mesh {
	friend class Simulation;
public:
	Mesh();
	~Mesh();

	// initialization
	void InitNodesAndSprings();

	// spring force
	void ComputeForces(const VectorX &Xi, VectorX &f_out);
	void ComputeGradient(const VectorX &Xi, const VectorX &Y, const double h, VectorX &gradient_output);
	void ComputeHessian(const VectorX &Xi, const double h, MatrixX &hessian_output);

	// ------------------- //
	// * local global
	// construct L matrix
	void ConstructLTriplets(std::vector<Eigen::Triplet<double, int>> &triplets);

	// construct J matrix
	void ConstructJTriplets(std::vector<Eigen::Triplet<double, int>> &triplets);

	// d vector
	void ConstructDVector(const VectorX &Xi, VectorX &d);
	// ------------------- //

	// state update (setter)
	void UpdateState(const VectorX &X_new, const double h);

	// reset
	void Reset();

	// rendering
	void Render();
	void ExportRenderData();
	void CopyRenderDataSafe(std::vector<DisplayData> &output);
	void UpdateVbo(const std::vector<DisplayData> &disp_in);
	glm::mat4 GetModelMatrix();
private:
	// main data : nodes & springs
	// sys_dim = 3 * node_count
	const int node_count, sys_dim;
	int constraint_count;
	std::vector<Constraint*> constraints;

	// states
	VectorX X_default;
	VectorX X, X_last;
	VectorX Vel;

	// inertia matrix
	MatrixX M;
	void InitInertiaMatrix();

	// damping matrix (PMI & IEI)
	MatrixX B;
	void InitDampingMatrix();

	// graphics
	std::vector<DisplayData> disp_data;
	HANDLE mutex_disp_data;
	std::vector<glm::vec3> vbo_data;
	const int vertices_count;
	void ComputeNormal(const glm::vec3 &v1, const glm::vec3 &v2, const glm::vec3 &v3, glm::vec3 &output);
	void BufferDataToVao();
	GLuint vao = 0;
	GLuint vbo;
	unsigned int vbo_size;

	// utility
	SpringNodeType GetNodeType(int r, int c);

};