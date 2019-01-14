#include "stdafx.h"
#include "mesh.h"

using std::cout;
using std::endl;
extern int g_fixed_index_1, g_fixed_index_2;
extern VectorX g_gravity_force;

Mesh::Mesh() : node_count(ROW*COL), constraint_count((ROW - 1)*COL + (COL - 1)*ROW), sys_dim(3 * node_count), vertices_count((ROW - 1)*(COL - 1) * 6), mass(MASS), damping(DAMPING_PHYSICAL) {
	mutex_disp_data = CreateMutex(NULL, FALSE, NULL);
	if (mutex_disp_data == NULL) {
		printf("CreateMutex error, mutex_disp_data: %d\n", GetLastError());
		getchar();
	}
	InitNodesAndSprings();
}

Mesh::~Mesh() {
	if (vao != 0) {
		glDeleteVertexArrays(1, &vao);
		glDeleteBuffers(1, &vbo);
	}
}

void Mesh::UpdateState(const VectorX &X_new, const double h) {
	X = X_new;
	Vel = (X - X_last) / h;
}
inline void Mesh::ComputeForces(const VectorX &Xi, VectorX &f_out) {
	// spring force
	f_out.setZero();
	for (std::vector<Constraint*>::iterator c = constraints.begin(); c != constraints.end(); c++) {
		(*c)->Construct_dE(Xi, f_out);
	}
}
void Mesh::ComputeGradient(const VectorX &Xi, const VectorX &Y, const double h, VectorX &gradient_output) {
	// spring force
	VectorX spring_force(sys_dim);
	ComputeForces(Xi, spring_force);

	// output
	gradient_output = M*(Xi - Y) - h * h * (spring_force + g_gravity_force);
}
void Mesh::ComputeHessian(const VectorX &Xi, const double h, MatrixX &hessian_output) {
	MatrixX ddE(sys_dim, sys_dim);
	ddE.setZero();
	std::vector<Eigen::Triplet<double, int>> ddE_triplets;
	ddE_triplets.reserve(constraints.size());

	for (std::vector<Constraint*>::iterator c = constraints.begin(); c != constraints.end(); c++) {
		(*c)->Construct_ddE(Xi, ddE_triplets);
	}
	ddE.setFromTriplets(ddE_triplets.begin(), ddE_triplets.end());

	// output
	hessian_output = M + h * h * ddE;
}
void Mesh::ConstructLTriplets(std::vector<Eigen::Triplet<double, int>> &triplets) {
	for (std::vector<Constraint*>::iterator c = constraints.begin(); c != constraints.end(); c++) {
		(*c)->Construct_L(triplets);
	}
}
void Mesh::ConstructJTriplets(std::vector<Eigen::Triplet<double, int>> &triplets) {
	for (std::vector<Constraint*>::iterator c = constraints.begin(); c != constraints.end(); c++) {
		(*c)->Construct_J(triplets);
	}
}



//
//
//
//
//
//
//
//
//
// non-iterative integration
void Mesh::ConstructNoniterative(const Integrator integrator, const double Tk, MatrixX &Mlhs, MatrixX &Mrhs, VectorX &F) {
	double mhat_rhs = 0;
	if (integrator == PMI) {
		mhat_rhs = mass / Tk * 2.0;
	}
	else if (integrator == IEI) {
		mhat_rhs = mass / Tk;
	}
	// Mrhs
	for (int i = 0; i < sys_dim; i++) {
		Mrhs.coeffRef(i, i) += mhat_rhs;
	}

	// Mlhs
	for (std::vector<Constraint*>::iterator c = constraints.begin(); c != constraints.end(); c++) {
		(*c)->ConstructNoniterative_Mlhs_F(integrator, Tk, mass, damping, X, Mlhs, F);
	}

	double mhat_lhs = mhat_rhs + damping;
	for (int i = 0; i < sys_dim; i++) {
		Mlhs.coeffRef(i, i) += mhat_lhs;
	}
}
void Mesh::ConstructNoniterative_F(const Integrator integrator, const double Tk, VectorX &F) {
	// F
	for (std::vector<Constraint*>::iterator c = constraints.begin(); c != constraints.end(); c++) {
		(*c)->ConstructNoniterative_F(integrator, Tk, mass, damping, X, F);
	}
}
void Mesh::PreconstructNoniterative(const Integrator integrator, const double Tk, MatrixX &Mlhs, MatrixX &Mrhs) {
	double mhat_rhs = 0;
	if (integrator == PMI) {
		mhat_rhs = mass / Tk * 2.0;
	}
	else if (integrator == IEI) {
		mhat_rhs = mass / Tk;
	}
	// Mrhs
	for (int i = 0; i < sys_dim; i++) {
		Mrhs.coeffRef(i, i) += mhat_rhs;
	}

	// Mlhs
	for (std::vector<Constraint*>::iterator c = constraints.begin(); c != constraints.end(); c++) {
		(*c)->PreconstructNoniterative_Mlhs(integrator, Tk, mass, damping, X, Mlhs);
	}

	double mhat_lhs = mhat_rhs + damping;
	for (int i = 0; i < sys_dim; i++) {
		Mlhs.coeffRef(i, i) += mhat_lhs;
	}
}

//
//
//
//
//
//
//
//
//
void Mesh::ConstructDVector(const VectorX &Xi, VectorX &d) {
	Spring *spring;
	Vector3 x1, x2, d12;
	int spring_index = 0;
	for (std::vector<Constraint*>::iterator c = constraints.begin(); c != constraints.end(); c++) {
		(*c)->Construct_DVector(Xi, d);
	}
}
void Mesh::Reset() {
	constraints.clear();
	InitNodesAndSprings();
}
void Mesh::InitInertiaMatrix() {
	// diagonal mass matrix
	const int size = 3 * node_count;
	M.resize(size, size);
	M.setIdentity();
	M *= mass;
}
inline SpringNodeType Mesh::GetNodeType(int r, int c) {
	if ((r == 0 && c == 0) || (r == ROW - 1 && c == COL - 1) || (r == 0 && c == COL - 1) || (r == ROW - 1 && c == 0)) {
		return vertex;
	}
	else if (r == 0 || r == ROW - 1 || c == 0 || c == COL - 1) {
		return edge;
	}
	else {
		return face;
	}
}
void Mesh::InitNodesAndSprings() {
	// initialize nodes and springs
	X.resize(sys_dim);
	X_last.resize(sys_dim);
	Vel.resize(sys_dim);
	X_default.resize(sys_dim);

	X.setZero();
	X_last.setZero();
	Vel.setZero();
	X_default.setZero();

	double length_initial = REST_LENGTH * 2.0;
	double y_initial = 0.0;

	// init nodes
	for (size_t r = 0; r < ROW; r++) {
		for (unsigned int c = 0; c < COL; c++) {
			double x = (r + 1) * length_initial;
			double y = y_initial;
			double z = (COL - c) * length_initial;
			unsigned int idx = 3 * (r * COL + c);
			X.segment<3>(idx) = Vector3(x, y, z);
			X_last.segment<3>(idx) = X.segment<3>(idx);
			X_default.segment<3>(idx) = X.segment<3>(idx);
		}
	}

	// init springs
	constraints.reserve(constraint_count);
	int idx = 0;
	for (unsigned int r = 0; r < ROW; r++) {
		for (unsigned int c = 0; c < COL; c++) {
			int curr_node_idx = 3 * (r * COL + c);
			SpringNodeType type = GetNodeType(r, c);
			Spring *spring1 = NULL;
			Spring *spring2 = NULL;
			if (type == vertex) {
				// spring in +x
				if ((r == 0 && c == 0) || (r == 0 && c == COL - 1)) {
					int node_x_idx = curr_node_idx + 3 * COL;
					spring1 = new Spring(idx++, curr_node_idx, node_x_idx, STIFFNESS, REST_LENGTH);
				}
				// spring in +y
				if ((c == 0 && r == 0) || (c == 0 && r == ROW - 1)) {
					int node_y_idx = curr_node_idx + 3 * 1;
					spring2 = new Spring(idx++, curr_node_idx, node_y_idx, STIFFNESS, REST_LENGTH);
				}
			}
			else if (type == edge) {
				// spring in +x
				if (r != ROW - 1) {
					int node_x_idx = curr_node_idx + 3 * COL;
					spring1 = new Spring(idx++, curr_node_idx, node_x_idx, STIFFNESS, REST_LENGTH);
				}
				// spring in +y
				if (c != COL - 1) {
					int node_y_idx = curr_node_idx + 3 * 1;
					spring2 = new Spring(idx++, curr_node_idx, node_y_idx, STIFFNESS, REST_LENGTH);
				}
			}
			else {
				// face: spring in +x, +y
				int node_x_idx = curr_node_idx + 3 * COL;
				spring1 = new Spring(idx++, curr_node_idx, node_x_idx, STIFFNESS, REST_LENGTH);
				int node_y_idx = curr_node_idx + 3 * 1;
				spring2 = new Spring(idx++, curr_node_idx, node_y_idx, STIFFNESS, REST_LENGTH);
			}
			if (spring1 != NULL) {
				constraints.push_back(spring1);
			}
			if (spring2 != NULL) {
				constraints.push_back(spring2);
			}
		}
	}
	// constraints
	SpringFixed *spring_fixed1 = new SpringFixed(idx++, g_fixed_index_1, X_default.segment<3>(g_fixed_index_1), STIFFNESS_CONSTRAINT);
	SpringFixed *spring_fixed2 = new SpringFixed(idx, g_fixed_index_2, X_default.segment<3>(g_fixed_index_2), STIFFNESS_CONSTRAINT);
	constraints.push_back(spring_fixed1);
	constraints.push_back(spring_fixed2);

	// inertia matrix
	InitInertiaMatrix();

	// results
	constraint_count = constraints.size();
}
// opengl
void Mesh::ExportRenderData() {
	WaitForSingleObject(mutex_disp_data, INFINITE);
	{
		disp_data.clear();
		disp_data.reserve(node_count);
		for (int r = 0; r < ROW; r++) {
			for (int c = 0; c < COL; c++) {
				int idx = r * COL + c;
				SpringNodeType type = GetNodeType(r, c);
				Vector3 pos = X.segment<3>(3 * idx);
				DisplayData dp(pos, type);
				disp_data.push_back(dp);
			}
		}
	}
	ReleaseMutex(mutex_disp_data);
}
void Mesh::CopyRenderDataSafe(std::vector<DisplayData> &output) {
	output.clear();
	output.reserve(node_count);
	WaitForSingleObject(mutex_disp_data, INFINITE);
	{
		for (int r = 0; r < ROW; r++) {
			for (int c = 0; c < COL; c++) {
				int idx = r * COL + c;
				SpringNodeType type = GetNodeType(r, c);
				Vector3 pos = X.segment<3>(3 * idx);
				DisplayData dp(pos, type);
				output.push_back(dp);
			}
		}
	}
	ReleaseMutex(mutex_disp_data);
}
void Mesh::UpdateVbo(const std::vector<DisplayData> &disp_data) {
	vbo_data.clear();
	vbo_data.reserve(vertices_count);

	glm::vec3 v1, v2, v3, v4, n1, n2;
	for (unsigned int r = 0; r < ROW - 1; r++) {
		for (unsigned int c = 0; c < COL - 1; c++) {
			bool r_odd = r % 2;
			bool c_odd = c % 2;

			// indice cw direction
			unsigned int i1 = r * COL + c;
			unsigned int i2 = (r + 1) *COL + c;
			unsigned int i3 = (r + 1) *COL + (c + 1);
			unsigned int i4 = r * COL + (c + 1);

			v1 = disp_data[i1].pos;
			v2 = disp_data[i2].pos;
			v3 = disp_data[i3].pos;
			v4 = disp_data[i4].pos;

			if (r_odd == c_odd) {
				// * 1st triangle: v1, v2, v4
				// normal
				ComputeNormal(v1, v2, v4, n1);
				vbo_data.push_back(v1);
				vbo_data.push_back(n1);

				vbo_data.push_back(v2);
				vbo_data.push_back(n1);

				vbo_data.push_back(v4);
				vbo_data.push_back(n1);

				// * 2nd triangle: v2, v3, v4
				// normal
				ComputeNormal(v2, v3, v4, n2);
				vbo_data.push_back(v2);
				vbo_data.push_back(n2);

				vbo_data.push_back(v3);
				vbo_data.push_back(n2);

				vbo_data.push_back(v4);
				vbo_data.push_back(n2);
			}
			else {
				// * 1st triangle: v1, v2, v3
				// normal
				ComputeNormal(v1, v2, v3, n1);
				vbo_data.push_back(v1);
				vbo_data.push_back(n1);

				vbo_data.push_back(v2);
				vbo_data.push_back(n1);

				vbo_data.push_back(v3);
				vbo_data.push_back(n1);

				// * 2nd triangle: v1, v3, v4
				// normal
				ComputeNormal(v1, v3, v4, n2);
				vbo_data.push_back(v1);
				vbo_data.push_back(n2);

				vbo_data.push_back(v3);
				vbo_data.push_back(n2);

				vbo_data.push_back(v4);
				vbo_data.push_back(n2);
			}
		}
	}

	BufferDataToVao();
}
void Mesh::BufferDataToVao() {
	if (vao == 0) {
		// initialize vao, vbo (runs only once)
		glGenVertexArrays(1, &vao);
		glGenBuffers(1, &vbo);
	}

	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	{
		glBufferData(GL_ARRAY_BUFFER, vbo_data.size() * sizeof(glm::vec3), &vbo_data[0], GL_DYNAMIC_DRAW);

		// Position attribute
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
		glEnableVertexAttribArray(0);

		// Normal attribute
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
		glEnableVertexAttribArray(1);
	}
	glBindVertexArray(0);
	vbo_size = vbo_data.size();
}
void Mesh::Render() {
	glBindVertexArray(vao);
	{
		glDrawArrays(GL_TRIANGLES, 0, vbo_size);
	}
	glBindVertexArray(0);
	
}
void Mesh::ComputeNormal(const glm::vec3 &v1, const glm::vec3 &v2, const glm::vec3 &v3, glm::vec3 &output) {
	// 1st vector: V1 = v1-v2
	glm::vec3 V1 = v1 - v2;
	
	// 2nd vector: V2 = v3-v2
	glm::vec3 V2 = v3- v2;

	// cross product V1 X V2
	output.x = V1.y*V2.z - V2.y*V1.z;
	output.y = V2.x*V1.z - V1.x*V2.z;
	output.z = V1.x*V2.y - V2.x*V1.y;
}
glm::mat4 Mesh::GetModelMatrix() {
	glm::mat4 model_matrix; // Default is Identity Matrix
	return model_matrix;
}