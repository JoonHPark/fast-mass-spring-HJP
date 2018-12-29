#include "object.h"
namespace opengl {
	void Object::Render() {
		glBindVertexArray(vao);
		{
			glDrawArrays(GL_TRIANGLES, 0, vertex_size);
		}
		glBindVertexArray(0);
	}
	Object::Object(const glm::vec3 pos, const glm::vec3 axis, const float ang, const glm::vec3 sca, const char* obj_path) : Node(pos, axis, ang, sca) {
		// Initialize using .obj file
		std::vector<glm::vec3> vertices;
		std::vector<glm::vec3> normals; // Won't be used at the moment.
		bool res = LoadObj(obj_path, vertices, normals);
		//std::cout << "   " << vertices.size() << " vertices" << std::endl;

		if (vertices.size() != normals.size()) {
			std::cout << "ERROR: vertex count and normal count don't match! Check if .obj file is correct" << std::endl;
			getchar();
		}

		std::vector<glm::vec3> vertices_out;
		vertices_out.reserve(MAX_ARRAY_SIZE);
		for (unsigned int i = 0; i < vertices.size(); i++) {
			vertices_out.push_back(vertices[i]);
			vertices_out.push_back(normals[i]);
		}
		vertices_out.shrink_to_fit();

		InitVaoVbo(vertices_out);
	}
	Object::Object(const glm::vec3 pos, const glm::vec3 axis, const float ang, const glm::vec3 sca, const char* obj_path, std::vector<glm::vec3> &vertices_out) : Node(pos, axis, ang, sca) {
		// Initialize using .obj file
		std::vector<glm::vec3> vertices;
		std::vector<glm::vec3> normals; // Won't be used at the moment.
		bool res = LoadObj(obj_path, vertices, normals);
		std::cout << "   " << vertices.size() << " vertices" << std::endl;
		if (vertices.size() != normals.size()) {
			std::cout << "ERROR: vertex count and normal count don't match! Check if .obj file is correct" << std::endl;
			getchar();
		}

		vertices_out.clear();
		vertices_out.reserve(MAX_ARRAY_SIZE);
		for (unsigned int i = 0; i < vertices.size(); i++) {
			vertices_out.push_back(vertices[i]);
			vertices_out.push_back(normals[i]);
		}
		vertices_out.shrink_to_fit();

		InitVaoVbo(vertices_out);
	}
	Object::Object(const glm::vec3 pos, const glm::vec3 axis, const float ang, const glm::vec3 sca, const std::vector<glm::vec3> &vertices_in) : Node(pos, axis, ang, sca) {
		InitVaoVbo(vertices_in);
	}

	void Object::InitVaoVbo(const std::vector<glm::vec3> &vertices_in) {
		// Initialize using vertex data
		glGenVertexArrays(1, &vao);
		glGenBuffers(1, &vbo);

		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, vertices_in.size() * sizeof(glm::vec3), &vertices_in[0], GL_STATIC_DRAW);

		glBindVertexArray(vao);
		{
			// Position attribute
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
			glEnableVertexAttribArray(0);

			// Normal attribute
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
			glEnableVertexAttribArray(1);
		}
		glBindVertexArray(0);

		vertex_size = vertices_in.size();
	}
}