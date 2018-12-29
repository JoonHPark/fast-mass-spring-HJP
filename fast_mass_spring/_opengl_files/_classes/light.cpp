#include "light.h"

namespace opengl {
	void Light::Render() {
		glBindVertexArray(vao);
		{
			glDrawArrays(GL_TRIANGLES, 0, vertex_size);
		}
		glBindVertexArray(0);
	}
	Light::Light(glm::vec3 pos, glm::vec3 axis, float ang, glm::vec3 sca, bool ison, std::vector<glm::vec3> &vertices) : Node(pos, axis, ang, sca) {
		is_on = ison;

		// Initialize using vertex data
		glGenVertexArrays(1, &vao);
		glGenBuffers(1, &vbo);

		glBindBuffer(GL_ARRAY_BUFFER, vbo);

		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

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
		vertex_size = vertices.size();
	}


	Light::Light(glm::vec3 pos, glm::vec3 axis, float ang, glm::vec3 sca, bool ison, const char* obj_path)
		:Node(pos, axis, ang, sca) {
		is_on = ison;

		// Initialize using vertex data
		glGenVertexArrays(1, &vao);
		glGenBuffers(1, &vbo);

		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		// Initialize using .obj file
		std::vector<glm::vec3> vertices;
		std::vector<glm::vec3> normals; // Won't be used at the moment.
		bool res = LoadObj(obj_path, vertices, normals);
		std::cout << "   " << vertices.size() << " vertices" << std::endl;
		if (vertices.size() != normals.size()) {
			// Some error in obj file
			return;
		}
		std::vector<glm::vec3> output_vertices;
		for (unsigned int i = 0; i < vertices.size(); i++) {
			output_vertices.push_back(vertices.at(i));
			output_vertices.push_back(normals.at(i));
		}
		glBufferData(GL_ARRAY_BUFFER, output_vertices.size() * sizeof(glm::vec3), &output_vertices[0], GL_STATIC_DRAW);

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
		vertex_size = vertices.size();
	}
}