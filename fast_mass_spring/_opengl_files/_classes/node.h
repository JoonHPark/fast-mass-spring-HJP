#pragma warning(disable:4996)

#define _CRT_SECURE_NO_WARNINGS
#pragma once
#define MAX_ARRAY_SIZE 10000000
#include <iostream>
#include <vector>
// GLEW
#define GLEW_STATIC
#include <GL/glew.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "data.h"

namespace opengl {
	class Node {
	public:
		glm::vec3 position;
		glm::vec3 rotation_axis;
		glm::vec3 scale;
		float angle;

		int vertex_size;

		GLuint vao = 0;
		GLuint vbo;

		glm::vec3 ambient;
		glm::vec3 diffuse;
		glm::vec3 specular;
		float shininess;

		Node(const glm::vec3 pos, const glm::vec3 axis, const  float ang, const glm::vec3 sca);
		~Node();
		virtual void Render() = 0;

		void SetMaterial(const glm::vec3 a, const glm::vec3 d, const glm::vec3 s, const float shine);
		glm::mat4 GetModelMatrix();
		bool LoadObj(const char * path, std::vector<glm::vec3> & out_vertices, std::vector<glm::vec3> & out_normals);
	};
}