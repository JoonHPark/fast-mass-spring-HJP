#pragma once
#include "node.h"
namespace opengl {
	class Object : public Node {
	public:
		void Render();
		Object(const glm::vec3 pos, const glm::vec3 axis, const float ang, const glm::vec3 sca, const char* path, std::vector<glm::vec3> &vertices_output);
		Object(const glm::vec3 pos, const glm::vec3 axis, const float ang, const glm::vec3 sca, const char* path);
		Object(const glm::vec3 pos, const glm::vec3 axis, const float ang, const glm::vec3 sca, const std::vector<glm::vec3> &vertices);
	private:
		void InitVaoVbo(const std::vector<glm::vec3> &vertices_in);
	};
}