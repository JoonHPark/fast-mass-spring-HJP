#pragma once
#include "node.h"

namespace opengl {
	class Light : public Node {
	public:
		bool is_on;
		Data::LightProp prop;

		void Render();
		Light(glm::vec3 pos, glm::vec3 axis, float ang, glm::vec3 sca, bool ison, std::vector<glm::vec3> &vertices);
		Light(glm::vec3 pos, glm::vec3 axis, float ang, glm::vec3 sca, bool ison, const char* path);

		glm::vec3 ambient, diffuse, specular;
		float shiniess;
	};
}