#pragma once
#include <vector>
#include <glm/glm.hpp>
namespace opengl {
	// Saves current camera, light position, etc
	class CacheManager {
	public:
		// Cached properties, in order.
		glm::vec3 camera_pos;
		double yaw, pitch;
		std::vector<glm::vec3> light_pos_arr;


		bool cache_loaded;
		void Save();

		CacheManager();
		~CacheManager();
	private:
	};
}