#include <fstream>
#include <sstream>
#include <iostream>

#include "cache_manager.h"

using std::cout;
using std::endl;
namespace opengl {
	CacheManager::CacheManager() {
		cache_loaded = false;

		// load cache if exists
		std::string line;
		std::ifstream text_file("opengl_cache.txt");

		int index = 0;
		glm::vec3 light_pos;
		if (text_file.is_open())
		{
			while (std::getline(text_file, line))
			{
				switch (index) {
				case 0:
					camera_pos.x = stof(line); break;
				case 1:
					camera_pos.y = stof(line); break;
				case 2:
					camera_pos.z = stof(line); break;
				case 3:
					yaw = stof(line); break;
				case 4:
					pitch = stof(line); break;
				default:
					int rem = (index - 5) % 3;
					if (rem == 0) {
						light_pos.x = stof(line);
					}
					else if (rem == 1) {
						light_pos.y = stof(line);
					}
					else {
						light_pos.z = stof(line);
						light_pos_arr.push_back(light_pos);
					}
					break;
				}
				index++;
			}
		}

		if (index > 0) {
			cout << "OpenGL Cache loaded. Camera: [" << camera_pos.x << ", " << camera_pos.y << ", " << camera_pos.z << "]" << endl;
			cache_loaded = true;
			text_file.close();
		}
		else {
			cout << "No OpenGL Cache exists." << endl;
		}
	}
	CacheManager::~CacheManager() {
		Save();
	}

	void CacheManager::Save() {
		std::ofstream textFile("opengl_cache.txt", std::ios::out | std::ios::trunc);

		textFile << camera_pos.x << '\n' << camera_pos.y << '\n' << camera_pos.z << '\n';
		textFile << yaw << '\n' << pitch << '\n';
		for (int i = 0; i < light_pos_arr.size(); i++) {
			textFile << light_pos_arr[i].x << '\n' << light_pos_arr[i].y << '\n' << light_pos_arr[i].z << '\n';
		}

		textFile.close();
	}
}