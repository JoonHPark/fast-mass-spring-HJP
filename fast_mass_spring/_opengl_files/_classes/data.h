#pragma once
#include <glm/glm.hpp>
#define NUM_OF_LIGHTS 3 // 0: PointLight0, 1: PointLight1, 2: DirectionalLight
namespace opengl {
	class Data {
	public:
		struct Material {
			glm::vec3 ambient;
			glm::vec3 diffuse;
			glm::vec3 specular;
			float shininess;
		};

		struct DataStruct {
			glm::vec3 position;
			glm::vec3 scale;
			glm::vec3 axis;
			float angle;
			const char* obj_path;

			glm::vec3 ambient;
			glm::vec3 diffuse;
			glm::vec3 specular;
			float shininess;
		};

		struct LightProp {
			glm::vec3 ambient;
			glm::vec3 diffuse;
			glm::vec3 specular;
			float constant, linear, quadratic;
		};
		Data() {
			camera_position = glm::vec3(6, 11, 5);

			light_positions[0] = glm::vec3(3.f, 8.f, 8.f); // Point Light 0
			light_positions[1] = glm::vec3(8.f, 0.f, 7.f); // Point Light 1
			light_positions[2] = glm::vec3(7.f, -7.f, 4.f); // Diretional Light};

			//glm::vec3 dirLightPosition = { 3.f, 3.f, 3.f }; // for shadowmapping
			light_direction = glm::vec3(-1.f, -1.f, -1.f);

			light_props = new LightProp[NUM_OF_LIGHTS]{
				LightProp{ 0.08f*glm::vec3(1), 0.8f*glm::vec3(1), 1.f*glm::vec3(1), 0.01f, 0.02f, 0.0009f },
				LightProp{ 0.08f*glm::vec3(1), 0.8f*glm::vec3(1), 1.f*glm::vec3(1), 0.01f, 0.02f, 0.0009f },
				LightProp{ 0.2f*glm::vec3(1), 0.4f*glm::vec3(1), 1.f*glm::vec3(1), 0.01f, 0.02f, 0.0009f },
			};


			emerald_green = Material{ glm::vec3(0.0215f, 0.1745f, 0.0215f),glm::vec3(0.07568f, 0.61424f, 0.07568f),glm::vec3(0.633f, 0.727811f, 0.633f),0.6f };
			murky_cyon = Material{ glm::vec3(0.135f, 0.2225f, 0.1575f),glm::vec3(0.54f, 0.89f, 0.63f),glm::vec3(0.316228f, 0.316228f, 0.316228f),0.6f };
			blackish = Material{ glm::vec3(0.05375f, 0.05f, 0.06625f),glm::vec3(0.18275f, 0.17f, 0.22525f),glm::vec3(0.332741f, 0.328634f, 0.346435f),0.3f };
			pearl = Material{ glm::vec3(0.25f, 0.20725f, 0.20725f),glm::vec3(1.f, 0.829f, 0.829f),glm::vec3(0.296648f, 0.296648f, 0.296648f),0.088f };
			wine = Material{ glm::vec3(0.1745f, 0.01175f, 0.01175f),glm::vec3(0.61424f, 0.04136f, 0.04136f),glm::vec3(0.727811f, 0.626959f, 0.626959f),0.6f };
			skyblue = Material{ glm::vec3(0.1f, 0.18725f, 0.1745f),glm::vec3(0.396f, 0.74151f, 0.69102f),glm::vec3(0.297254f, 0.30829f, 0.306678f),0.1f };
			goldish_yellow = Material{ glm::vec3(0.329412f, 0.223529f, 0.027451f),glm::vec3(0.780392f, 0.568627f, 0.113725f),glm::vec3(0.992157f, 0.941176f, 0.807843f),0.21794872f };
			bronze = Material{ glm::vec3(0.2125f, 0.1275f, 0.054f),glm::vec3(0.714f, 0.4284f, 0.18144f),glm::vec3(0.393548f, 0.271906f, 0.166721f),0.2f };
			gray = Material{ glm::vec3(0.25f, 0.25f, 0.25f),glm::vec3(0.4f, 0.4f, 0.4f),glm::vec3(0.774597f, 0.774597f, 0.774597f),0.6f };
			dark_orange = Material{ glm::vec3(0.19125f, 0.0735f, 0.0225f),glm::vec3(0.7038f, 0.27048f, 0.0828f),glm::vec3(0.256777f, 0.137622f, 0.086014f),0.1f };
			gold = Material{ glm::vec3(0.24725f, 0.1995f, 0.0745f),glm::vec3(0.75164f, 0.60648f, 0.22648f),glm::vec3(0.628281f, 0.555802f, 0.366065f),0.4f };
			silver = Material{ glm::vec3(0.19225f, 0.19225f, 0.19225f),glm::vec3(0.50754f, 0.50754f, 0.50754f),glm::vec3(0.508273f, 0.508273f, 0.508273f),0.4f };
			whitish = Material{ 5.f*glm::vec3(1), 5.f*glm::vec3(1), 5.f*glm::vec3(1),20.f };
			bluish = Material{ 0.5f*glm::vec3(1), 1.f*glm::vec3(0, 0, 0.5f), 1.f*glm::vec3(1), 0.99f };
			yellowish = Material{ 0.5f*glm::vec3(1, 1, 0), 1.f*glm::vec3(1, 1, 0), 1.f*glm::vec3(1, 1, 0), 0.99f };
			cyon = Material{ 0.5f*glm::vec3(0, 0.8, 1), 1.f*glm::vec3(0, 0.8, 1), 1.f*glm::vec3(0, 0.8, 1), 0.99f };
			orange = Material{ 0.5f*glm::vec3(1, 0.5, 0), 1.f*glm::vec3(1, 0.5, 0), 1.f*glm::vec3(1, 0.5, 0), 0.99f };
			redMat = Material{ 0.5f*red, 1.f*red, 1.f*red, 0.99f };
			floor_blue = Material{ 0.8f*glm::vec3(1), 1.5f*glm::vec3(145 / 255.f, 200 / 255.f, 240 / 255.f), 1.f*glm::vec3(1), 0.99f };

			stanford_bunny_data = DataStruct{
				glm::vec3(0.f, 0.f, 0.f),
				glm::vec3(0.4f, 0.4f, 0.4f),
				glm::vec3(1.f, 0.f, 0.f),
				90.f,
				"_opengl_files/_models/bunny.obj",
				pearl.ambient,
				pearl.diffuse,
				pearl.specular,
				pearl.shininess
			};

			utah_teapot_data = DataStruct{
				glm::vec3(0.f, 0.f, 1.f),
				glm::vec3(1.f, 1.f, 1.f),
				glm::vec3(0.f, 0.f, 1.f),
				0.f,
				"_opengl_files/_models/cube.obj",
				whitish.ambient,
				whitish.diffuse,
				whitish.specular,
				whitish.shininess
			};

			cube_data = DataStruct{
				glm::vec3(2.f, -2.f, 0.f),
				glm::vec3(0.05f),
				glm::vec3(0.f, 0.f, 1.f),
				0.f,
				"_opengl_files/_models/cube.obj",
				glm::vec3(0.3f),
				glm::vec3(0.1f),
				glm::vec3(1.f),
				10.f
			};

			floor_data = DataStruct{
				glm::vec3(0.f, 0.f, -20.f),
				glm::vec3(1000.f, 1000.f, 0.1f),
				glm::vec3(0.f, 0.f, 1.f),
				0.f,
				"_opengl_files/_models/cube.obj",

				floor_blue.ambient,
				floor_blue.diffuse,
				floor_blue.specular,
				floor_blue.shininess
			};
		}

		glm::vec3 light_positions[NUM_OF_LIGHTS];
		glm::vec3 camera_position;

		LightProp *light_props;

		Material emerald_green;
		Material murky_cyon;
		Material blackish;
		Material pearl;
		Material wine;
		Material skyblue;
		Material goldish_yellow;
		Material bronze;
		Material gray;
		Material dark_orange;
		Material gold;
		Material silver;
		Material whitish;
		Material bluish;
		Material yellowish;
		Material cyon;
		Material orange;
		Material redMat;
		Material floor_blue;


		DataStruct floor_data;

		DataStruct stanford_bunny_data;
		DataStruct utah_teapot_data;
		DataStruct cube_data;


		glm::vec3 red = glm::vec3(1, 0, 0);
		glm::vec3 green = glm::vec3(0, 1, 0);
		glm::vec3 blue = glm::vec3(0, 0, 1);

		glm::vec3 light_direction;



		~Data() {
			delete[] light_props;
		}
	};
}