#pragma once
// GLEW
#define GLEW_STATIC
#include <GL/glew.h>
// GLFW
#include <GLFW/glfw3.h>

// GLM Mathemtics
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "_classes/object.h"
#include "_classes/light.h"
#include "_classes/camera.h"
#include "_classes/cache_manager.h"

// Defines
//#define DISP_4k
#ifdef DISP_4k
// If 4K display
#define WINDOW_WIDTH 2560
#define WINDOW_HEIGHT 1440
#else
// If not 4K display
#define WINDOW_WIDTH 1366
#define WINDOW_HEIGHT 768  
#endif

#define DEPTHMAP_WIDTH 1024
#define DEPTHMAP_HEIGHT 1024
#define NEAR_PLANE 0.1f
#define FAR_PLANE 1000.f
#define FIELD_OF_VIEW 90.0f

#define LIGHT_SPEED 0.1f
#define CAM_POS 10.f
#define AXIS_LENGTH 20.f
#define AXIS_SCALE 0.0025f

static GLFWwindow * g_window;
namespace opengl {
	class OpenglManager {
	public:

		OpenglManager();
		~OpenglManager();

		void StartRenderLoop();

		void ProcessKey(const int action, const int key);
		void ProcessMouse(const double xpos, const double ypos);
		void SetKeyCallback(void(*keyCallback)(GLFWwindow* window, const int key, const int scancode, const int action, const int mode));
		void SetMouseCallback(void(*mouseCallback)(GLFWwindow* window, const double xpos, const double ypos));
		void ToggleCursorMode(const bool toggle);
		void ToggleLightSelection();

		void SaveCache();

		void Exit();
	private:
		Data data;
		CacheManager cacheManager;

		Camera* camera;
		// Moving lights
		int selected_light_idx; // selected light index
		Light *selected_light;

		// Floor object to cast shadow on.
		Object *floor;

		// Lights: 0 = point light0, 1 = point light1, 2 = directional light
		std::vector<Light> lights;

		// Objects to render
		Object *cube;
		std::vector<Object> objects;

		// FBO and Textures
		std::vector<GLuint> depthmap_buffers;
		std::vector<GLuint> depthmap_textures;

		// Shader programs
		GLuint object_shader_program, lamp_shader_program;
		GLuint depthmap_program_directional, depthmap_program_point;
		GLuint depthmap_program_dir_debug, depthmap_program_point_debug;

		// Up, At vector
		glm::vec3 at, up;

		// For continouse keyboard inputs
		bool keys[1024];
		GLfloat delta_time;	// Time between current frame and last frame
		GLfloat last_frame;  // Time of last frame

		// For mouse callback
		bool first_mouse, cursor_enabled;
		GLfloat last_x, last_y;

		// renderQuad() renders a 1x1 XY quad in NDC
		GLuint quad_vao, quad_vbo;
		GLuint axis_vao, axis_vbo;
		Object *axis_obj;

		const glm::vec3 floor_blue;
		void InitGlfw();
		void InitGlew();
		void InitObjects();
		void InitShaders();
		void InitDepthBuffer();
		void LoadShaders(GLuint &program, const GLchar* vs_path, const GLchar* fs_path, const GLchar* gs_path);
		inline bool IsPointLight(const int i);

		void DoMovement();
		void RenderToDepthBuffer();
		void RenderScene();
		void LayoutObjects(GLuint &shaderProgram, const bool isDepthMapping);
		void LayoutAxes(GLuint &shaderProgram, const bool isDepthMapping);
		void RenderDepthDebug();
		void RenderDepthDebuggingQuad();
		void RenderLightSources(const glm::mat4 &view, const glm::mat4 &projection);
		inline glm::mat4 CalculateLightSpaceTransformationMatrix(const int lightIndex);
	};
}