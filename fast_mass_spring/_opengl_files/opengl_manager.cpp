#include <iostream>
#include <fstream>
#include <sstream>
#include "opengl_manager.h"
#include "../../mesh.h"
using std::cout;
using std::endl;

extern Mesh *g_mesh;
extern bool g_mesh_fill;
opengl::OpenglManager::OpenglManager() {
	at = glm::vec3(AT_X, AT_Y, AT_Z);
	up = glm::vec3(UP_X, UP_Y, UP_Z);

	delta_time = 0.f;
	last_frame = 0.f;

	first_mouse = true;
	cursor_enabled = false;
	last_x = WINDOW_WIDTH / 2.0;
	last_y = WINDOW_HEIGHT / 2.0;

	quad_vao = 0;
	axis_vao = 0;

	selected_light_idx = -1;

	if (cacheManager.cache_loaded) {
		camera = new Camera(true, cacheManager.camera_pos, cacheManager.yaw, cacheManager.pitch);
	}
	else {
		camera = new Camera(false, data.camera_position);
	}

	InitGlfw();
	InitGlew();
	InitObjects();
	InitShaders();
	InitDepthBuffer();
}
void opengl::OpenglManager::ToggleLightSelection() {
	if (selected_light_idx == NUM_OF_LIGHTS - 1) {
		selected_light_idx = -1;
	}
	else {
		selected_light_idx++;
	}
	cout << "Light selected: " << selected_light_idx << endl;
}
void opengl::OpenglManager::SaveCache() {
	// Save cache
	cacheManager.camera_pos = camera->position;
	cacheManager.yaw = camera->yaw;
	cacheManager.pitch = camera->pitch;
	cacheManager.light_pos_arr.clear();

	for (int i = 0; i < NUM_OF_LIGHTS; i++) {
		cacheManager.light_pos_arr.push_back(lights[i].position);
	}
	cacheManager.Save();

	cout << "OpenGL Cache Saved!" << endl;
	cout << "  - Camera: [" << cacheManager.camera_pos.x << ", " << cacheManager.camera_pos.y << ", " << cacheManager.camera_pos.z << "]" << endl;
}
opengl::OpenglManager::~OpenglManager() {
	cout << "~OpenglManager()" << endl;

	delete camera;
}
void opengl::OpenglManager::Exit() {
	glfwSetWindowShouldClose(g_window, GL_TRUE);
}
void opengl::OpenglManager::StartRenderLoop() {
	// Update loop
	glEnable(GL_CULL_FACE);
	while (!glfwWindowShouldClose(g_window))
	{
		glEnable(GL_DEPTH_TEST);

		// Check and call events
		glfwPollEvents();
		DoMovement();

		// Reset buffers1
		glClearColor(0.01f, 0.01f, 0.01f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// ---------------------------------------------------- //
		// Rendering commands : START
		// ---------------------------------------------------- //
		// 1) Render depth.
		RenderToDepthBuffer();

		// 2) Render normal scene.
		glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		RenderScene();

		// 3) Render debugging depth map bottom left.
		glDisable(GL_DEPTH_TEST);
		glViewport(0, 0, WINDOW_WIDTH * 0.25, WINDOW_HEIGHT * 0.25);
		RenderDepthDebug();
		// ---------------------------------------------------- //
		// Rendering commands : END
		// ---------------------------------------------------- //
		// Swap the buffers
		glfwSwapBuffers(g_window);
	}

	// Clean up
	glDeleteShader(object_shader_program);
	glDeleteShader(lamp_shader_program);
	glDeleteShader(depthmap_program_directional);
	glDeleteShader(depthmap_program_point);
	glDeleteShader(depthmap_program_dir_debug);
	glDeleteShader(depthmap_program_point_debug);

	glfwTerminate();
}


void opengl::OpenglManager::RenderScene() {
	// Use the object shader program (VS, FS attached to it)
	glUseProgram(object_shader_program);

	// Re-Create tranformation matrix every frame
	GLint viewpos_loc = glGetUniformLocation(object_shader_program, "viewPos");
	GLint view_loc = glGetUniformLocation(object_shader_program, "view");
	GLint proj_loc = glGetUniformLocation(object_shader_program, "projection");
	glm::mat4 view = camera->GetViewMatrix();
	glm::mat4 projection = glm::perspective(camera->zoom, (GLfloat)WINDOW_WIDTH / (GLfloat)WINDOW_HEIGHT, NEAR_PLANE, FAR_PLANE);
	glUniformMatrix4fv(view_loc, 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(proj_loc, 1, GL_FALSE, glm::value_ptr(projection));
	glUniform3f(viewpos_loc, camera->position.x, camera->position.y, camera->position.z);


	// Set lights properties
	// Point light 1
	Light* light = &lights.at(0);
	glUniform3f(glGetUniformLocation(object_shader_program, "pointLights[0].position"), light->position.x, light->position.y, light->position.z);
	glUniform3f(glGetUniformLocation(object_shader_program, "pointLights[0].ambient"), light->prop.ambient.x, light->prop.ambient.y, light->prop.ambient.z);
	glUniform3f(glGetUniformLocation(object_shader_program, "pointLights[0].diffuse"), light->prop.diffuse.x, light->prop.diffuse.y, light->prop.diffuse.z);
	glUniform3f(glGetUniformLocation(object_shader_program, "pointLights[0].specular"), light->prop.specular.x, light->prop.specular.y, light->prop.specular.z);
	glUniform1f(glGetUniformLocation(object_shader_program, "pointLights[0].constant"), light->prop.constant);
	glUniform1f(glGetUniformLocation(object_shader_program, "pointLights[0].linear"), light->prop.linear);
	glUniform1f(glGetUniformLocation(object_shader_program, "pointLights[0].quadratic"), light->prop.quadratic);
	glUniform1i(glGetUniformLocation(object_shader_program, "pointLights[0].isOn"), light->is_on);
	// Point light 2
	light = &lights.at(1);
	glUniform3f(glGetUniformLocation(object_shader_program, "pointLights[1].position"), light->position.x, light->position.y, light->position.z);
	glUniform3f(glGetUniformLocation(object_shader_program, "pointLights[1].ambient"), light->prop.ambient.x, light->prop.ambient.y, light->prop.ambient.z);
	glUniform3f(glGetUniformLocation(object_shader_program, "pointLights[1].diffuse"), light->prop.diffuse.x, light->prop.diffuse.y, light->prop.diffuse.z);
	glUniform3f(glGetUniformLocation(object_shader_program, "pointLights[1].specular"), light->prop.specular.x, light->prop.specular.y, light->prop.specular.z);
	glUniform1f(glGetUniformLocation(object_shader_program, "pointLights[1].constant"), light->prop.constant);
	glUniform1f(glGetUniformLocation(object_shader_program, "pointLights[1].linear"), light->prop.linear);
	glUniform1f(glGetUniformLocation(object_shader_program, "pointLights[1].quadratic"), light->prop.quadratic);
	glUniform1i(glGetUniformLocation(object_shader_program, "pointLights[1].isOn"), light->is_on);
	// Directional light
	light = &lights.at(2);
	glUniform3f(glGetUniformLocation(object_shader_program, "dirLights[0].direction"), data.light_direction.x, data.light_direction.y, data.light_direction.z);
	glUniform3f(glGetUniformLocation(object_shader_program, "dirLights[0].ambient"), light->prop.ambient.x, light->prop.ambient.y, light->prop.ambient.z);
	glUniform3f(glGetUniformLocation(object_shader_program, "dirLights[0].diffuse"), light->prop.diffuse.x, light->prop.diffuse.y, light->prop.diffuse.z);
	glUniform3f(glGetUniformLocation(object_shader_program, "dirLights[0].specular"), light->prop.specular.x, light->prop.specular.y, light->prop.specular.z);
	glUniform1i(glGetUniformLocation(object_shader_program, "dirLights[0].isOn"), light->is_on);


	// For Light POV
	glm::mat4 light_pov_0 = CalculateLightSpaceTransformationMatrix(0);
	glUniformMatrix4fv(glGetUniformLocation(object_shader_program, "lightViewProjection_point0"), 1, GL_FALSE, glm::value_ptr(light_pov_0));
	glm::mat4 light_pov_1 = CalculateLightSpaceTransformationMatrix(1);
	glUniformMatrix4fv(glGetUniformLocation(object_shader_program, "lightViewProjection_point1"), 1, GL_FALSE, glm::value_ptr(light_pov_1));
	glm::mat4 light_pov_2 = CalculateLightSpaceTransformationMatrix(2);
	glUniformMatrix4fv(glGetUniformLocation(object_shader_program, "lightViewProjection_dir0"), 1, GL_FALSE, glm::value_ptr(light_pov_2));

	// Far plane
	glUniform1f(glGetUniformLocation(object_shader_program, "far_plane"), FAR_PLANE);

	// Attach textures
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, depthmap_textures.at(0));

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_CUBE_MAP, depthmap_textures.at(1));

	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, depthmap_textures.at(2));


	// Render
	LayoutObjects(object_shader_program, false);


	// Render lamps in the light's position
	//renderLightSources(&view, &projection);
}


void opengl::OpenglManager::LayoutObjects(GLuint &shaderProgram, const bool is_depthmapping) {
	GLint model = glGetUniformLocation(shaderProgram, "model");

	// Draw the floor
	glm::vec3 a = floor->ambient;
	glm::vec3 d = floor->diffuse;
	glm::vec3 s = floor->specular;
	glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.ambient"), a.x, a.y, a.z);
	glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.diffuse"), d.x, d.y, d.z);
	glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.specular"), s.x, s.y, s.z);
	glUniform1f(glGetUniformLocation(shaderProgram, "objMaterial.shininess"), floor->shininess);
	glUniformMatrix4fv(model, 1, GL_FALSE, glm::value_ptr(floor->GetModelMatrix()));
	floor->Render();

	// Render Axes
	if (!is_depthmapping) {
		//LayoutAxes(shaderProgram, is_depthmapping);
	}


	// -----------------------
	// nodes
	std::vector<DisplayData> disp_data;
	g_mesh->CopyRenderDataSafe(disp_data);

	// generate mesh
	g_mesh->UpdateVbo(disp_data);
	if (!is_depthmapping) { // If rendering depth buffer, material properties not needed.
		a = data.whitish.ambient;
		d = data.whitish.diffuse;
		s = data.whitish.specular;

		glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.ambient"), a.x, a.y, a.z);
		glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.diffuse"), d.x, d.y, d.z);
		glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.specular"), s.x, s.y, s.z);
		glUniform1f(glGetUniformLocation(shaderProgram, "objMaterial.shininess"), data.emerald_green.shininess);
	}
	glUniformMatrix4fv(model, 1, GL_FALSE, glm::value_ptr(g_mesh->GetModelMatrix()));
	if (g_mesh_fill) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
	else {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	}
	g_mesh->Render();


	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	// render each node
	if (!is_depthmapping) { // If rendering depth buffer, material properties not needed.
		a = cube->ambient;
		s = cube->specular;
		glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.ambient"), a.x, a.y, a.z);
		glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.specular"), s.x, s.y, s.z);
		glUniform1f(glGetUniformLocation(shaderProgram, "objMaterial.shininess"), cube->shininess);
	}
	for (std::vector<DisplayData>::iterator dp = disp_data.begin(); dp != disp_data.end(); dp++) {
		cube->position = (*dp).pos;
		SpringNodeType type = (*dp).type;

		if (g_mesh_fill) {
			if (type == face) {
				continue;
			}
		}
		if (!is_depthmapping) { // If rendering depth buffer, material properties not needed.
			d = data.red;
			if (g_mesh_fill) {
				if (type == vertex) {
					d = data.red;
					glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.diffuse"), d.x, d.y, d.z);
				}
				else if (type == edge) {
					d = data.blackish.diffuse;
					glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.diffuse"), d.x, d.y, d.z);
				}
				else {
					continue;
				}
			}
			else {
				if (type == vertex) {
					d = data.red;
				}
				else if (type == edge) {
					d = data.blackish.diffuse;
				}
				else {
					d = data.blue;
				}
				glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.diffuse"), d.x, d.y, d.z);
			}
		}

		if (type == vertex) {
			cube->scale *= 2.0f;
		}
		glUniformMatrix4fv(model, 1, GL_FALSE, glm::value_ptr(cube->GetModelMatrix()));
		if (type == vertex) {
			cube->scale /= 2.0f;
		}

		cube->Render();
	}


	Object *obj;
	// Draw other objects, if any
	for (unsigned int i = 0; i < objects.size(); i++) {
		obj = &objects[i];
		if (!is_depthmapping) { // If rendering depth buffer, material properties not needed.
			a = obj->ambient;
			d = obj->diffuse;
			s = obj->specular;
			glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.ambient"), a.x, a.y, a.z);
			glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.diffuse"), d.x, d.y, d.z);
			glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.specular"), s.x, s.y, s.z);
			glUniform1f(glGetUniformLocation(shaderProgram, "objMaterial.shininess"), obj->shininess);
		}

		// Render object
		glUniformMatrix4fv(model, 1, GL_FALSE, glm::value_ptr(obj->GetModelMatrix()));
		obj->Render();
	}


}
void opengl::OpenglManager::LayoutAxes(GLuint &shaderProgram, const bool isDepthMapping) {
	GLint model = glGetUniformLocation(shaderProgram, "model");

	glm::vec3 *d;
	if (!isDepthMapping) {
		glm::vec3 a = glm::vec3(0.2);
		glm::vec3 s = glm::vec3(5);
		glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.ambient"), a.x, a.y, a.z);
		glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.specular"), s.x, s.y, s.z);
		glUniform1f(glGetUniformLocation(shaderProgram, "objMaterial.shininess"), floor->shininess);
	}
	// Default is Identity Matrix
	glm::mat4 model_matrix; 

	// x
	d = &data.red;
	glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.diffuse"), d->x, d->y, d->z);
	model_matrix = glm::translate(model_matrix, AXIS_LENGTH*glm::vec3(0.5, 0, 0));
	model_matrix = glm::scale(model_matrix, AXIS_LENGTH*glm::vec3(1, AXIS_SCALE, AXIS_SCALE));
	glUniformMatrix4fv(model, 1, GL_FALSE, glm::value_ptr(model_matrix));
	axis_obj->Render();

	// y
	d = &data.green;
	glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.diffuse"), d->x, d->y, d->z);
	model_matrix = glm::mat4();
	model_matrix = glm::translate(model_matrix, AXIS_LENGTH*glm::vec3(0, 0.5, 0));
	model_matrix = glm::scale(model_matrix, AXIS_LENGTH*glm::vec3(AXIS_SCALE, 1, AXIS_SCALE));
	glUniformMatrix4fv(model, 1, GL_FALSE, glm::value_ptr(model_matrix));
	axis_obj->Render();

	// z
	d = &data.blue;
	glUniform3f(glGetUniformLocation(shaderProgram, "objMaterial.diffuse"), d->x, d->y, d->z);
	model_matrix = glm::mat4();
	model_matrix = glm::translate(model_matrix, AXIS_LENGTH*glm::vec3(0, 0, 0.5));
	model_matrix = glm::scale(model_matrix, AXIS_LENGTH*glm::vec3(AXIS_SCALE, AXIS_SCALE, 1));
	glUniformMatrix4fv(model, 1, GL_FALSE, glm::value_ptr(model_matrix));
	axis_obj->Render();
}

void opengl::OpenglManager::RenderLightSources(const glm::mat4 &view, const glm::mat4 &projection) {
	// Draw point lights as spheres
	glUseProgram(lamp_shader_program);
	GLint model_loc = glGetUniformLocation(lamp_shader_program, "model");
	GLint view_loc = glGetUniformLocation(lamp_shader_program, "view");
	GLint proj_loc = glGetUniformLocation(lamp_shader_program, "projection");
	GLint is_on = glGetUniformLocation(lamp_shader_program, "isOn");
	GLint is_selected = glGetUniformLocation(lamp_shader_program, "isSelected");

	// Set matrices
	glUniformMatrix4fv(view_loc, 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(proj_loc, 1, GL_FALSE, glm::value_ptr(projection));

	Light *light;
	for (unsigned int i = 0; i < lights.size(); i++) {
		light = &lights.at(i);
		glUniformMatrix4fv(model_loc, 1, GL_FALSE, glm::value_ptr(light->GetModelMatrix()));
		glUniform1i(is_on, light->is_on);
		if (selected_light_idx == i) {
			// This is the selected light
			glUniform1i(is_selected, true);
		}
		else {
			glUniform1i(is_selected, false);
		}
		light->Render();
	}
}
void opengl::OpenglManager::RenderDepthDebug() {
	if (selected_light_idx == -1) {
		return;
	}
	// render Depth map to quad for visual debugging
	if (IsPointLight(selected_light_idx)) {
		if (lights.at(selected_light_idx).is_on) {
			// Use the shader program (VS, FS attached to it)
			glUseProgram(depthmap_program_point_debug);

			// Re-Create tranformation matrix
			GLint viewpos_loc = glGetUniformLocation(depthmap_program_point_debug, "viewPos");
			GLint view_loc = glGetUniformLocation(depthmap_program_point_debug, "view");
			GLint proj_loc = glGetUniformLocation(depthmap_program_point_debug, "projection");
			glm::mat4 view = glm::lookAt(lights.at(selected_light_idx).position, at, up);
			glm::mat4 projection = glm::perspective(glm::radians(FIELD_OF_VIEW), (GLfloat)WINDOW_WIDTH / (GLfloat)WINDOW_HEIGHT, NEAR_PLANE, FAR_PLANE);
			glUniformMatrix4fv(view_loc, 1, GL_FALSE, glm::value_ptr(view));
			glUniformMatrix4fv(proj_loc, 1, GL_FALSE, glm::value_ptr(projection));
			glUniform3fv(viewpos_loc, 1, glm::value_ptr(lights.at(selected_light_idx).position));

			// For Light POV
			glm::mat4 light_pov = CalculateLightSpaceTransformationMatrix(selected_light_idx);
			glUniformMatrix4fv(glGetUniformLocation(depthmap_program_point_debug, "shadowMapTexture"), 1, GL_FALSE, glm::value_ptr(light_pov));
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_CUBE_MAP, depthmap_textures.at(selected_light_idx));

			LayoutObjects(depthmap_program_point_debug, true);
		}
	}
	else {
		glUseProgram(depthmap_program_dir_debug);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, depthmap_textures.at(selected_light_idx));
		RenderDepthDebuggingQuad();
	}

	// reset viewport
	glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
	glEnable(GL_DEPTH_TEST);
}


void opengl::OpenglManager::RenderDepthDebuggingQuad()
{
	if (quad_vao == 0)
	{
		float quad_vertices[] = {
			// positions        // texture Coords
			-1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
			-1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
			1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
			1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
		};
		// setup plane VAO
		glGenVertexArrays(1, &quad_vao);
		glGenBuffers(1, &quad_vbo);
		glBindVertexArray(quad_vao);
		glBindBuffer(GL_ARRAY_BUFFER, quad_vbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(quad_vertices), &quad_vertices, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
	}

	glBindVertexArray(quad_vao);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glBindVertexArray(0);

}
void opengl::OpenglManager::RenderToDepthBuffer() {
	Light* light;
	for (unsigned int i = 0; i < NUM_OF_LIGHTS; i++) {
		light = &lights.at(i);
		if (!light->is_on) {
			// Don't render if the light is off
			continue;
		}
		glViewport(0, 0, DEPTHMAP_WIDTH, DEPTHMAP_HEIGHT);
		GLuint shader_program;
		if (IsPointLight(i)) { // Point lights
			shader_program = depthmap_program_point;

			// 1. Set depth map transformation matrices
			glm::mat4 shadow_proj = glm::perspective(glm::radians(FIELD_OF_VIEW), (float)DEPTHMAP_WIDTH / (float)DEPTHMAP_HEIGHT, NEAR_PLANE, FAR_PLANE);
			std::vector<glm::mat4> shadow_transforms;
			glm::vec3 light_pos = light->position;
			shadow_transforms.push_back(shadow_proj * glm::lookAt(light_pos, light_pos + glm::vec3(1.0, 0.0, 0.0), glm::vec3(0.0, -1.0, 0.0)));
			shadow_transforms.push_back(shadow_proj * glm::lookAt(light_pos, light_pos + glm::vec3(-1.0, 0.0, 0.0), glm::vec3(0.0, -1.0, 0.0)));
			shadow_transforms.push_back(shadow_proj * glm::lookAt(light_pos, light_pos + glm::vec3(0.0, 1.0, 0.0), glm::vec3(0.0, 0.0, 1.0)));
			shadow_transforms.push_back(shadow_proj * glm::lookAt(light_pos, light_pos + glm::vec3(0.0, -1.0, 0.0), glm::vec3(0.0, 0.0, -1.0)));
			shadow_transforms.push_back(shadow_proj * glm::lookAt(light_pos, light_pos + glm::vec3(0.0, 0.0, 1.0), glm::vec3(0.0, -1.0, 0.0)));
			shadow_transforms.push_back(shadow_proj * glm::lookAt(light_pos, light_pos + glm::vec3(0.0, 0.0, -1.0), glm::vec3(0.0, -1.0, 0.0)));

			// 2. Pass matrices to the shader: Transfrom current vertex to light's POV
			glUseProgram(depthmap_program_point);
			for (unsigned int n = 0; n < 6; ++n) {
				std::string shadow_matrices = "shadowMatrices[" + std::to_string(n) + "]";
				glUniformMatrix4fv(glGetUniformLocation(depthmap_program_point, shadow_matrices.c_str()), 1, GL_FALSE, glm::value_ptr(shadow_transforms.at(n)));
			}
			glUniform1f(glGetUniformLocation(depthmap_program_point, "far_plane"), FAR_PLANE);
			glUniform3fv(glGetUniformLocation(depthmap_program_point, "lightPos"), 1, glm::value_ptr(light_pos));
		}
		else { // Directional light
			shader_program = depthmap_program_directional;

			// 1. Set matrices for the shader
			glUseProgram(depthmap_program_directional);

			// Transfrom current vertex to light's POV
			glm::mat4 light_view_projection_matrix = CalculateLightSpaceTransformationMatrix(i);
			glUniformMatrix4fv(glGetUniformLocation(depthmap_program_directional, "viewProjection"), 1, GL_FALSE, glm::value_ptr(light_view_projection_matrix));
		}

		// Render on the depth buffer
		glBindFramebuffer(GL_FRAMEBUFFER, depthmap_buffers.at(i));
		glClear(GL_DEPTH_BUFFER_BIT);
		LayoutObjects(shader_program, true);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}
}

void opengl::OpenglManager::ProcessKey(const int action, const int key) {
	if (selected_light_idx > -1 && selected_light_idx < NUM_OF_LIGHTS) {
		// -1 : no light selected
		Light* light = &lights[selected_light_idx];
		if (key == GLFW_KEY_Z) // Left -x
			light->position.x -= LIGHT_SPEED;
		if (key == GLFW_KEY_C) // Back -z
			light->position.z -= LIGHT_SPEED;
		if (key == GLFW_KEY_A) // Right +x
			light->position.x += LIGHT_SPEED;
		if (key == GLFW_KEY_D) // Forward +z
			light->position.z += LIGHT_SPEED;
		if (key == GLFW_KEY_S) // Up +y
			light->position.y += LIGHT_SPEED;
		if (key == GLFW_KEY_X) // Down -y
			light->position.y -= LIGHT_SPEED;

		if (action == GLFW_PRESS) {
			if (key == GLFW_KEY_0) // turn on/off the light
				light->is_on = !light->is_on;
		}
	}

	if (key >= 0 && key < 1024)
	{
		if (action == GLFW_PRESS)
			keys[key] = true;
		else if (action == GLFW_RELEASE)
			keys[key] = false;
	}
}
void opengl::OpenglManager::ProcessMouse(const double pos_x, const double pos_y) {

	if (first_mouse)
	{
		last_x = pos_x;
		last_y = pos_y;
		first_mouse = false;
	}

	GLfloat offset_x = pos_x - last_x;
	GLfloat offset_y = last_y - pos_y;  // Reversed since y-coordinates go from bottom to left

	last_x = pos_x;
	last_y = pos_y;

	camera->ProcessMouseMovement(offset_x, offset_y);
}

void opengl::OpenglManager::DoMovement()
{
	// Calculate deltatime of current frame
	GLfloat curr_frame = glfwGetTime();
	delta_time = curr_frame - last_frame;
	last_frame = curr_frame;

	// _camera controls
	if (keys[GLFW_KEY_UP])
		camera->ProcessKeyboard(FORWARD, delta_time);
	if (keys[GLFW_KEY_DOWN])
		camera->ProcessKeyboard(BACKWARD, delta_time);
	if (keys[GLFW_KEY_LEFT])
		camera->ProcessKeyboard(LEFT, delta_time);
	if (keys[GLFW_KEY_RIGHT])
		camera->ProcessKeyboard(RIGHT, delta_time);
	if (keys[GLFW_KEY_LEFT_BRACKET])
		camera->ProcessKeyboard(DOWNWARD, delta_time);
	if (keys[GLFW_KEY_RIGHT_BRACKET])
		camera->ProcessKeyboard(UPWARD, delta_time);
}

void opengl::OpenglManager::InitDepthBuffer() {
	// Create FBO and TextureObject for each of the 3 lights, keep them in the global array for easy access.
	for (unsigned int i = 0; i < NUM_OF_LIGHTS; i++) {
		// Configure depth map FBO
		GLuint depthmap_fbo, depthmap_texture;
		glGenFramebuffers(1, &depthmap_fbo);

		// Wil attach depth texture as FBO's depth buffer
		glBindFramebuffer(GL_FRAMEBUFFER, depthmap_fbo);
		// Create depth texture
		glGenTextures(1, &depthmap_texture);
		if (IsPointLight(i)) {
			// Point lights -> Cube Map
			glBindTexture(GL_TEXTURE_CUBE_MAP, depthmap_texture);
			for (unsigned int n = 0; n < 6; ++n) {
				glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + n, 0, GL_DEPTH_COMPONENT, DEPTHMAP_WIDTH, DEPTHMAP_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
			}
			glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
			glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depthmap_texture, 0);
		}
		else {
			// Directional light -> 2D Texture
			glBindTexture(GL_TEXTURE_2D, depthmap_texture);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, DEPTHMAP_WIDTH, DEPTHMAP_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
			GLfloat borderColor[] = { 1.0, 1.0, 1.0, 1.0 };
			glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthmap_texture, 0);
		}
		// No need for RGB data.	
		glDrawBuffer(GL_NONE);
		glReadBuffer(GL_NONE);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		// Keep FBO and Texture in the global array.
		depthmap_buffers.push_back(depthmap_fbo);
		depthmap_textures.push_back(depthmap_texture);
	} // For loop ends

	  // Attach texture maps to the shaders
	glUseProgram(object_shader_program);
	GLint shadowmap_texture_0 = glGetUniformLocation(object_shader_program, "shadowMapTexture_point0");
	glUniform1i(shadowmap_texture_0, 0);
	GLint shadowmap_texture_1 = glGetUniformLocation(object_shader_program, "shadowMapTexture_point1");
	glUniform1i(shadowmap_texture_1, 1);
	GLint shadowmap_texture_2 = glGetUniformLocation(object_shader_program, "shadowMapTexture_dir0");
	glUniform1i(shadowmap_texture_2, 2);

	// For debugging
	glUseProgram(depthmap_program_dir_debug);
	GLint depthmap = glGetUniformLocation(depthmap_program_dir_debug, "depthMap");
	glUniform1i(depthmap, 0);

	glUseProgram(depthmap_program_point_debug);
	GLint depthmap_cube = glGetUniformLocation(depthmap_program_point_debug, "depthMap");
	glUniform1i(depthmap_cube, 0);
}
void opengl::OpenglManager::InitShaders() {
	LoadShaders(object_shader_program, "_opengl_files/_shaders/object.vs", "_opengl_files/_shaders/object.fs", NULL);
	LoadShaders(lamp_shader_program, "_opengl_files/_shaders/lamp.vs", "_opengl_files/_shaders/lamp.fs", NULL);
	LoadShaders(depthmap_program_directional, "_opengl_files/_shaders/depth_mapping_directional.vs", "_opengl_files/_shaders/depth_mapping_directional.fs", NULL);
	LoadShaders(depthmap_program_point, "_opengl_files/_shaders/depth_mapping_point.vs", "_opengl_files/_shaders/depth_mapping_point.fs", "_opengl_files/_shaders/depth_mapping_point.gs");
	LoadShaders(depthmap_program_dir_debug, "_opengl_files/_shaders/depth_debug_directional.vs", "_opengl_files/_shaders/depth_debug_directional.fs", NULL);
	LoadShaders(depthmap_program_point_debug, "_opengl_files/_shaders/depth_debug_point.vs", "_opengl_files/_shaders/depth_debug_point.fs", NULL);
}
void opengl::OpenglManager::InitObjects() {
	// init objects to render
	cube = new Object(data.cube_data.position, data.cube_data.axis, data.cube_data.angle, data.cube_data.scale, data.cube_data.obj_path);
	cube->SetMaterial(data.cube_data.ambient, data.cube_data.diffuse, data.cube_data.specular, data.cube_data.shininess);

	// floors, axes and lights
	std::vector<glm::vec3> obj_vertices;
	floor = new Object(data.floor_data.position, data.floor_data.axis, data.floor_data.angle, data.floor_data.scale, data.floor_data.obj_path, obj_vertices);
	floor->SetMaterial(data.floor_data.ambient, data.floor_data.diffuse, data.floor_data.specular, data.floor_data.shininess);
	
	axis_obj = new Object(glm::vec3(0), glm::vec3(0, 1, 0), 0.f, glm::vec3(1), obj_vertices);

	for (int i = 0; i < NUM_OF_LIGHTS; i++) {
		glm::vec3 light_pos;
		if (cacheManager.cache_loaded) {
			light_pos = cacheManager.light_pos_arr[i];
		}
		else {
			light_pos = data.light_positions[i];
			cacheManager.light_pos_arr.push_back(light_pos);
		}

		Light light = Light(light_pos, glm::vec3(0.f, 0.f, 1.f), 0.f, glm::vec3(0.5f), true, obj_vertices);
		light.prop = data.light_props[i];
		lights.push_back(light);
	}
}

void opengl::OpenglManager::InitGlew() {
	// GL_TRUE: ensures GLEW uses more modern techniques for managing OpenGL functionality
	glewExperimental = GL_TRUE;
	if (glewInit() != GLEW_OK) {
		std::cout << "Failed to initialize GLEW" << std::endl;
		return;
	}
}
void opengl::OpenglManager::InitGlfw() {
	glfwInit();

	// Configure GLFW: tell OpenGL that I am using GLFW version 3.x
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	// Create window
	g_window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Hyojoon Park", nullptr, nullptr);
	if (g_window == nullptr) {
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return;
	}
	glfwMakeContextCurrent(g_window);
}

void opengl::OpenglManager::SetKeyCallback(void(*keyCallback)(GLFWwindow* window, const int key, const int scancode, const int action, const int mode)) {
	glfwSetKeyCallback(g_window, keyCallback);
}
void opengl::OpenglManager::SetMouseCallback(void(*mouseCallback)(GLFWwindow* window, const double xpos, const double ypos)) {
	glfwSetCursorPosCallback(g_window, mouseCallback);
	//glfwSetInputMode(g_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
}
void opengl::OpenglManager::ToggleCursorMode(const bool toggle) {
	if (toggle) {
		glfwSetInputMode(g_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	}
	else {
		glfwSetInputMode(g_window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	}
}

inline glm::mat4 opengl::OpenglManager::CalculateLightSpaceTransformationMatrix(const int lightIndex) {
	glm::mat4 light_view, light_projection;
	light_view = glm::lookAt(lights.at(lightIndex).position, at, up);
	if (IsPointLight(lightIndex)) {
		// Point Light POV -> Use perspective
		light_projection = glm::perspective(FIELD_OF_VIEW, (GLfloat)DEPTHMAP_WIDTH / (GLfloat)DEPTHMAP_HEIGHT, NEAR_PLANE, FAR_PLANE);
	}
	else {
		// Directional Light POV -> Use ortho
		light_projection = glm::ortho(-5.0f, 5.0f, -5.0f, 5.0f, NEAR_PLANE, FAR_PLANE);
	}
	return (light_projection * light_view);
}
void opengl::OpenglManager::LoadShaders(GLuint &program, const GLchar* vertexPath, const GLchar* fragmentPath, const GLchar* geometryPath) {
	// 1. Retrieve the vertex/fragment source code from filePath
	std::string vs_code;
	std::string fs_code;
	std::ifstream vs_file;
	std::ifstream fs_file;
	// ensures ifstream objects can throw exceptions:
	vs_file.exceptions(std::ifstream::badbit);
	fs_file.exceptions(std::ifstream::badbit);
	try
	{
		// Open files
		vs_file.open(vertexPath);
		fs_file.open(fragmentPath);
		std::stringstream vs_stream, fs_stream;
		// Read file's buffer contents into streams
		vs_stream << vs_file.rdbuf();
		fs_stream << fs_file.rdbuf();
		// close file handlers
		vs_file.close();
		fs_file.close();
		// Convert stream into string
		vs_code = vs_stream.str();
		fs_code = fs_stream.str();
	}
	catch (std::ifstream::failure e)
	{
		std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ: " << vertexPath << std::endl;
	}
	const GLchar* vs_code_cstr = vs_code.c_str();
	const GLchar * fs_coder_cstr = fs_code.c_str();
	// 2. Compile shaders
	GLuint vertex, fragment;
	GLint success;
	GLchar info_log[512];
	// Vertex Shader
	vertex = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertex, 1, &vs_code_cstr, NULL);
	glCompileShader(vertex);
	// Print compile errors if any
	glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(vertex, 512, NULL, info_log);
		std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << info_log << vertexPath << std::endl;
	}
	// Fragment Shader
	fragment = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragment, 1, &fs_coder_cstr, NULL);
	glCompileShader(fragment);
	// Print compile errors if any
	glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(fragment, 512, NULL, info_log);
		std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << info_log << vertexPath << std::endl;
	}
	// Shader Program
	program = glCreateProgram();
	glAttachShader(program, vertex);
	glAttachShader(program, fragment);


	GLuint geometry;
	if (geometryPath != NULL) {
		// 1. Retrieve the geometry source code from filePath
		std::string gs_code;
		std::ifstream gs_file;
		// ensures ifstream objects can throw exceptions:
		gs_file.exceptions(std::ifstream::badbit);
		try
		{
			// Open files
			gs_file.open(geometryPath);
			std::stringstream gs_stream;
			// Read file's buffer contents into streams
			gs_stream << gs_file.rdbuf();
			// close file handlers
			gs_file.close();
			// Convert stream into string
			gs_code = gs_stream.str();
		}
		catch (std::ifstream::failure e)
		{
			std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ: " << geometryPath << std::endl;
		}
		const GLchar* gs_code_cstr = gs_code.c_str();
		// 2. Compile shaders
		// Vertex Shader
		geometry = glCreateShader(GL_GEOMETRY_SHADER);
		glShaderSource(geometry, 1, &gs_code_cstr, NULL);
		glCompileShader(geometry);
		// Print compile errors if any
		glGetShaderiv(geometry, GL_COMPILE_STATUS, &success);
		if (!success)
		{
			glGetShaderInfoLog(geometry, 512, NULL, info_log);
			std::cout << "ERROR::SHADER::GEOMETRY::COMPILATION_FAILED\n" << info_log << geometryPath << std::endl;
		}
		glAttachShader(program, geometry);
	}

	glLinkProgram(program);
	// Print linking errors if any
	glGetProgramiv(program, GL_LINK_STATUS, &success);
	if (!success)
	{
		glGetProgramInfoLog(program, 512, NULL, info_log);
		std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << info_log << std::endl;
	}
	// Delete the shaders as they're linked into our program now and no longer necessery
	//std::cout << "SHADERS LOADED:\n    " << vertexPath << "\n    " << fragmentPath;

	if (geometryPath != NULL) {
		//std::cout << "\n    " << geometryPath << std::endl;
		glDeleteShader(geometry);
	}
	else {
		//std::cout << std::endl;
	}
	glDeleteShader(vertex);
	glDeleteShader(fragment);
}
inline bool opengl::OpenglManager::IsPointLight(const int i) {
	// Directional light is stored in the last index.
	return (i != NUM_OF_LIGHTS - 1);
}
