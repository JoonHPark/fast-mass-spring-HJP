#pragma once

// Std. Includes
#include <vector>

// GL Includes
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#define AT_X 0.0f
#define AT_Y 0.0f
#define AT_Z 0.0f
#define UP_X 0.0f
#define UP_Y 0.0f
#define UP_Z 1.0f

namespace opengl {
	// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
	enum Camera_Movement {
		FORWARD,
		BACKWARD,
		LEFT,
		RIGHT,
		UPWARD,
		DOWNWARD
	};

	// Default camera values
	const GLfloat kYaw = 0.0f;
	const GLfloat kPitch = -40.0f;
	const GLfloat kSpeed = 8.0f;
	const GLfloat kSensitivity = 0.25f;
	const GLfloat kZoom = 45.0f;

	// An abstract camera class that processes input and calculates the corresponding Eular Angles, Vectors and Matrices for use in OpenGL
	class Camera
	{
	public:
		// Camera Attributes
		glm::vec3 position;
		glm::vec3 front;
		glm::vec3 up;
		glm::vec3 right;
		glm::vec3 world_up;
		// Eular Angles
		GLfloat yaw;
		GLfloat pitch;
		// Camera options
		GLfloat movement_speed;
		GLfloat mouse_sensitivity;
		GLfloat zoom;


		// Constructor with vectors
		Camera(bool loadedFromCache, glm::vec3 position = glm::vec3(10.0f, 10.0f, 10.0f), GLfloat yaw = kYaw, GLfloat pitch = kPitch) : front(glm::vec3(-1.0f, 0.0f, 0.0f)), movement_speed(kSpeed), mouse_sensitivity(kSensitivity), zoom(kZoom)
		{
			this->position = position;
			if (loadedFromCache) {
				this->yaw = yaw;
				this->pitch = pitch;
			}
			else {
				// compute initial yaw and pitch from position, looking at the origin
				glm::vec3 looking_direction = glm::normalize(-position);
				this->pitch = glm::degrees(asin(looking_direction.z));
				this->yaw = -glm::degrees(asin(looking_direction.x / cos(glm::radians(this->pitch)))) + 180.f;
			}
			this->world_up = glm::vec3(UP_X, UP_Y, UP_Z);
			this->UpdateCameraVectors();
		}


		// Returns the view matrix calculated using Eular Angles and the LookAt Matrix
		glm::mat4 GetViewMatrix()
		{
			return glm::lookAt(this->position, this->position + this->front, this->up);
		}

		// Processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
		void ProcessKeyboard(Camera_Movement direction, GLfloat deltaTime)
		{
			GLfloat velocity = this->movement_speed * deltaTime;
			if (direction == FORWARD)
				this->position += this->front * velocity;
			if (direction == BACKWARD)
				this->position -= this->front * velocity;
			if (direction == LEFT)
				this->position -= this->right * velocity;
			if (direction == RIGHT)
				this->position += this->right * velocity;

			if (direction == UPWARD) {
				this->position.z += velocity;
			}
			else if (direction == DOWNWARD) {
				this->position.z -= velocity;
			}
		}

		// Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
		void ProcessMouseMovement(GLfloat xoffset, GLfloat yoffset, GLboolean constrainPitch = true)
		{
			xoffset *= this->mouse_sensitivity;
			yoffset *= this->mouse_sensitivity;

			this->yaw += xoffset;
			this->pitch += yoffset;

			// Make sure that when pitch is out of bounds, screen doesn't get flipped
			if (constrainPitch)
			{
				if (this->pitch > 89.0f)
					this->pitch = 89.0f;
				if (this->pitch < -89.0f)
					this->pitch = -89.0f;
			}

			// Update Front, Right and Up Vectors using the updated Eular angles
			this->UpdateCameraVectors();
		}

		// Processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
		void ProcessMouseScroll(GLfloat yoffset)
		{
			if (this->zoom >= 1.0f && this->zoom <= 45.0f)
				this->zoom -= yoffset;
			if (this->zoom <= 1.0f)
				this->zoom = 1.0f;
			if (this->zoom >= 45.0f)
				this->zoom = 45.0f;
		}

	private:
		// Calculates the front vector from the Camera's (updated) Eular Angles
		void UpdateCameraVectors()
		{
			// Calculate the new Front vector
			glm::vec3 front;
			front.y = cos(glm::radians(this->yaw)) * cos(glm::radians(this->pitch));
			front.z = sin(glm::radians(this->pitch));
			front.x = sin(glm::radians(this->yaw)) * cos(glm::radians(this->pitch));
			this->front = glm::normalize(front);
			// Also re-calculate the Right and Up vector
			this->right = glm::normalize(glm::cross(this->front, this->world_up));  // Normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
			this->up = glm::normalize(glm::cross(this->right, this->front));
		}
	};
}