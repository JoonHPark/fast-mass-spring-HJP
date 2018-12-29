#include "node.h"
namespace opengl {
	Node::Node(const glm::vec3 pos, const glm::vec3 axis, const float ang, const glm::vec3 sca) {
		position = pos;
		rotation_axis = axis;
		angle = ang;
		scale = sca;
	}
	Node::~Node() {
		if (vao != 0) {
			glDeleteVertexArrays(1, &vao);
			glDeleteBuffers(1, &vbo);
		}
	}

	void Node::SetMaterial(const glm::vec3 a, const  glm::vec3 d, const glm::vec3 s, const float shine) {
		ambient = a;
		diffuse = d;
		specular = s;
		shininess = shine;
	}

	glm::mat4 Node::GetModelMatrix() {
		glm::mat4 model_matrix; // Default is Identity Matrix
		model_matrix = glm::translate(model_matrix, position);
		model_matrix = glm::rotate(model_matrix, (GLfloat)angle, rotation_axis);
		model_matrix = glm::scale(model_matrix, scale);
		return model_matrix;
	}

	bool Node::LoadObj(const char * path, std::vector<glm::vec3> & out_vertices, std::vector<glm::vec3> & out_normals) {
		//printf("Loading OBJ file: %s...", path);


		FILE * file = fopen(path, "r");
		if (file == NULL) {
			printf("Impossible to open the file. Check the file path.\n");
			getchar();
			return false;
		}

		std::vector<unsigned int> vertex_indices, normal_indices;
		std::vector<glm::vec3> temp_vertices, temp_normals;
		temp_vertices.reserve(MAX_ARRAY_SIZE);
		temp_normals.reserve(MAX_ARRAY_SIZE);
		vertex_indices.reserve(MAX_ARRAY_SIZE);
		normal_indices.reserve(MAX_ARRAY_SIZE);

		while (1) {
			char line_header[1024];
			// read the first word of the line
			int res = fscanf(file, "%s", line_header);
			if (res == EOF)
				break; // EOF = End Of File. Quit the loop.
						// else : parse lineHeader
			if (strcmp(line_header, "v") == 0) {
				// Vertex data
				glm::vec3 vertex;
				fscanf(file, "%f %f %f\n", &vertex.x, &vertex.y, &vertex.z);
				temp_vertices.push_back(vertex);
			}

			else if (strcmp(line_header, "vn") == 0) {
				// Vertex normal data
				glm::vec3 normal;
				fscanf(file, "%f %f %f\n", &normal.x, &normal.y, &normal.z);
				temp_normals.push_back(normal);
			}
			else if (strcmp(line_header, "f") == 0) {
				// Face data
				unsigned int vertex_index[3], uv_index[3], normal_index[3];
				int matches = fscanf(file, "%d//%d %d//%d %d//%d\n", &vertex_index[0], &normal_index[0], &vertex_index[1], &normal_index[1], &vertex_index[2], &normal_index[2]);
				if (matches != 6) {
					printf("File can't be read.\n");
					fclose(file);
					return false;
				}
				vertex_indices.push_back(vertex_index[0]);
				vertex_indices.push_back(vertex_index[1]);
				vertex_indices.push_back(vertex_index[2]);
				normal_indices.push_back(normal_index[0]);
				normal_indices.push_back(normal_index[1]);
				normal_indices.push_back(normal_index[2]);
			}
			else {
			}
		}
		vertex_indices.shrink_to_fit();
		normal_indices.shrink_to_fit();
		temp_vertices.shrink_to_fit();
		temp_normals.shrink_to_fit();

		// For each vertex of each triangle
		for (unsigned int i = 0; i < vertex_indices.size(); i++) {

			// Get the indices of its attributes
			unsigned int vertex_index = vertex_indices[i];
			unsigned int normal_index = normal_indices[i];

			// Get the attributes thanks to the index
			glm::vec3 vertex = temp_vertices[vertex_index - 1];
			glm::vec3 normal = temp_normals[normal_index - 1];

			// Put the attributes in buffers
			out_vertices.push_back(vertex);
			out_normals.push_back(normal);
		}
		fclose(file);
		return true;
	}
}