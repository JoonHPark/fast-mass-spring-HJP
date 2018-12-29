#version 330 core
layout (location = 0) in vec3 position; // of the object to be rendered
layout (location = 1) in vec3 normal;
layout (location = 2) in vec2 texcoords;
out VS_OUT {
    vec3 FragPos;
    vec3 Normal;
	vec2 TexCoords;
} vs_out;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    vs_out.FragPos = vec3(model * vec4(position, 1.0));
	vs_out.TexCoords  = texcoords;
	
    vs_out.Normal = transpose(inverse(mat3(model))) * normal;
	gl_Position = (projection * view *  model) * vec4(position, 1.0f);
} 