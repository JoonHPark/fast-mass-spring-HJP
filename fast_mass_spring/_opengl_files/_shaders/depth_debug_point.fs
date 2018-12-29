#version 330 core
#define NUM_POINT_LIGHTS 2
struct Material {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;    
    float shininess;
}; 
struct DirectionalLight {
    vec3 direction;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
	bool isOn;
};
struct PointLight {
    vec3 position;
    float constant;
    float linear;
    float quadratic;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
	bool isOn;
};
in VS_OUT {
    vec3 FragPos;
    vec3 Normal;
	vec2 TexCoords;
} fs_in;


out vec4 color;


uniform samplerCube shadowMapTexture; // depth values are mapped as texture
uniform vec3 viewPos;


void main() {
    // get vector between fragment position and light position
    vec3 fragToLight = fs_in.FragPos - viewPos;
	// Get current linear depth as the length between the fragment and light position
    float currentDepth = length(fragToLight);
    // Test for shadows with PCF
    float shadow = 0.0;
    float bias = 0.15;
    float closestDepth = texture(shadowMapTexture, fragToLight).r;
	
	if(currentDepth - bias > closestDepth) {
        shadow = 1.0;
	}
    // display closestDepth
    color = vec4(vec3(closestDepth), 1.0);    
}
