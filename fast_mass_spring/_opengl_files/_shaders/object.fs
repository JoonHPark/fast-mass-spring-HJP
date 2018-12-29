#version 330 core

#define NUM_POINT_LIGHTS 2
#define NUM_DIRECTIONAL_LIGHTS 1
#define SHINE_CORRECTION 100.f
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

// Output
out vec4 color;

// Receive from vertex shader
in VS_OUT {
    vec3 FragPos;
    vec3 Normal;
	vec2 TexCoords;
} fs_in;
in vec4 FragPosLightSpaceDirLight;

// Set from main.cpp
uniform samplerCube shadowMapTexture_point0;
uniform samplerCube shadowMapTexture_point1;
uniform sampler2D shadowMapTexture_dir0;

uniform float far_plane;
uniform vec3 viewPos;
uniform DirectionalLight dirLights[NUM_DIRECTIONAL_LIGHTS];
uniform PointLight pointLights[NUM_POINT_LIGHTS];
uniform Material objMaterial;



// Function prototypes
vec3 CalcDirectionalLight(DirectionalLight light, vec3 normal, vec3 viewDir, int i);
vec3 CalcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir, int i);
float ShadowCalculation_Directional(vec4 fragPosLightSpace, sampler2D shadowMapTexture, bool isPoint, int i);
float ShadowCalculation_PointLight(vec3 fragPos, vec3 lightPos, samplerCube depthMap);
float advancedShadowEffect_Directional(sampler2D shadowMapTexture, vec3 projCoords, float currentDepth, float bias);

void main() {
	// Properties
    vec3 norm = normalize(fs_in.Normal);
    vec3 viewDir = normalize(viewPos - fs_in.FragPos);

	vec3 result;
	// Calculate Directional Light
	for(int i = 0; i < NUM_DIRECTIONAL_LIGHTS; i++) {
		if(dirLights[i].isOn) {
			result += CalcDirectionalLight(dirLights[i], norm, viewDir, i);
		}
	}

	// Calculate Point Lights
	for(int i = 0; i < NUM_POINT_LIGHTS; i++) {
		if(pointLights[i].isOn) {
			result += CalcPointLight(pointLights[i], norm, fs_in.FragPos, viewDir, i);   
		}
	}

	// Normalize the color value
	vec3 mapped = result / (result + vec3(1.0));

	// Output the color value
	color = vec4(mapped, 1.0);
}


// Calculates the color when using a point light.
vec3 CalcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir, int i)
{
	// Calculate Shadow
	float shadow;
	if(i == 0){
		shadow = ShadowCalculation_PointLight(fs_in.FragPos, light.position, shadowMapTexture_point0);
	} else if (i == 1) {
		shadow = ShadowCalculation_PointLight(fs_in.FragPos, light.position, shadowMapTexture_point1);
	} else {
		// Should NEVER be reached
		return vec3(0.0f);
	}
	
	// Ambient
    vec3 ambient = light.ambient * objMaterial.ambient;
  	
    // Diffuse 
    vec3 lightDir = normalize(light.position - fragPos);
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = light.diffuse * diff * objMaterial.diffuse;  
    
    // Specular
    vec3 reflectDir = reflect(-lightDir, normal);  
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), objMaterial.shininess * SHINE_CORRECTION);
    vec3 specular = light.specular * spec * objMaterial.specular;
    
    // Attenuation
    float distance    = length(light.position - fragPos);
    float attenuation = 1.0f / (light.constant + light.linear * distance + light.quadratic * (distance * distance));    

    ambient  *= attenuation;  
    diffuse  *= attenuation;
    specular *= attenuation;

	// Without shadow
    //return (ambient + diffuse + specular);

	// With shadow
	return (ambient + (1.0 - shadow) * (diffuse + specular));
}


// Calculates the color when using a directional light.
vec3 CalcDirectionalLight(DirectionalLight light, vec3 normal, vec3 viewDir, int i)
{
	// Calculate Shadow
	float shadow;
	if(i == 0){
		shadow = ShadowCalculation_Directional(FragPosLightSpaceDirLight, shadowMapTexture_dir0, false, i);
	} else {
		// Should NEVER be reached
		return vec3(0.0f);
	}

    vec3 lightDir = normalize(-light.direction);

    // Diffuse 
    float diff = max(dot(normal, lightDir), 0.0);

    // Specular 
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), objMaterial.shininess * SHINE_CORRECTION);

    // Combine results
    vec3 ambient = light.ambient * objMaterial.diffuse;
    vec3 diffuse = light.diffuse * diff * objMaterial.diffuse;
    vec3 specular = light.specular * spec * objMaterial.specular;

	// Without shadow
    //return (ambient + diffuse + specular);

	// With Shadow
	return (ambient + (1.0 - shadow*0.5) * (diffuse + specular));
}

float ShadowCalculation_PointLight(vec3 fragPos, vec3 lightPos, samplerCube depthMap)
{
	// Get vector between fragment position and light position
    vec3 fragToLight = fragPos - lightPos;
    // Get current linear depth as the length between the fragment and light position
    float currentDepth = length(fragToLight);
    // Test for shadows with PCF
    float shadow = 0.0;
    float bias = 0.15;
    float closestDepth = texture(depthMap, fragToLight).r;
    closestDepth *= far_plane;   // Undo mapping [0;1]
    if(currentDepth - bias > closestDepth) {
        shadow = 1.0;
	}
	
    // return shadow;
    return shadow;
}

float ShadowCalculation_Directional(vec4 fragPosLightSpace, sampler2D shadowMapTexture, bool isPoint, int i)
{
    // perform perspective divide
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    // Transform to [0,1] range
    projCoords = projCoords * 0.5 + 0.5;
    // Get closest depth value from light's perspective (using [0,1] range fragPosLight as coords)
    float closestDepth = texture(shadowMapTexture, projCoords.xy).r; 
    // Get depth of current fragment from light's perspective
    float currentDepth = projCoords.z;
    // Calculate bias (based on depth map resolution and slope)
    vec3 normal = normalize(fs_in.Normal);
    vec3 lightDir = normalize(-dirLights[i].direction);
	
    float bias = max(0.05 * (1.0 - dot(normal, lightDir)), 0.005);
	//float bias = 0.005 * tan(acos(dot(normal, lightDir)));
	bias = clamp(bias, 0, 0.01);

   // Improved shadow effect
    float shadow = advancedShadowEffect_Directional(shadowMapTexture, projCoords, currentDepth, bias);
    return shadow;
}


float advancedShadowEffect_Point(sampler2D shadowMapTexture, vec3 projCoords, float currentDepth, float bias) {
	// Check whether current frag pos is in shadow
    // float shadow = currentDepth - bias > closestDepth  ? 1.0 : 0.0; // NOT USED ANYMORE

    // PCF
    float shadow = 0.0;
    vec2 texelSize = 1.0 / textureSize(shadowMapTexture, 0);
    for(int x = -1; x <= 1; ++x)
    {
        for(int y = -1; y <= 1; ++y)
        {
            float pcfDepth = texture(shadowMapTexture, projCoords.xy + vec2(x, y) * texelSize).r; 
            shadow += currentDepth - bias > pcfDepth  ? 1.0 : 0.0;        
        }    
    }
    shadow /= 9.0;
    
    // Keep the shadow at 0.0 when outside the far_plane region of the light's frustum.
    if(projCoords.z > 1.0) {
        shadow = 0.0;
	}

	return shadow;
}

float advancedShadowEffect_Directional(sampler2D shadowMapTexture, vec3 projCoords, float currentDepth, float bias) {
	// Check whether current frag pos is in shadow
    // float shadow = currentDepth - bias > closestDepth  ? 1.0 : 0.0; // NOT USED ANYMORE

    // PCF
    float shadow = 0.0;
    vec2 texelSize = 1.0 / textureSize(shadowMapTexture, 0);
    for(int x = -1; x <= 1; ++x)
    {
        for(int y = -1; y <= 1; ++y)
        {
            float pcfDepth = texture(shadowMapTexture, projCoords.xy + vec2(x, y) * texelSize).r; 
            shadow += currentDepth - bias > pcfDepth  ? 1.0 : 0.0;        
        }    
    }
    shadow /= 9.0;
    
    // Keep the shadow at 0.0 when outside the far_plane region of the light's frustum.
    if(projCoords.z > 1.0) {
        shadow = 0.0;
	}

	return shadow;
}