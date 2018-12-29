#version 330 core
out vec4 outputColor;

uniform bool isOn;
uniform bool isSelected;
void main()
{
	vec3 color, light;
	if(isOn) {
		light = vec3(1.f, 1.f, 1.f);
	} else {
		light = vec3(0.5f, 0.5f, 0.5f);
	}

	
	if(isSelected) {
		color = vec3(1.f, 0.f, 0.f);
	} else {
	color = vec3(1.f, 1.f, 1.f);
	}
	outputColor = vec4(color * light, 1.f);
}