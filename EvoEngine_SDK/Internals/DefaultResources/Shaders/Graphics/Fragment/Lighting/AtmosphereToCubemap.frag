#extension GL_ARB_shading_language_include : enable

precision highp float;
layout (location = 0) out vec4 FragColor;
layout (location = 0) in vec3 inWorldPos;

#include "Atmosphere.glsl"

layout(push_constant) uniform EE_PUSH_CONSTANTS{
	mat4 projection_view;
	Atmosphere atmosphere;

	vec3 sun_direction;
	float gamma;

	vec3 ground_color;
	float ground_transmittance;
};

void main()
{
	vec3 ray_direction = normalize(inWorldPos);
	vec3 color = NishitaSkyIncidentLight(atmosphere, vec3(0, 0, 0), ray_direction, sun_direction);
	if(ray_direction.y <= 0.f){
		color = mix(ground_color, color, ground_transmittance);
	}
	FragColor = vec4(pow(color, vec3(gamma)), 1.0);
}
