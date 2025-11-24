#extension GL_ARB_shading_language_include : enable
#include "RayTracingBasic.glsl"

#include "PointCloudRayTracingPayload.glsl"
#include "Atmosphere.glsl"

layout(location = 0) rayPayloadInEXT PointCloudRayTracingPayload hit_value;

layout(push_constant) uniform EE_POINT_CLOUD_CONSTANTS {
  uint bounce;
  uint envIndex;
  uint skybox_tex_index;
  uint use_clear_color;
  vec4 clear_color;
};

vec3 EE_FUNC_ENV(vec3 rayDir)
{
	// Local copies  fewer uniform reads, cleaner code
	vec4 bg      = EE_ENVIRONMENT.background_color;
	float gamma  = EE_ENVIRONMENT.gamma;
	float inten  = EE_ENVIRONMENT.light_intensity;

	// Solid background color case (no texture fetch)
	if (bg.w == 1.0)
	{
		return bg.rgb * inten;
	}

	// LOD 0: sharp environment for miss rays
	vec3 envColor = textureLod(EE_CUBEMAPS[envIndex], rayDir, 0.0).rgb;

	// Apply gamma only if needed
	if (gamma != 1.0)
	{
		float invGamma = 1.0 / gamma;
		envColor = pow(envColor, vec3(invGamma));
	}

	return envColor * inten;
}

vec3 EE_SKY_COLOR(vec3 direction) {
	return use_clear_color == 1 ?
		clear_color.xyz * clear_color.w
		: pow(texture(EE_CUBEMAPS[skybox_tex_index], normalize(direction)).rgb, vec3(1.0f / EE_ENVIRONMENT.gamma)) * clear_color.w;
}

void main() 
{
	vec3 ray_origin = gl_WorldRayDirectionEXT;
	vec3 ray_direction = gl_WorldRayDirectionEXT;

	if (hit_value.hit_count > 0) {
		hit_value.hit_info.color = vec4(EE_FUNC_ENV(ray_direction), 1.0f);
	} else {
		hit_value.hit_info.color = vec4(EE_SKY_COLOR(ray_direction), 1.0f);
	}
	hit_value.handle = 0;
	hit_value.hit_info.position = vec3(0, 0, 0);
	hit_value.hit_info.normal = vec3(0, 0, 0);
	hit_value.hit_info.tangent = vec3(0, 0, 0);
	hit_value.hit_info.tex_coord = vec2(0, 0);

	hit_value.hit_info.vertex_info1 = 0;
    hit_value.hit_info.vertex_info2 = 0;
    hit_value.hit_info.vertex_info3 = 0;
    hit_value.hit_info.vertex_info4 = vec2(0, 0);
}