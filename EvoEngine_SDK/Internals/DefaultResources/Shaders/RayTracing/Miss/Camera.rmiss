#extension GL_ARB_shading_language_include : enable
#include "RayTracingBasic.glsl"
#include "CameraRayTracingPayload.glsl"
#include "Atmosphere.glsl"

layout(location = 0) rayPayloadInEXT CameraRayTracingPayload hit_value;

layout(push_constant) uniform EE_CAMERA_CONSTANTS {
  uint EE_CAMERA_INDEX;
  uint EE_FRAME_ID;
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

	// Environment cubemap case
	int envIndex = EE_CAMERAS[EE_CAMERA_INDEX].prefiltered_map_index;

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
	Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
	return camera.use_clear_color == 1 ?
		camera.clear_color.xyz * camera.clear_color.w
		: pow(texture(EE_CUBEMAPS[camera.skybox_tex_index], normalize(direction)).rgb, vec3(1.0f / EE_ENVIRONMENT.gamma)) * camera.clear_color.w;
}

void main() 
{
	vec3 ray_origin = gl_WorldRayDirectionEXT;
	vec3 ray_direction = gl_WorldRayDirectionEXT;

	hit_value.type = 0;

	if (hit_value.hit_count > 0) {
		hit_value.color = EE_FUNC_ENV(ray_direction);
	} else {
		hit_value.color = EE_SKY_COLOR(ray_direction);
	}
}