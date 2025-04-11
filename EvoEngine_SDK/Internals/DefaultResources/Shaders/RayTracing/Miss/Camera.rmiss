#extension GL_ARB_shading_language_include : enable
#include "RayTracingBasic.glsl"
#include "CameraRayTracingPayload.glsl"
#include "Atmosphere.glsl"

layout(location = 0) rayPayloadInEXT CameraRayTracingPayload hit_value;

layout(push_constant) uniform EE_CAMERA_CONSTANTS {
  uint EE_CAMERA_INDEX;
  uint EE_FRAME_ID;
};


vec3 EE_SKY_COLOR(vec3 direction) {
	const Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
	return camera.use_clear_color == 1 ?
		camera.clear_color.xyz * camera.clear_color.w
		: pow(texture(EE_CUBEMAPS[camera.skybox_tex_index], normalize(direction)).rgb, vec3(1.0f / EE_ENVIRONMENT.gamma)) * camera.clear_color.w;
}


void main() 
{
	vec3 ray_origin = gl_WorldRayDirectionEXT;
	vec3 ray_direction = gl_WorldRayDirectionEXT;
	hit_value.color = EE_SKY_COLOR(ray_direction) * EE_ENVIRONMENT.light_intensity;
	hit_value.type = 0;
}