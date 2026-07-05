#extension GL_ARB_shading_language_include : enable
#include "RayTracingBasic.glsl"
#include "CameraRayTracingPayload.glsl"
#include "Atmosphere.glsl"

layout(location = 0) rayPayloadInEXT CameraRayTracingPayload hit_value;

layout(push_constant) uniform EE_CAMERA_CONSTANTS {
  uint EE_CAMERA_INDEX;
  uint EE_FRAME_ID;
};

const uint EE_CAMERA_RAY_PAYLOAD_SHADOW = 1u;
const float EE_CAMERA_PI = 3.14159265359f;
const float EE_CAMERA_PDF_EPSILON = 1e-6f;

bool EE_CAMERA_HAS_ENVIRONMENT_LIGHTING() {
	if (EE_ENVIRONMENT.light_intensity <= 0.0f) {
		return false;
	}
	if (EE_ENVIRONMENT.background_color.w == 1.0f) {
		return dot(max(EE_ENVIRONMENT.background_color.rgb, vec3(0.0f)), vec3(1.0f)) > 0.0f;
	}
	return true;
}

float EE_CAMERA_ENVIRONMENT_PDF(const vec3 normal, const vec3 light_direction) {
	if (!EE_CAMERA_HAS_ENVIRONMENT_LIGHTING() || dot(normal, light_direction) <= 0.0f) {
		return 0.0f;
	}
	return 1.0f / (2.0f * EE_CAMERA_PI);
}

float EE_CAMERA_ENVIRONMENT_HIT_MIS_WEIGHT(const float bsdf_pdf, const float environment_pdf) {
	const float safe_bsdf_pdf = max(bsdf_pdf, 0.0f);
	const float safe_environment_pdf = max(environment_pdf, 0.0f);
	const float pdf_sum = safe_bsdf_pdf + safe_environment_pdf;
	return pdf_sum > EE_CAMERA_PDF_EPSILON ? safe_bsdf_pdf / pdf_sum : 1.0f;
}

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
	if (hit_value.type == EE_CAMERA_RAY_PAYLOAD_SHADOW) {
		hit_value.hit_count = 0u;
		return;
	}

	vec3 ray_origin = gl_WorldRayDirectionEXT;
	vec3 ray_direction = gl_WorldRayDirectionEXT;

	hit_value.type = 0;

	if (hit_value.hit_count > 0) {
		const float environment_pdf = EE_CAMERA_ENVIRONMENT_PDF(hit_value.normal, ray_direction);
		const float mis_weight = EE_CAMERA_ENVIRONMENT_HIT_MIS_WEIGHT(hit_value.last_sample_pdf, environment_pdf);
		hit_value.color = EE_FUNC_ENV(ray_direction) * mis_weight;
	} else {
		hit_value.color = EE_SKY_COLOR(ray_direction);
	}
}
