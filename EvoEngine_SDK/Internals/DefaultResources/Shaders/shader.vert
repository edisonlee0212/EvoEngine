#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec2 inTexCoord;

layout(location = 0) out vec3 fragColor;
layout(location = 1) out vec2 fragTexCoord;


layout(set = 0, binding = 0) uniform EE_RENDERING_SETTINGS_BLOCK
{
	float EE_SHADOW_SPLIT_0;
	float EE_SHADOW_SPLIT_1;
	float EE_SHADOW_SPLIT_2;
	float EE_SHADOW_SPLIT_3;

	int EE_SHADOW_SAMPLE_SIZE;
	int EE_SHADOW_PCSS_BLOCKER_SEARCH_SIZE;
	float EE_SHADOW_SEAM_FIX_RATIO;
	float EE_GAMMA;

	float EE_STRANDS_SUBDIVISION_X_FACTOR;
	float EE_STRANDS_SUBDIVISION_Y_FACTOR;
	int EE_STRANDS_SUBDIVISION_MAX_X;
	int EE_STRANDS_SUBDIVISION_MAX_Y;
};

layout(set = 0, binding = 1) uniform EE_ENVIRONMENTAL_BLOCK
{
	vec4 EE_ENVIRONMENTAL_BACKGROUND_COLOR;
	float EE_ENVIRONMENTAL_MAP_GAMMA;
	float EE_ENVIRONMENTAL_LIGHTING_INTENSITY;
	float EE_BACKGROUND_INTENSITY;
	float EE_ENVIRONMENTAL_PADDING2;
};


//Camera
layout (set = 1, binding = 2) uniform EE_CAMERA
{
	mat4 EE_CAMERA_PROJECTION;
	mat4 EE_CAMERA_VIEW;
	mat4 EE_CAMERA_PROJECTION_VIEW;
	mat4 EE_CAMERA_INVERSE_PROJECTION;
	mat4 EE_CAMERA_INVERSE_VIEW;
	mat4 EE_CAMERA_INVERSE_PROJECTION_VIEW;
	vec4 EE_CAMERA_CLEAR_COLOR;
	vec4 EE_CAMERA_RESERVED1;
	vec4 EE_CAMERA_RESERVED2;
};



layout(set = 2, binding = 3) uniform EE_MATERIAL_BLOCK
{
	bool EE_ALBEDO_MAP_ENABLED;
	bool EE_NORMAL_MAP_ENABLED;
	bool EE_METALLIC_MAP_ENABLED;
	bool EE_ROUGHNESS_MAP_ENABLED;
	bool EE_AO_MAP_ENABLED;
	bool EE_CAST_SHADOW;
	bool EE_RECEIVE_SHADOW;
	bool EE_ENABLE_SHADOW;

	vec4 EE_PBR_ALBEDO;
	vec4 EE_PBR_SSSC;
	vec4 EE_PBR_SSSR;
	float EE_PBR_METALLIC;
	float EE_PBR_ROUGHNESS;
	float EE_PBR_AO;
	float EE_PBR_EMISSION;
};


void main() {
    gl_Position = vec4(inPosition, 1.0);
    fragColor = inColor;
}