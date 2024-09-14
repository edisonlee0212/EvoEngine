#extension GL_EXT_nonuniform_qualifier : enable
#extension GL_EXT_shader_explicit_arithmetic_types_int8 : require
#extension GL_ARB_shading_language_include : enable

#define EE_PER_FRAME_SET 0
#define EE_PER_PASS_SET 1
#define EE_PER_GROUP_SET 2
#define EE_PER_COMMAND_SET 3


struct RenderInfo{
	float shadow_split_0;
	float shadow_split_1;
	float shadow_split_2;
	float shadow_split_3;

	int shadow_sample_size;
	int pcss_blocker_search;
	float shadow_seam_fix;
	float gamma;

	float strand_subdivision_x;
	float strand_subdivision_y;
	int strand_subdivision_max_x;
	int strand_subdivision_max_y;

	int directional_light_size;
	int point_light_size;
	int spot_light_size;
	int brdf_lut_map_index;

	int debug_visualization;
	int padding0;
	int padding1;
	int padding2;
};

layout(set = EE_PER_FRAME_SET, binding = 0) uniform EE_RENDERING_SETTINGS_BLOCK
{
	RenderInfo EE_RENDER_INFO;
};

#define EE_ENVIRONMENTAL_BLOCK_BINDING 1
#include "Environment.glsl"

#define EE_CAMERA_BLOCK_BINDING 2
#include "Cameras.glsl"

#define EE_MATERIAL_BLOCK_BINDING 3
#include "Materials.glsl"

#define EE_INSTANCE_BLOCK_BINDING 4
#include "Instances.glsl"

layout(set = EE_PER_FRAME_SET, binding = 5) uniform EE_KERNEL_BLOCK {
  vec4 EE_UNIFORM_KERNEL[MAX_KERNEL_AMOUNT];
  vec4 EE_GAUSS_KERNEL[MAX_KERNEL_AMOUNT];
};

#define EE_DIRECTIONAL_LIGHT_BLOCK_BINDING 6
#define EE_POINT_LIGHT_BLOCK_BINDING 7
#define EE_SPOT_LIGHT_BLOCK_BINDING 8
#include "Lights.glsl"

#include "Vertex.glsl"

struct VertexDataChunk {
	Vertex vertices[MESHLET_MAX_VERTICES_SIZE];
};

layout(set = EE_PER_FRAME_SET, binding = 9) readonly buffer EE_VERTICES_BLOCK
{
	VertexDataChunk EE_VERTEX_DATA_CHUNKS[];
};

struct Meshlet {
	uint8_t indices[MESHLET_MAX_INDICES_SIZE];
	uint verticesSize;
	uint triangleSize;
	uint chunkIndex;
};

layout(set = EE_PER_FRAME_SET, binding = 10) readonly buffer EE_MESHLETS_BLOCK
{
	Meshlet EE_MESHLETS[];
};

#define EE_TEXTURE_2DS_BINDING 13
#define EE_CUBEMAPS_BINDING 14
#include "Textures.glsl"

struct InstancedData {
	mat4 instance_matrix;
	vec4 color;
};

layout(set = EE_PER_PASS_SET, binding = 18) readonly buffer EE_ANIM_BONES_BLOCK
{
	mat4 EE_ANIM_BONES[];
};

layout(set = EE_PER_PASS_SET, binding = 18) readonly buffer EE_INSTANCED_DATA_BLOCK
{
	InstancedData EE_INSTANCED_DATA[];
};

int EE_STRANDS_SEGMENT_SUBDIVISION(in vec3 worldPosA, in vec3 worldPosB) {
	vec4 coordA = EE_CAMERAS[EE_CAMERA_INDEX].projection_view * vec4(worldPosA, 1.0);
	vec4 coordB = EE_CAMERAS[EE_CAMERA_INDEX].projection_view * vec4(worldPosB, 1.0);
		vec2 screenSize = vec2(EE_CAMERA_RESOLUTION_X(EE_CAMERA_INDEX), EE_CAMERA_RESOLUTION_Y(EE_CAMERA_INDEX));
	coordA = coordA / coordA.w;
	coordB = coordB / coordB.w;
	if (coordA.z < -1.0 && coordB.z < -1.0) return 0;
	float pixelDistance = distance(coordA.xy * screenSize / 2.0, coordB.xy * screenSize / 2.0);
	return max(1, min(EE_RENDER_INFO.strand_subdivision_max_x, int(pixelDistance / EE_RENDER_INFO.strand_subdivision_x)));
}

void EE_SPLINE_INTERPOLATION(in vec3 v0, in vec3 v1, in vec3 v2, in vec3 v3, out vec3 result, out vec3 tangent, float u)
{
	vec3 p0 = (v2 + v0) / 6.0 + v1 * (4.0 / 6.0);
	vec3 p1 = v2 - v0;
	vec3 p2 = v2 - v1;
	vec3 p3 = v3 - v1;
	float uu = u * u;
	float u3 = (1.0f / 6.0) * uu * u;
	vec3 q = vec3(u3 + 0.5 * (u - uu), uu - 4.0 * u3, u3);
	result = p0 + q.x * p1 + q.y * p2 + q.z * p3;
	if (u == 0.0)
		u = 0.000001;
	if (u == 1.0)
		u = 0.999999;
	float v = 1.0 - u;
	tangent = 0.5 * v * v * p1 + 2.0 * v * u * p2 + 0.5 * u * u * p3;
}

void EE_SPLINE_INTERPOLATION(in float v0, in float v1, in float v2, in float v3, out float result, out float tangent, float u)
{
	float p0 = (v2 + v0) / 6.0 + v1 * (4.0 / 6.0);
	float p1 = v2 - v0;
	float p2 = v2 - v1;
	float p3 = v3 - v1;
	float uu = u * u;
	float u3 = (1.0f / 6.0) * uu * u;
	vec3 q = vec3(u3 + 0.5 * (u - uu), uu - 4.0 * u3, u3);
	result = p0 + q.x * p1 + q.y * p2 + q.z * p3;
	if (u == 0.0)
		u = 0.000001;
	if (u == 1.0)
		u = 0.999999;
	float v = 1.0 - u;
	tangent = 0.5 * v * v * p1 + 2.0 * v * u * p2 + 0.5 * u * u * p3;
}

void EE_SPLINE_INTERPOLATION(in vec4 v0, in vec4 v1, in vec4 v2, in vec4 v3, out vec4 result, out vec4 tangent, float u)
{
	vec4 p0 = (v2 + v0) / 6.0 + v1 * (4.0 / 6.0);
	vec4 p1 = v2 - v0;
	vec4 p2 = v2 - v1;
	vec4 p3 = v3 - v1;
	float uu = u * u;
	float u3 = (1.0f / 6.0) * uu * u;
	vec3 q = vec3(u3 + 0.5 * (u - uu), uu - 4.0 * u3, u3);
	result = p0 + q.x * p1 + q.y * p2 + q.z * p3;
	if (u == 0.0)
		u = 0.000001;
	if (u == 1.0)
		u = 0.999999;
	float v = 1.0 - u;
	tangent = 0.5 * v * v * p1 + 2.0 * v * u * p2 + 0.5 * u * u * p3;
}

int EE_STRANDS_RING_SUBDIVISION(in mat4 model, in vec3 worldPos, in vec3 modelPos, in float thickness)
{
	vec3 modelPosB = thickness * vec3(0, 1, 0) + modelPos;
	vec3 endPointWorldPos = (model * vec4(modelPosB, 1.0)).xyz;
	vec3 redirectedWorldPosB = worldPos + EE_CAMERA_UP(EE_CAMERA_INDEX) * distance(worldPos, endPointWorldPos);
	float subdivision = EE_PIXEL_DISTANCE(EE_CAMERA_INDEX, worldPos, redirectedWorldPosB);
	return max(3, min(int(subdivision), EE_RENDER_INFO.strand_subdivision_max_y));
}