#extension GL_EXT_nonuniform_qualifier : enable
#extension GL_EXT_shader_explicit_arithmetic_types_int8 : require

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

struct Environment {
	vec4 background_color;
	float gamma;
	float light_intensity;
	float padding1;
	float padding2;
};

layout(set = EE_PER_FRAME_SET, binding = 1) uniform EE_ENVIRONMENTAL_BLOCK
{
	Environment EE_ENVIRONMENT;
};

struct Camera {
	mat4 projection;
	mat4 view;
	mat4 projection_view;
	mat4 inverse_projection;
	mat4 inverse_view;
	mat4 interse_projection_view;
	vec4 clear_color;
	vec4 reserved1;
	vec4 reserved_2;
	int skybox_tex_index;
	int irradiance_map_index;
	int prefiltered_map_index;
	int use_clear_color;
};

//Camera
layout(set = EE_PER_FRAME_SET, binding = 2) readonly buffer EE_CAMERA_BLOCK
{
	Camera EE_CAMERAS[];
};

struct MaterialProperties {
	int albedo_map_index;
	int normal_map_index;
	int metallic_map_index;
	int roughness_map_index;

	int ao_texture_index;
	bool cast_shadow;
	bool receive_shadow;
	bool enable_shadow;

	vec4 albedo;
	vec4 sss_c;
	vec4 sss_r;

	float metallic;
	float roughness;
	float ambient_occulusion;
	float emission;
};

layout(set = EE_PER_FRAME_SET, binding = 3) readonly buffer EE_MATERIAL_BLOCK
{
	MaterialProperties EE_MATERIAL_PROPERTIES[];
};

struct Instance {
	mat4 model;
	uint material_index;
	uint info_index;
	uint meshlet_offset;
	uint meshlet_size;
};

layout(set = EE_PER_FRAME_SET, binding = 4) readonly buffer EE_INSTANCE_BLOCK
{
	Instance EE_INSTANCES[];
};

//Lights
struct DirectionalLight {
	vec3 direction;
	vec4 diffuse;
	vec3 specular;
	mat4 light_space_matrix[4];
	vec4 light_frustum_width;
	vec4 light_frustum_distance;
	vec4 reserved_parameters;
	int viewport_x_offset;
	int viewport_y_offset;
	int viewport_x_size;
	int viewport_y_size;
};

struct PointLight {
	vec3 position;
	vec4 constant_linear_quadratic_far;
	vec4 diffuse;
	vec3 specular;
	mat4 light_space_matrix[6];
	vec4 reserved_parameters;
	int viewport_x_offset;
	int viewport_y_offset;
	int viewport_x_size;
	int viewport_y_size;
};

struct SpotLight {
	vec3 position;
	float padding0;

	vec3 direction;
	float padding1;

	mat4 light_space_matrix;
	vec4 cutoff_outer_inner_size_bias;
	vec4 constant_linear_quadratic_far;
	vec4 diffuse;
	vec3 specular;
	float padding3;
	int viewport_x_offset;
	int viewport_y_offset;
	int viewport_x_size;
	int viewport_y_size;
};
layout(set = EE_PER_FRAME_SET, binding = 5) uniform EE_KERNEL_BLOCK
{
	vec4 EE_UNIFORM_KERNEL[MAX_KERNEL_AMOUNT];
	vec4 EE_GAUSS_KERNEL[MAX_KERNEL_AMOUNT];
};

layout(set = EE_PER_FRAME_SET, binding = 6) readonly buffer EE_DIRECTIONAL_LIGHT_BLOCK
{
	DirectionalLight EE_DIRECTIONAL_LIGHTS[];
};

layout(set = EE_PER_FRAME_SET, binding = 7) readonly buffer EE_POINT_LIGHT_BLOCK
{
	PointLight EE_POINT_LIGHTS[];
};

layout(set = EE_PER_FRAME_SET, binding = 8) readonly buffer EE_SPOT_LIGHT_BLOCK
{
	SpotLight EE_SPOT_LIGHTS[];
};

struct Vertex {
	vec3 position;
	vec3 normal;
	vec3 tangent;
	
	vec4 color;
	vec2 tex_coord;
	vec2 vertex_info;
};

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

layout(set = EE_PER_FRAME_SET, binding = 13) uniform sampler2D[] EE_TEXTURE_2DS;
layout(set = EE_PER_FRAME_SET, binding = 14) uniform samplerCube[] EE_CUBEMAPS;

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

vec3 EE_DEPTH_TO_CLIP_POS(vec2 tex_coords, float ndcDepth);
vec3 EE_DEPTH_TO_WORLD_POS(vec2 tex_coords, float ndcDepth);
vec3 EE_DEPTH_TO_VIEW_POS(vec2 tex_coords, float ndcDepth);

vec3 EE_CAMERA_LEFT() {
	return EE_CAMERAS[EE_CAMERA_INDEX].view[0].xyz;
}

vec3 EE_CAMERA_RIGHT() {
	return -EE_CAMERAS[EE_CAMERA_INDEX].view[0].xyz;
}

vec3 EE_CAMERA_UP() {
	return EE_CAMERAS[EE_CAMERA_INDEX].view[1].xyz;
}

vec3 EE_CAMERA_DOWN() {
	return -EE_CAMERAS[EE_CAMERA_INDEX].view[1].xyz;
}

vec3 EE_CAMERA_BACK() {
	return EE_CAMERAS[EE_CAMERA_INDEX].view[2].xyz;
}

vec3 EE_CAMERA_FRONT() {
	return -EE_CAMERAS[EE_CAMERA_INDEX].view[2].xyz;
}

vec3 EE_CAMERA_POSITION() {
	return EE_CAMERAS[EE_CAMERA_INDEX].inverse_view[3].xyz;
}

float EE_CAMERA_NEAR() {
	return EE_CAMERAS[EE_CAMERA_INDEX].reserved1.x;
}

float EE_CAMERA_FAR() {
	return EE_CAMERAS[EE_CAMERA_INDEX].reserved1.y;
}

float EE_CAMERA_TAN_FOV() {
	return EE_CAMERAS[EE_CAMERA_INDEX].reserved1.z;
}

float EE_CAMERA_TAN_HALF_FOV() {
	return EE_CAMERAS[EE_CAMERA_INDEX].reserved1.w;
}

float EE_CAMERA_RESOLUTION_X() {
	return EE_CAMERAS[EE_CAMERA_INDEX].reserved_2.x;
}

float EE_CAMERA_RESOLUTION_Y() {
	return EE_CAMERAS[EE_CAMERA_INDEX].reserved_2.y;
}

float EE_CAMERA_RESOLUTION_RATIO() {
	return EE_CAMERAS[EE_CAMERA_INDEX].reserved_2.z;
}

float EE_LINEARIZE_DEPTH(float ndcDepth)
{
	float near = EE_CAMERA_NEAR();
	float far = EE_CAMERA_FAR();
	return near * far / (far - ndcDepth * (far - near));
}

vec3 EE_DEPTH_TO_WORLD_POS(vec2 tex_coords, float ndcDepth) {
	vec4 viewPos = vec4(EE_DEPTH_TO_VIEW_POS(tex_coords, ndcDepth), 1.0);
	vec4 worldPos = EE_CAMERAS[EE_CAMERA_INDEX].inverse_view * viewPos;
	worldPos = worldPos / worldPos.w;
	return worldPos.xyz;
}

vec3 EE_DEPTH_TO_VIEW_POS(vec2 tex_coords, float ndcDepth) {
	vec4 clipPos = vec4(EE_DEPTH_TO_CLIP_POS(tex_coords, ndcDepth), 1.0);
	vec4 viewPos = EE_CAMERAS[EE_CAMERA_INDEX].inverse_projection * clipPos;
	viewPos = viewPos / viewPos.w;
	return viewPos.xyz;
}

vec3 EE_DEPTH_TO_CLIP_POS(vec2 tex_coords, float ndcDepth) {
	vec4 clipPos = vec4(tex_coords * 2 - vec2(1), ndcDepth, 1.0);
	return clipPos.xyz;
}


float EE_PIXEL_DISTANCE(in vec3 worldPosA, in vec3 worldPosB) {
	vec4 coordA = EE_CAMERAS[EE_CAMERA_INDEX].projection_view * vec4(worldPosA, 1.0);
	vec4 coordB = EE_CAMERAS[EE_CAMERA_INDEX].projection_view * vec4(worldPosB, 1.0);
	vec2 screenSize = vec2(EE_CAMERA_RESOLUTION_X(), EE_CAMERA_RESOLUTION_Y());
	coordA = coordA / coordA.w;
	coordB = coordB / coordB.w;
	return distance(coordA.xy * screenSize / 2.0, coordB.xy * screenSize / 2.0);
}

int EE_STRANDS_SEGMENT_SUBDIVISION(in vec3 worldPosA, in vec3 worldPosB) {
	vec4 coordA = EE_CAMERAS[EE_CAMERA_INDEX].projection_view * vec4(worldPosA, 1.0);
	vec4 coordB = EE_CAMERAS[EE_CAMERA_INDEX].projection_view * vec4(worldPosB, 1.0);
	vec2 screenSize = vec2(EE_CAMERA_RESOLUTION_X(), EE_CAMERA_RESOLUTION_Y());
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
	vec3 redirectedWorldPosB = worldPos + EE_CAMERA_UP() * distance(worldPos, endPointWorldPos);
	float subdivision = EE_PIXEL_DISTANCE(worldPos, redirectedWorldPosB);
	return max(3, min(int(subdivision), EE_RENDER_INFO.strand_subdivision_max_y));
}