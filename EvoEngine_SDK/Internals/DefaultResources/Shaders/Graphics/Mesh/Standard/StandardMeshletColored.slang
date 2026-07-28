#extension GL_EXT_mesh_shader : enable
#extension GL_ARB_shader_draw_parameters : enable
#extension GL_EXT_control_flow_attributes : require
#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.slangh"
#include "Basic.slangh"

const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;

const uint MESHLET_VERTEX_ITERATIONS    = ((MESHLET_MAX_VERTICES_SIZE	+ WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
const uint MESHLET_PRIMITIVE_ITERATIONS = ((MESHLET_MAX_TRIANGLES_SIZE	+ WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

struct EEMeshRasterOutput {
	vec4 position : SV_Position;
	[[vk::location(0)]]
	vec3 FragPos;
	[[vk::location(1)]]
	vec3 Normal;
	[[vk::location(2)]]
	vec3 Tangent;
	[[vk::location(3)]]
	nointerpolation float TangentHandedness;
	[[vk::location(4)]]
	vec4 TexCoord01;
	[[vk::location(5)]]
	vec4 TexCoord23;
	[[vk::location(6)]]
	vec4 Color;
	[[vk::location(7)]]
	nointerpolation uint currentInstanceIndex;
	[[vk::location(8)]]
	nointerpolation float transformHandedness;
};

struct EEMeshTaskPayload{
	uint	instanceIndex;
	uint	baseID;
	uint8_t	deltaIDs[EXT_TASK_WORK_GROUP_INVOCATIONS];
};

[outputtopology("triangle")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("mesh")]
void main(in payload EEMeshTaskPayload ts_in,
	OutputVertices<EEMeshRasterOutput, MESHLET_MAX_VERTICES_SIZE> vertices,
	OutputIndices<uint3, MESHLET_MAX_TRIANGLES_SIZE> triangles)
{
	uint instanceIndex = ts_in.instanceIndex;
	uint meshletID = ts_in.baseID + ts_in.deltaIDs[gl_WorkGroupID.x];
	uint laneID = gl_LocalInvocationID.x;

	mat4 model = EE_INSTANCES[instanceIndex].model;

	Meshlet meshlet = EE_MESHLETS[meshletID];

	uint verticesSize = meshlet.verticesSize;
	uint triangleSize = meshlet.triangleSize;
	SetMeshOutputCounts(verticesSize, triangleSize);
	mat4 projectionView = EE_CAMERAS[EE_CAMERA_INDEX].projection_view;
	[[unroll]]
	for (uint i = 0; i < uint(MESHLET_VERTEX_ITERATIONS); ++i)
	{
		uint vert = laneID + i * WORKGROUP_SIZE;

		if (vert >= verticesSize) break;

		Vertex v = EE_VERTEX_DATA_CHUNKS[meshletID].vertices[vert];
		vec3 N = normalize(transpose(inverse(mat3(model))) * v.normal);
		vec3 T = mat3(model) * v.tangent;
		vec3 fragPos = vec3(model * vec4(v.position.xyz, 1.0));
		float handedness = EE_TRANSFORM_HANDEDNESS(model);
		vertices[vert].FragPos = fragPos;
		vertices[vert].Normal = N;
		vertices[vert].Tangent = T;
		vertices[vert].transformHandedness = handedness;
		vertices[vert].TangentHandedness =
			(v.vertex_info3 < 0.0 ? -1.0 : 1.0) * handedness;
		vertices[vert].TexCoord01 = vec4(v.tex_coord, v.tex_coord_1);
		vertices[vert].TexCoord23 = vec4(v.tex_coord_2, v.tex_coord_3);
		vertices[vert].Color = v.color;
		vertices[vert].position = projectionView * vec4(fragPos, 1.0);
		vertices[vert].currentInstanceIndex = instanceIndex;
	}
	[[unroll]]
	for (uint i = 0; i < uint(MESHLET_PRIMITIVE_ITERATIONS); ++i)
	{
		uint prim = laneID + i * WORKGROUP_SIZE;
		if(prim <= triangleSize){
			triangles[prim] =
				uint3(
					meshlet.indices[prim * 3],
					meshlet.indices[prim * 3 + 1],
					meshlet.indices[prim * 3 + 2]);
		}
	}
}
