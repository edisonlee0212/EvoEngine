#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_mesh_shader : enable
#extension GL_ARB_shader_draw_parameters : enable
#extension GL_EXT_control_flow_attributes : require
#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.slangh"
#include "Basic.slangh"

const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;

const uint MESHLET_VERTEX_ITERATIONS    = ((MESHLET_MAX_VERTICES_SIZE	+ WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
const uint MESHLET_PRIMITIVE_ITERATIONS = ((MESHLET_MAX_TRIANGLES_SIZE	+ WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

struct EEMeshPositionOutput {
	vec4 position : SV_Position;
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
	OutputVertices<EEMeshPositionOutput, MESHLET_MAX_VERTICES_SIZE> vertices,
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
	mat4 projectionView = EE_DIRECTIONAL_LIGHTS[EE_CAMERA_INDEX].light_space_matrix[EE_LIGHT_SPLIT_INDEX] * model;
	[[unroll]]
	for (uint i = 0; i < uint(MESHLET_VERTEX_ITERATIONS); ++i)
	{
		uint vert = laneID + i * WORKGROUP_SIZE;

		if (vert >= verticesSize) break;

		Vertex v = EE_VERTEX_DATA_CHUNKS[meshletID].vertices[vert];
		vertices[vert].position = projectionView * vec4(v.position.xyz, 1.0);
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
