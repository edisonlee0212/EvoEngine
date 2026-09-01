import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;
import EvoEngine.Meshlet;
#if defined(EE_ALPHA_MASKED_SHADOW)
import EvoEngine.ShadowRaster;
#define EE_SHADOW_MESH_OUTPUT ShadowRasterOutput
#else
#define EE_SHADOW_MESH_OUTPUT MeshPositionOutput
#endif

static const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint MESHLET_VERTEX_ITERATIONS =
    (EE_MESHLET_MAX_VERTICES_SIZE + WORKGROUP_SIZE - 1u) / WORKGROUP_SIZE;
static const uint MESHLET_PRIMITIVE_ITERATIONS =
    (EE_MESHLET_MAX_TRIANGLES_SIZE + WORKGROUP_SIZE - 1u) / WORKGROUP_SIZE;

struct MeshPositionOutput {
  float4 position : SV_Position;
};

struct EEMeshTaskPayload {
  uint instanceIndex;
  uint baseID;
  uint8_t deltaIDs[EXT_TASK_WORK_GROUP_INVOCATIONS];
};

[outputtopology("triangle")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("mesh")]
void main(in payload EEMeshTaskPayload task_input, uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex,
          OutputVertices<EE_SHADOW_MESH_OUTPUT, EE_MESHLET_MAX_VERTICES_SIZE> vertices,
          OutputIndices<uint3, EE_MESHLET_MAX_TRIANGLES_SIZE> triangles) {
  uint instance_index = task_input.instanceIndex;
  uint meshlet_id = task_input.baseID + uint(task_input.deltaIDs[group_id.x]);
  Meshlet meshlet = EE_MESHLETS[meshlet_id];
  uint vertex_count = meshlet.verticesSize;
  uint triangle_count = meshlet.triangleSize;
  SetMeshOutputCounts(vertex_count, triangle_count);
  float4x4 model = EE_INSTANCES[instance_index].model;
  float4x4 light_projection = EE_SPOT_LIGHTS[EE_BASIC_CONSTANTS.camera_index].light_space_matrix;

  [unroll]
  for (uint iteration = 0u; iteration < MESHLET_VERTEX_ITERATIONS; ++iteration) {
    uint vertex_index = lane_id + iteration * WORKGROUP_SIZE;
    if (vertex_index >= vertex_count) break;
    Vertex vertex = EE_VERTEX_DATA_CHUNKS[meshlet_id].vertices[vertex_index];
    float4 clip_position = mul(mul(float4(vertex.position, 1.0f), model), light_projection);
#if defined(EE_ALPHA_MASKED_SHADOW)
    vertices[vertex_index] = EE_BUILD_SHADOW_RASTER_OUTPUT(
        clip_position, vertex.tex_coord, vertex.tex_coord_1, vertex.color, instance_index);
#else
    vertices[vertex_index].position = clip_position;
#endif
  }
  [unroll]
  for (uint iteration = 0u; iteration < MESHLET_PRIMITIVE_ITERATIONS; ++iteration) {
    uint primitive_index = lane_id + iteration * WORKGROUP_SIZE;
    if (primitive_index < triangle_count)
      triangles[primitive_index] =
          uint3(meshlet.indices[primitive_index * 3u], meshlet.indices[primitive_index * 3u + 1u],
                meshlet.indices[primitive_index * 3u + 2u]);
  }
}
