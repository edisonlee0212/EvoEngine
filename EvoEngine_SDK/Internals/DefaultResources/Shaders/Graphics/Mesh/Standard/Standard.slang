// @evoengine-dialect native
import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;
import EvoEngine.Meshlet;
import EvoEngine.RasterVertex;

static const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint MESHLET_VERTEX_ITERATIONS =
    (EE_MESHLET_MAX_VERTICES_SIZE + WORKGROUP_SIZE - 1u) / WORKGROUP_SIZE;
static const uint MESHLET_PRIMITIVE_ITERATIONS =
    (EE_MESHLET_MAX_TRIANGLES_SIZE + WORKGROUP_SIZE - 1u) / WORKGROUP_SIZE;

struct EEMeshTaskPayload {
  uint instanceIndex;
  uint baseID;
  uint8_t deltaIDs[EXT_TASK_WORK_GROUP_INVOCATIONS];
};

[outputtopology("triangle")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("mesh")]
void main(in payload EEMeshTaskPayload task_input, uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex,
          OutputVertices<StandardRasterOutput, EE_MESHLET_MAX_VERTICES_SIZE> vertices,
          OutputIndices<uint3, EE_MESHLET_MAX_TRIANGLES_SIZE> triangles) {
  uint instance_index = task_input.instanceIndex;
  uint meshlet_id = task_input.baseID + uint(task_input.deltaIDs[group_id.x]);
  Meshlet meshlet = EE_MESHLETS[meshlet_id];
  uint vertex_count = meshlet.verticesSize;
  uint triangle_count = meshlet.triangleSize;
  SetMeshOutputCounts(vertex_count, triangle_count);
  float4x4 model = EE_INSTANCES[instance_index].model;
  float4x4 projection_view = EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].projection_view;

  [unroll]
  for (uint iteration = 0u; iteration < MESHLET_VERTEX_ITERATIONS; ++iteration) {
    uint vertex_index = lane_id + iteration * WORKGROUP_SIZE;
    if (vertex_index >= vertex_count) break;
    Vertex vertex = EE_VERTEX_DATA_CHUNKS[meshlet_id].vertices[vertex_index];
    vertices[vertex_index] = EE_BUILD_STANDARD_RASTER_OUTPUT(
        vertex.position, vertex.normal, vertex.tangent, vertex.vertex_info3, vertex.tex_coord, vertex.tex_coord_1,
        vertex.tex_coord_2, vertex.tex_coord_3, vertex.color, instance_index, model, projection_view);
  }

  [unroll]
  for (uint iteration = 0u; iteration < MESHLET_PRIMITIVE_ITERATIONS; ++iteration) {
    uint primitive_index = lane_id + iteration * WORKGROUP_SIZE;
    if (primitive_index < triangle_count) {
      triangles[primitive_index] =
          uint3(meshlet.indices[primitive_index * 3u], meshlet.indices[primitive_index * 3u + 1u],
                meshlet.indices[primitive_index * 3u + 2u]);
    }
  }
}
