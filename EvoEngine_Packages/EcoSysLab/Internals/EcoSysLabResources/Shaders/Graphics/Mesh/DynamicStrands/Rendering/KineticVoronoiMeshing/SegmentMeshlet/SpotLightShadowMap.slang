import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EcoSysLab.KineticVoronoiMeshingSet1;
import EvoEngine.PerFrame;
import EcoSysLab.Math;
import EcoSysLab.KineticVoronoiMeshing;
import EcoSysLab.KineticVoronoiRenderingConstants;

static const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const float PI = 3.1415926535897932384626433832795;
#define MAX_VERTEX_COUNT 3  // individual vertices for each triangle
#define MAX_PRIMITIVE_COUNT 1



static const uint TETRAHEDRON_VERTEX_ITERATIONS = ((MAX_VERTEX_COUNT + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
static const uint TETRAHEDRON_PRIMITIVE_ITERATIONS = ((MAX_PRIMITIVE_COUNT + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

struct EcoSysLabMeshVertex {
  float4 position : SV_Position;
};

struct Task {
  uint baseID;
  uint8_t deltaIDs[EXT_TASK_WORK_GROUP_INVOCATIONS];
};
// group_id.x runs from [0 .. parentTask.groupCountX - 1]
void SetUpTriangleVertex(OutputVertices<EcoSysLabMeshVertex, MAX_VERTEX_COUNT> vertices, uint vertex_index, in SegmentMeshletVertex segment_meshlet_vertex) {
  float4x4 transform = EE_SPOT_LIGHTS[EE_CAMERA_INDEX].light_space_matrix;

  float3 pos = segment_meshlet_vertex.x - fracture_distance * segment_meshlet_vertex.shift;
  vertices[vertex_index].position = mul(float4(pos, 1.0), transform);
}

[outputtopology("triangle")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("mesh")]
void main(in payload Task ts_in, uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex,
          OutputVertices<EcoSysLabMeshVertex, MAX_VERTEX_COUNT> vertices,
          OutputIndices<uint3, MAX_PRIMITIVE_COUNT> output_indices) {
  const uint laneID = lane_id;
  uint tri_id = ts_in.baseID + ts_in.deltaIDs[group_id.x];

  SegmentMeshletTriangle tri = segment_meshlet_triangles[tri_id];
  // report amount of accepted triangles
  SetMeshOutputCounts(MAX_VERTEX_COUNT, 1);

  // assign all accepted triangles
  [[unroll]]
  for (uint i = 0; i < uint(TETRAHEDRON_PRIMITIVE_ITERATIONS); ++i) {
    uint triangle_index = laneID + i * WORKGROUP_SIZE;  // should always be 0 in our setup
    if (triangle_index < MAX_PRIMITIVE_COUNT) {
      [[unroll]]
      for (int j = 0; j < 3; j++) {
        uint global_vertex_index = tri.vertex_indices[j];
        SegmentMeshletVertex segment_meshlet_vertex = segment_meshlet_vertices[global_vertex_index];
        SetUpTriangleVertex(vertices, j, segment_meshlet_vertex);
      }
    }
  }
  output_indices[0] = uint3(0, 1, 2);
}
