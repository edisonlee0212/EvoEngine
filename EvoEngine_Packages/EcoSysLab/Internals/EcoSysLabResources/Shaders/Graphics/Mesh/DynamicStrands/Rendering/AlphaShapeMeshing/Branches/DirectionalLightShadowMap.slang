import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet1;
import EvoEngine.PerFrame;
import EcoSysLab.AlphaShapeRenderingConstants;

static const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;
#define TETRAHEDRON_VERTICES_SIZE 4
#define TETRAHEDRON_TRIANGLE_SIZE 4



static const uint TETRAHEDRON_VERTEX_ITERATIONS = ((TETRAHEDRON_VERTICES_SIZE + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
static const uint TETRAHEDRON_PRIMITIVE_ITERATIONS = ((TETRAHEDRON_TRIANGLE_SIZE + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

struct EcoSysLabMeshVertex {
  float4 position : SV_Position;
};

struct Task {
  uint baseID;
  uint8_t deltaIDs[EXT_TASK_WORK_GROUP_INVOCATIONS];
};

// order them such that the triangle index corresponds to the vertex index on the opposing side
static const uint3 triangles[] = {
    uint3(2, 1, 3),
    uint3(0, 2, 3),
    uint3(1, 0, 3),
    uint3(0, 1, 2),
};
// group_id.x runs from [0 .. parentTask.groupCountX - 1]
[outputtopology("triangle")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("mesh")]
void main(in payload Task ts_in, uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex,
          OutputVertices<EcoSysLabMeshVertex, TETRAHEDRON_VERTICES_SIZE> vertices,
          OutputIndices<uint3, TETRAHEDRON_TRIANGLE_SIZE> output_indices) {
  const uint laneID = lane_id;
  uint tet_id = ts_in.baseID + ts_in.deltaIDs[group_id.x];

  // report amount of accepted triangles
  SetMeshOutputCounts(TETRAHEDRON_VERTICES_SIZE, delaunay_tetrahedrons[tet_id].triangles_accepted);
  DelaunayTetrahedron tet = delaunay_tetrahedrons[tet_id];
  // set up vertex properties
  float4x4 transform = EE_DIRECTIONAL_LIGHTS[EE_CAMERA_INDEX].light_space_matrix[EE_INSTANCE_INDEX];

  [[unroll]]
  for (uint i = 0; i < uint(TETRAHEDRON_VERTEX_ITERATIONS); ++i) {
    uint vertex_index = laneID + i * WORKGROUP_SIZE;
    if (vertex_index >= TETRAHEDRON_VERTICES_SIZE)
      break;
    float3 vertex_position = uniform_particles[tet.indices[vertex_index]].position_t.xyz;

    vertices[vertex_index].position = mul(float4(vertex_position, 1.0), transform);
  }

  // assign all accepted triangles
  [[unroll]]
  for (uint i = 0; i < uint(TETRAHEDRON_PRIMITIVE_ITERATIONS); ++i) {
    uint triangle_index = laneID + i * WORKGROUP_SIZE;
    if (triangle_index < TETRAHEDRON_TRIANGLE_SIZE) {
      if (delaunay_tetrahedrons[tet_id].render_neighbor[triangle_index] == 1) {
        // count triangles downwards
        int index;
        InterlockedAdd(delaunay_tetrahedrons[tet_id].triangles_accepted, -1, index);
        output_indices[index - 1] = triangles[triangle_index];
      }
    }
  }
}
