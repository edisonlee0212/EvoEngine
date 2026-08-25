import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;
import EcoSysLab.KineticVoronoiMeshing;
import EcoSysLab.KineticVoronoiMeshingSet0;

#define EPSILON 1e-3
static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint vertices_size;
  int padding0;
  int padding1;
  int padding2;
};

float3 boneApply(float3 pos0,  // vertex at t = 0
               float3 P0_0,  // bone start at t = 0
               float3 P0_t,  // bone start at time t
               float4 q0,    // initial rotation
               float4 qt     // rotation at time t
) {
  // Move to bone-local space
  float3 v = pos0 - P0_0;

  // Undo initial orientation
  float4 q0_inv = float4(-q0.xyz, q0.w);
  v = rotate_vec3(q0_inv, v);

  // Apply current orientation
  v = rotate_vec3(qt, v);

  // Move to world space
  return v + P0_t;
}

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint vertex_index = baseID + local_index;

    if (vertex_index >= vertices_size)
      break;
    SegmentMeshletVertex vertex = segment_meshlet_vertices[vertex_index];

    Segment segment = segments[vertex.segment_index];

    vertex.x = boneApply(float3(vertex.x0), segment.particle0.x0, segment.particle0.x, segment.q0, segment.q);

    segment_meshlet_vertices[vertex_index] = vertex;
  }
}
