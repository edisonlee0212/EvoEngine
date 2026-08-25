import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;
import EcoSysLab.KineticVoronoiMeshing;
import EcoSysLab.KineticVoronoiMeshingSet0;

#define EPSILON 1e-3
static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint triangle_count;
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
  for (uint i = 0; i < ITERATIONS_PER_TASK; ++i) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint triangle_index = baseID + local_index;

    if (triangle_index >= triangle_count)
      break;

    SegmentMeshletTriangle tri = segment_meshlet_triangles[triangle_index];

    [[unroll]]
    for (int j = 0; j < 3; j++) {
      uint vertex_index = tri.vertex_indices[j];
      SegmentMeshletVertex vertex = segment_meshlet_vertices[vertex_index];

      Segment segment = segments[vertex.segment_index];

      // Undo initial orientation
      float4 q0_inv = float4(-segment.q0.xyz, segment.q0.w);
      float3 normal_tmp = rotate_vec3(q0_inv, tri.normal0[j].xyz);

      // Apply current orientation
      tri.normal[j] = float4(rotate_vec3(segment.q, normal_tmp), 0.0);

      // If the neighbor is on the same strand, set a shift for the vertex
      if (tri.neighbor_segment_index >= 0) {
        Segment neighbor_segment = segments[tri.neighbor_segment_index];
        if (neighbor_segment.next_handle == vertex.segment_index ||
          segment.next_handle == tri.neighbor_segment_index) {
          SegmentPair segment_pair = segment_pairs[tri.segment_pair_index];
          if (segment_pair.connectivity_integrity == 0.0f && segment_pair.bend_twist_bundle_integrity == 0.0f) {
            segment_meshlet_vertices[vertex_index].shift = float3(-tri.normal[j].x, tri.normal[j].y, tri.normal[j].z);
          }
        }
      }
    }

    segment_meshlet_triangles[triangle_index] = tri;
  }
}
