import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  float2 line_start;
  float2 line_end;
  float4x4 projection_view;
  uint segment_pair_size;
  uint cut_mode;
};

bool LineIntersect(in float2 A, in float2 B, in float2 C, in float2 D) {
  float2 r = B - A;  // Direction vector of segment AB
  float2 s = D - C;  // Direction vector of segment CD

  float crossRS = r.x * s.y - r.y * s.x;  // Cross product of r and s
  float2 diff = C - A;

  // If crossRS is zero, the lines are parallel or collinear
  if (abs(crossRS) < 1e-6)
    return false;

  float t = (diff.x * s.y - diff.y * s.x) / crossRS;
  float u = (diff.x * r.y - diff.y * r.x) / crossRS;

  // Check if t and u are within [0, 1]
  if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f) {
    // return A + t * r; // Compute intersection point
    return true;
  }
  return false;
}

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex, uint3 dispatch_thread_id : SV_DispatchThreadID) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint segment_pair_index = baseID + local_index;

    if (segment_pair_index >= segment_pair_size)
      break;

    SegmentPair segment_pair = segment_pairs[segment_pair_index];

    Segment segment0 = segments[segment_pair.segment0_handle];
    Segment segment1 = segments[segment_pair.segment1_handle];

    float3 segment0_center_position = (segment0.particle0.x + segment0.particle1.x) * 0.5f;
    float3 segment1_center_position = (segment1.particle0.x + segment1.particle1.x) * 0.5f;

    float4 segment0_screen_space_coord = mul(projection_view, float4(segment0_center_position, 1.0f));
    float3 segment0_screen_space_position = segment0_screen_space_coord.xyz / segment0_screen_space_coord.w;
    float4 segment1_screen_space_coord = mul(projection_view, float4(segment1_center_position, 1.0f));
    float3 segment1_screen_space_position = segment1_screen_space_coord.xyz / segment1_screen_space_coord.w;

    float2 segment0_screen =
        float2((segment0_screen_space_position.x + 1.0f) * 0.5f, (-segment0_screen_space_position.y + 1.0f) * 0.5f);
    float2 segment1_screen =
        float2((segment1_screen_space_position.x + 1.0f) * 0.5f, (-segment1_screen_space_position.y + 1.0f) * 0.5f);

    if (LineIntersect(segment0_screen, segment1_screen, line_start, line_end)) {
      if (cut_mode == 0)
        segment_pairs[segment_pair_index].connectivity_integrity = 0.f;
      segment_pairs[segment_pair_index].bend_twist_bundle_integrity = 0.f;
    }
  }
}