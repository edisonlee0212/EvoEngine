import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;
import EcoSysLab.LineCircleIntersect;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  float4x4 projection_view;
  float2 point;
  float2 screen_size;

  uint segment_pair_size;
  uint cut_mode;
  float point_size;
};

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
        float2((segment0_screen_space_position.x + 1.0f) * 0.5f, (-segment0_screen_space_position.y + 1.0f) * 0.5f) *
        screen_size;
    float2 segment1_screen =
        float2((segment1_screen_space_position.x + 1.0f) * 0.5f, (-segment1_screen_space_position.y + 1.0f) * 0.5f) *
        screen_size;

    if (LineCircleIntersect(segment0_screen, segment1_screen, point, point_size)) {
      if (cut_mode == 0)
        segment_pairs[segment_pair_index].connectivity_integrity = 0.f;
      segment_pairs[segment_pair_index].bend_twist_bundle_integrity = 0.f;
    }
  }
}