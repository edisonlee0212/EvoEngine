import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;
import EcoSysLab.FloatOrderedInt;
import EcoSysLab.LineCircleIntersect;

#define INFINITY asfloat(0x7F800000)

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::binding(0, 1)]] RWStructuredBuffer<int4, Std430DataLayout> min_distance_state;
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  float4x4 projection_view;
  float2 point;
  float2 screen_size;

  uint segment_size;
  float point_size;
  float injection_amount;
};

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex, uint3 dispatch_thread_id : SV_DispatchThreadID) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  float min_distance_to_screen = INFINITY;
  uint index_of_min = 0;

  GroupMemoryBarrierWithGroupSync();

  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint segment_index = baseID + local_index;

    if (segment_index >= segment_size)
      break;

    segments[segment_index].screen_depth = INFINITY;
    // only allow injection if there is no fungus already.
    // we set threshold to 0.5 * injection amount so small diffusions to surrounding segments will not prohibit
    // injection but the most recently injected segments are avoided.
    if (segments[segment_index].RW >= 0.5 * injection_amount)
      continue;

    Segment segment = segments[segment_index];

    float3 particle0_position = segment.particle0.x;
    float3 particle1_position = segment.particle1.x;

    float4 particle0_screen_space_coord = mul(projection_view, float4(particle0_position, 1.0f));
    float3 particle0_screen_space_position = particle0_screen_space_coord.xyz / particle0_screen_space_coord.w;
    float4 particle1_screen_space_coord = mul(projection_view, float4(particle1_position, 1.0f));
    float3 particle1_screen_space_position = particle1_screen_space_coord.xyz / particle1_screen_space_coord.w;

    float2 particle0_screen =
        float2((particle0_screen_space_position.x + 1.0f) * 0.5f, (-particle0_screen_space_position.y + 1.0f) * 0.5f) *
        screen_size;
    float2 particle1_screen =
        float2((particle1_screen_space_position.x + 1.0f) * 0.5f, (-particle1_screen_space_position.y + 1.0f) * 0.5f) *
        screen_size;

    if (LineCircleIntersect(particle0_screen, particle1_screen, point, point_size)) {
      float distance_to_screen = min(particle1_screen_space_position.z, particle0_screen_space_position.z);
      segments[segment_index].screen_depth = distance_to_screen;

      // find minimum among thread to avoid atomic operations
      if (distance_to_screen < min_distance_to_screen) {
        min_distance_to_screen = distance_to_screen;
        index_of_min = segment_index;
      }
    }
  }

  // now use subgroups to determine the minimum distance among all work groups
  float subgroup_min_distance = WaveActiveMin(min_distance_to_screen);
  if (WaveIsFirstLane()) {
    InterlockedMin(min_distance_state[0].x, floatToOrderedInt(subgroup_min_distance));
  }
}