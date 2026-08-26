import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;
import EcoSysLab.FloatOrderedInt;

#define INFINITY asfloat(0x7F800000)

// Rot type bitmask
static const uint WHITE_ROT = 1 << 0;
static const uint BROWN_ROT = 1 << 1;

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
  uint fungus_type;
};

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex, uint3 dispatch_thread_id : SV_DispatchThreadID) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint segment_index = baseID + local_index;

    if (segment_index >= segment_size)
      break;

    Segment segment = segments[segment_index];
    if (segment.screen_depth != INFINITY && min_distance_state[0].x == floatToOrderedInt(segment.screen_depth)) {
      // useful for debugging, but not needed
      // min_distance_state[0].y = segment_index;

      if ((fungus_type & WHITE_ROT) != 0) {
        segment.RW = segment.RW + injection_amount;
        segment.RW_pre = segment.RW;
      }

      if ((fungus_type & BROWN_ROT) != 0) {
        segment.RB = segment.RB + injection_amount;
        segment.RB_pre = segment.RB;
      }
    }
    segments[segment_index] = segment;
  }
}
