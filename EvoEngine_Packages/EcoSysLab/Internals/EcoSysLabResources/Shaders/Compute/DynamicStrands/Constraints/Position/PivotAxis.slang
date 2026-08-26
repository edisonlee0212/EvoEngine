import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
struct SegmentUpdate {
  float axis_offset;
  float axis_distance;
  uint segment_index;
  uint padding;
};

[[vk::binding(0, 1)]] StructuredBuffer<SegmentUpdate, Std430DataLayout> segment_updates;

[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  float3 pivot_position;
  uint command_size;
  float3 axis;
};

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint global_index = baseID + local_index;

    if (global_index >= command_size)
      break;

    SegmentUpdate segment_update = segment_updates[global_index];
    uint segment_index = segment_update.segment_index;
    Segment segment = segments[segment_index];
    Particle segment_particle0 = segment_particle0s[segment_index];
    Particle segment_particle1 = segment_particle1s[segment_index];

    float3 front_direction = normalize(rotate_vec3(segment.q, float3(0, 0, -1)));

    float current_segment_length = distance(segment_particle0.x, segment_particle1.x);
    float3 segment_center_position = (segment_particle0.x + segment_particle1.x) * 0.5f;

    float3 axis_position = pivot_position + segment_update.axis_offset * axis;
    float3 line_projection = dot(segment_center_position - pivot_position, axis) * axis;
    float3 diff = segment_center_position - pivot_position - line_projection;
    float3 direction = length(diff) == 0.0f ? front_direction : normalize(diff);
    float3 desired_segment_center_position = axis_position + segment_update.axis_distance * direction;

    float3 offset = front_direction * 0.5f * current_segment_length;
    segment_particle0.x = segment_particle0.last_x = desired_segment_center_position - offset;
    segment_particle1.x = segment_particle1.last_x = desired_segment_center_position + offset;
    segments[segment_index] = segment;
    segment_particle0s[segment_index] = segment_particle0;
    segment_particle1s[segment_index] = segment_particle1;
  }
}
