import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
struct SegmentUpdate {
  float4 new_rotation;
  float3 new_particle0_position;
  uint fix_particle0;
  float3 new_particle1_position;
  uint fix_particle1;

  uint segment_index;
  uint padding0;
  uint padding1;
  uint padding2;
};

[[vk::binding(0, 1)]] StructuredBuffer<SegmentUpdate, Std430DataLayout> segment_updates;

[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
	uint command_size;
	float ring_radius;
	float HC_threshold;
	float HL_threshold;
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

    // continue;

		SegmentUpdate segment_update = segment_updates[global_index];
		uint segment_index = segment_update.segment_index;
                if (segment_index == 4294967295u)
                        continue;
		Segment segment = segments[segment_index];
		Particle segment_particle0 = segment_particle0s[segment_index];
		Particle segment_particle1 = segment_particle1s[segment_index];
                if (segment.boundary_distance > ring_radius || segment.unlink_constraint == 1)
                        continue;
                if (segment.HC < HC_threshold || segment.HL < HL_threshold)
                //if (segment.HC < 0.1 + max(0.3 - segment.boundary_distance, 0.0))
                  segment.unlink_constraint = 1;
		segment.q = segment.last_q = segment_update.new_rotation;
		segment.torque = float3(0, 0, 0);
		segment.inv_mass = 0.0f;
		if(segment_update.fix_particle0 == 1){
			segment_particle0.x = segment_particle0.last_x = segment_update.new_particle0_position;
			segment_particle0.acceleration.xyz = float3(0, 0, 0);
		}
		if(segment_update.fix_particle1 == 1){
			segment_particle1.x = segment_particle1.last_x = segment_update.new_particle1_position;
			segment_particle1.acceleration.xyz = float3(0, 0, 0);
		}
		segments[segment_index] = segment;
		segment_particle0s[segment_index] = segment_particle0;
		segment_particle1s[segment_index] = segment_particle1;
	}
}
