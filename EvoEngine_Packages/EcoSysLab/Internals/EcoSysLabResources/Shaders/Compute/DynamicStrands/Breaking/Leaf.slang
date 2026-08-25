import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.DynamicStrandsPhysics;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
	uint leaf_size;
        uint leaf_break_from_moisture;
        float leaf_break_threshold;
};

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint leaf_index = baseID + local_index;

    if (leaf_index >= leaf_size)
      break;

		Leaf leaf = foliage[leaf_index];
                Segment segment = segments[leaf.segment_handle];
                float multiplier = 1.0f;
                uint seed = leaf_index;
                float r = rand_ab(seed, 0.0, 0.4);
                if (leaf_break_from_moisture == 1) {
                  multiplier = clamp(
                      max(-r + segment.moisture_pre - leaf_break_threshold, 0.0) / (1.0 - leaf_break_threshold), 0.0f, 1.0f);
                  leaf.health = multiplier;
                }


		if(leaf.position_strain > leaf.position_strain_limit * multiplier){
			leaf.attachment_integrity = 0.0f;
			foliage[leaf_index] = leaf;
		}
                //if (leaf_break_from_moisture == 1) {
                //  if (segment.moisture_pre < leaf_break_threshold) {
                //    leaf.attachment_integrity = 0.0f;
                //    foliage[leaf_index] = leaf;
                //  }
                //}
	}
}
