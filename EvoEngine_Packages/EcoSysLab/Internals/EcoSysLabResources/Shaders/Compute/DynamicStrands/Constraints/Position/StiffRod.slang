import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.DynamicStrandsPhysics;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint strand_size;
  float inv_time_step;
  uint frame_index;
};
[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint strand_index = baseID + local_index;

    if (strand_index >= strand_size)
      break;

    Strand strand = strands[strand_index];
    int front_propagate_handle = frame_index % 2 == 0 ? strand.front_propagate_begin_segment_handle
                                                      : strand.alternative_front_propagate_begin_segment_handle;
    int back_propagate_handle = frame_index % 2 == 0 ? strand.back_propagate_begin_segment_handle
                                                     : strand.alternative_back_propagate_begin_segment_handle;

    while (front_propagate_handle != -1 || back_propagate_handle != -1) {
      int prev_segment_pair_handle = segment_connection_handles[front_propagate_handle].handles[0];
      if (prev_segment_pair_handle != -1) {
        project_bend_twist_constraint(inv_time_step, prev_segment_pair_handle);
      }
      if (front_propagate_handle != -1) {
        project_shear_stretch_constraint(inv_time_step, front_propagate_handle);
        front_propagate_handle = segments[front_propagate_handle].next_handle;
        if (front_propagate_handle != -1)
          front_propagate_handle = segments[front_propagate_handle].next_handle;
      }

      int next_segment_pair_handle = segment_connection_handles[back_propagate_handle].handles[1];
      if (next_segment_pair_handle != -1) {
        project_bend_twist_constraint(inv_time_step, next_segment_pair_handle);
      }
      if (back_propagate_handle != -1) {
        project_shear_stretch_constraint(inv_time_step, back_propagate_handle);
        back_propagate_handle = segments[back_propagate_handle].prev_handle;
        if (back_propagate_handle != -1)
          back_propagate_handle = segments[back_propagate_handle].prev_handle;
      }
    }
  }
}
