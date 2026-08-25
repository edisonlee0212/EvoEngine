import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint ITERATIONS_PER_TASK =
    (COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE;

struct PushConstants {
  uint segment_size;
};

[[vk::push_constant]] ConstantBuffer<PushConstants> constants;

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint base_id = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  [[unroll]] for (uint i = 0; i < ITERATIONS_PER_TASK; ++i) {
    const uint segment_handle = base_id + lane_id + i * WORKGROUP_SIZE;
    if (segment_handle >= constants.segment_size)
      break;

    const uint cell_id = hashed_grid_elements[segment_handle].cell_id;
    InterlockedMin(hashed_grid_cell_starts[cell_id].start_index, segment_handle);
    uint old_end;
    InterlockedMax(hashed_grid_cell_starts[cell_id].end_index, segment_handle, old_end);
    if (old_end == 0xFFFFFFFFu)
      InterlockedExchange(hashed_grid_cell_starts[cell_id].end_index, segment_handle);
  }
}
