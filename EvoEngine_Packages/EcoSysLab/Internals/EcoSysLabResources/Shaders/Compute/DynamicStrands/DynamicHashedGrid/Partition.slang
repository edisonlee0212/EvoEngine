import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint ITERATIONS_PER_TASK =
    (COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE;

struct PushConstants {
  uint segment_size;
  float cell_size;
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

    Segment segment = segments[segment_handle];
    const float3 center = (segment.particle0.x + segment.particle1.x) * 0.5f;
    const int3 coordinate = int3(center / constants.cell_size);

    HashedGridElement element;
    element.cell_id = hash_cell_coordinate(coordinate);
    element.segment_handle = segment_handle;
    element.padding0 = 0;
    element.padding1 = 0;
    hashed_grid_elements[segment_handle] = element;
  }
}
