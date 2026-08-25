import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.SortCompare;

static const uint WORKGROUP_SIZE = COMPUTE_WORK_GROUP_INVOCATIONS / 2;

struct PushConstants {
  uint segment_size;
  uint group_size;
};

[[vk::push_constant]] ConstantBuffer<PushConstants> constants;
groupshared HashedGridElement local_value[WORKGROUP_SIZE * 2];

void local_compare_and_swap(int2 indices) {
  if (is_larger(local_value[indices.x], local_value[indices.y])) {
    HashedGridElement temporary = local_value[indices.x];
    local_value[indices.x] = local_value[indices.y];
    local_value[indices.y] = temporary;
  }
}

void local_disperse(uint height, uint lane_id) {
  for (; height > 1; height /= 2) {
    GroupMemoryBarrierWithGroupSync();
    const uint half_height = height >> 1;
    const int2 indices = int2(height * ((2 * lane_id) / height)) +
                         int2(lane_id % half_height, half_height + (lane_id % half_height));
    local_compare_and_swap(indices);
  }
}

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint offset = WORKGROUP_SIZE * 2 * group_id.x;
  const uint segment_handle0 = offset + lane_id * 2;
  const uint segment_handle1 = segment_handle0 + 1;

  if (segment_handle0 < constants.segment_size)
    local_value[lane_id * 2] = hashed_grid_elements[segment_handle0];
  else
    local_value[lane_id * 2].cell_id = 0xFFFFFFFFu;
  if (segment_handle1 < constants.segment_size)
    local_value[lane_id * 2 + 1] = hashed_grid_elements[segment_handle1];
  else
    local_value[lane_id * 2 + 1].cell_id = 0xFFFFFFFFu;

  local_disperse(constants.group_size, lane_id);

  GroupMemoryBarrierWithGroupSync();
  if (segment_handle0 < constants.segment_size)
    hashed_grid_elements[segment_handle0] = local_value[lane_id * 2];
  if (segment_handle1 < constants.segment_size)
    hashed_grid_elements[segment_handle1] = local_value[lane_id * 2 + 1];
}
