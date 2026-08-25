import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.SortCompare;

static const uint WORKGROUP_SIZE = COMPUTE_WORK_GROUP_INVOCATIONS / 2;

struct PushConstants {
  uint segment_size;
  uint group_size;
};

[[vk::push_constant]] ConstantBuffer<PushConstants> constants;

void global_compare_and_swap(uint2 indices) {
  if (is_larger(hashed_grid_elements[indices.x], hashed_grid_elements[indices.y])) {
    HashedGridElement temporary = hashed_grid_elements[indices.x];
    hashed_grid_elements[indices.x] = hashed_grid_elements[indices.y];
    hashed_grid_elements[indices.y] = temporary;
  }
}

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  const uint half_height = constants.group_size >> 1;
  const uint block = ((2 * dispatch_thread_id.x) / constants.group_size) * constants.group_size;
  const uint x = block + dispatch_thread_id.x % half_height;
  const uint y = block + constants.group_size - dispatch_thread_id.x % half_height - 1;
  if (x < constants.segment_size && y < constants.segment_size)
    global_compare_and_swap(uint2(x, y));
}
