// Godot MODE_LIGHT_SCROLL, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt. Host edits exclude the union of rebuilt light-cell bounds.
import EvoEngine.HddagiVoxel;
struct HddagiScrollParams {
  int3 grid; uint capacity;
  int3 scroll; uint cascade;
  uint region_count; uint3 padding;
};
[[vk::push_constant]] ConstantBuffer<HddagiScrollParams> params;
[[vk::binding(0, 0)]] StructuredBuffer<uint> source_dispatch;
[[vk::binding(1, 0)]] StructuredBuffer<HddagiProcessVoxel> source;
[[vk::binding(2, 0)]] RWStructuredBuffer<uint> destination_dispatch;
[[vk::binding(3, 0)]] RWStructuredBuffer<HddagiProcessVoxel> destination;
[[vk::binding(4, 0)]] StructuredBuffer<HddagiUpdateBounds> update_bounds;
groupshared uint group_count;
groupshared uint group_offset;
[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID, uint3 local : SV_GroupThreadID) {
  if (local.x == 0) group_count = 0;
  if (id.x == 0) {
    destination_dispatch[1] = destination_dispatch[2] = 1;
    InterlockedOr(destination_dispatch[4], source_dispatch[4]);
  }
  bool keep = id.x < min(source_dispatch[3], params.capacity);
  HddagiProcessVoxel voxel = {0, 0, 0, 0};
  int3 cell = int3(0);
  uint local_index = 0;
  if (keep) {
    voxel = source[id.x];
    cell = int3(voxel.position & 255u, (voxel.position >> 8) & 255u, (voxel.position >> 16) & 255u) - params.scroll;
    keep = all(cell >= 0) && all(cell < params.grid);
    for (uint region = 0; region < params.region_count; ++region)
      if (HddagiContains(update_bounds[region], cell, params.cascade)) keep = false;
  }
  GroupMemoryBarrierWithGroupSync();
  if (keep) InterlockedAdd(group_count, 1, local_index);
  GroupMemoryBarrierWithGroupSync();
  if (keep && local_index == 0) {
    InterlockedAdd(destination_dispatch[3], group_count, group_offset);
    InterlockedMax(destination_dispatch[0], (min(group_offset + group_count, params.capacity) + 63) / 64);
    if (group_offset + group_count > params.capacity) InterlockedOr(destination_dispatch[4], 1u);
  }
  GroupMemoryBarrierWithGroupSync();
  if (keep && group_offset + local_index < params.capacity) {
    voxel.position = uint(cell.x | (cell.y << 8) | (cell.z << 16)) | (voxel.position & 0xff000000u);
    destination[group_offset + local_index] = voxel;
  }
}
