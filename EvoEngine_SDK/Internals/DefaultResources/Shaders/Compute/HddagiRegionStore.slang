// Godot hddagi_preprocess.glsl::MODE_REGION_STORE, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt.
import EvoEngine.HddagiTrace;

struct HddagiRegionParams {
  int3 grid;
  uint cascade;
  int3 offset;
  uint version;
  int3 region_world_offset;
  uint padding;
};
[[vk::push_constant]] ConstantBuffer<HddagiRegionParams> params;
[[vk::binding(0, 0)]] Texture3D<uint> normals;
[[vk::binding(1, 0)]] [vk::image_format("rg32ui")] RWTexture3D<uint2> voxels;
[[vk::binding(2, 0)]] [vk::image_format("r8ui")] RWTexture3D<uint> regions;
[[vk::binding(3, 0)]] [vk::image_format("r16ui")] RWTexture3D<uint> versions;
groupshared uint solid_count;
groupshared uint bits[16];

[numthreads(8, 8, 8)]
void main(uint3 id : SV_DispatchThreadID, uint3 local : SV_GroupThreadID) {
  uint3 block = local / 4;
  uint block_index = block.z * 4 + block.y * 2 + block.x;
  uint3 voxel = local % 4;
  uint bit = voxel.z * 16 + voxel.y * 4 + voxel.x;
  if (all(local == 0))
    solid_count = 0;
  if (all(voxel == 0)) {
    bits[block_index * 2] = 0;
    bits[block_index * 2 + 1] = 0;
  }
  GroupMemoryBarrierWithGroupSync();
  int3 source = int3(id) + params.offset;
  if (normals.Load(int4(source, 0)) != 0) {
    InterlockedAdd(solid_count, 1);
    InterlockedOr(bits[block_index * 2 + (bit >> 5)], 1u << (bit & 31));
  }
  GroupMemoryBarrierWithGroupSync();
  int3 region = HddagiWrap(source / 8 + params.region_world_offset, params.grid / 8);
  if (all(voxel == 0)) {
    int3 destination = (region * 8 + int3(local)) / 4;
    destination.y += int(params.cascade) * params.grid.y / 4;
    voxels[destination] = uint2(bits[block_index * 2], bits[block_index * 2 + 1]);
  }
  if (all(local == 0)) {
    region.y += int(params.cascade) * params.grid.y / 8;
    regions[region] = solid_count != 0 ? 1u : 0u;
    versions[region] = params.version;
  }
}
