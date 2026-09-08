// Godot hddagi_preprocess.glsl probe neighbours/proximity, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt. Camera scheduling is disabled; supported probes are treated as visible.
import EvoEngine.HddagiTrace;
struct Params {
  int3 grid;
  uint cascade;
  int3 region_offset;
  uint cascade_count;
  int3 probe_size;
  uint padding;
};
[[vk::push_constant]] ConstantBuffer<Params> params;
[[vk::binding(0, 0)]][vk::image_format("r8ui")] Texture3D<uint> regions;
[[vk::binding(1, 0)]] Texture3D<float4> occlusion0;
[[vk::binding(2, 0)]] Texture3D<float4> occlusion1;
[[vk::binding(3, 0)]] SamplerState linear_sampler;
[[vk::binding(4, 0)]][vk::image_format("r32ui")] RWTexture2DArray<uint> neighbors;
[[vk::binding(5, 0)]][vk::image_format("r8")] RWTexture2DArray<float> proximity;
[[vk::binding(6, 0)]][vk::image_format("r8")] RWTexture2DArray<float> camera_visibility;
[numthreads(4, 4, 4)] void main(uint3 id : SV_DispatchThreadID) {
  if (any(id >= uint3(params.probe_size)))
    return;
  int3 probe = int3(id);
  int3 physical = HddagiWrap(probe + params.region_offset, params.probe_size);
  int3 destination = int3(physical.xy + int2(0, physical.z * params.probe_size.y), params.cascade);
  bool geometry = false;
  for (uint i = 0; i < 8; ++i) {
    int3 region = probe + int3(uint3(i, i >> 1, i >> 2) & 1) * 2 - 1;
    if (any(region < 0) || any(region >= params.probe_size - 1))
      continue;
    region = HddagiWrap(region + params.region_offset, params.grid / 8);
    region.y += params.cascade * params.grid.y / 8;
    geometry = geometry || regions.Load(int4(region, 0)) != 0;
  }
  uint visibility = 0;
  uint3 bits = uint3(probe + params.region_offset) & 1;
  uint layer = bits.x | (bits.y << 1) | (bits.z << 2);
  float3 extent = float3(params.grid + 2) * float3(1, params.cascade_count, 1);
  for (uint i = 0; i < 6; ++i) {
    int3 direction = int3(0);
    direction[i / 2] = (i & 1) != 0 ? 1 : -1;
    int3 next = probe + direction;
    if (any(next < 0) || any(next >= params.probe_size))
      continue;
    int3 sample = HddagiWrap(probe * 8 + direction * 7 + params.region_offset * 8, params.grid) + 1;
    sample.y += (params.grid.y + 2) * params.cascade;
    float3 uv = float3(sample) / extent;
    float4 weights =
        layer < 4 ? occlusion0.SampleLevel(linear_sampler, uv, 0) : occlusion1.SampleLevel(linear_sampler, uv, 0);
    visibility |= uint(clamp(weights[layer & 3] * 15, 0, 15)) << (i * 4);
  }
  neighbors[destination] = visibility;
  proximity[destination] = geometry ? 1 : 0;
  camera_visibility[destination] = 1;
}
