// Godot hddagi_integrate.glsl::MODE_PROCESS, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt. Uses native cubemap sky and conservative geometry cache invalidation.
import EvoEngine.HddagiTrace;
import EvoEngine.HddagiCache;
import EvoEngine.HddagiVoxel;
import EvoEngine.HddagiProbe;
struct Params {
  int3 grid;
  uint cascade_count;
  float ray_bias;
  uint cascade;
  uint inactive_frames;
  uint history_size;
  int3 world_offset;
  uint sky_mode;
  float3 sky_color;
  float sky_energy;
  int3 probe_size;
  float y_mult;
  float sky_rotation;
  float sky_inverse_gamma;
  uint global_frame;
  uint force_update;
};
[[vk::push_constant]] ConstantBuffer<Params> params;
[[vk::binding(0, 0)]] Texture3D<uint2> voxels;
[[vk::binding(1, 0)]][vk::image_format("r8ui")] Texture3D<uint> regions;
[[vk::binding(2, 0)]] ConstantBuffer<HddagiCascadeBlock> cascades;
[[vk::binding(3, 0)]] Texture3D<float3> light_field;
[[vk::binding(4, 0)]][vk::image_format("r16ui")] Texture3D<uint> versions;
[[vk::binding(5, 0)]][vk::image_format("r32ui")] RWTexture2DArray<uint> hit_cache;
[[vk::binding(6, 0)]][vk::image_format("r16ui")] RWTexture2DArray<uint> hit_versions;
[[vk::binding(7, 0)]][vk::image_format("r32ui")] RWTexture2DArray<uint> history;
[[vk::binding(8, 0)]][vk::image_format("r32ui")] RWTexture2DArray<uint> history_sum;
[[vk::binding(9, 0)]][vk::image_format("r32ui")] RWTexture2DArray<uint> specular;
[[vk::binding(10, 0)]][vk::image_format("r32ui")] RWTexture2DArray<uint> diffuse;
[[vk::binding(11, 0)]][vk::image_format("rgba16f")] RWTexture2DArray<float4> ambient;
[[vk::binding(12, 0)]][vk::image_format("r32ui")] RWTexture2DArray<uint> process_frame;
[[vk::binding(13, 0)]] Texture2DArray<float> proximity;
[[vk::binding(14, 0)]] TextureCube<float4> sky;
[[vk::binding(15, 0)]] SamplerState sky_sampler;
[[vk::binding(16, 0)]] RWStructuredBuffer<uint> status;
groupshared uint3 accumulated[25];
groupshared float3 neighbors[25];
groupshared uint3 ambient_accum;
groupshared int history_index;
groupshared uint field_failure;

[numthreads(5, 5, 1)] void main(uint3 group : SV_GroupID, uint3 local : SV_GroupThreadID) {
  if (all(local == 0))
    field_failure = status[1];
  GroupMemoryBarrierWithGroupSync();
  if (field_failure != 0)
    return;
  uint index = local.x + local.y * 5;
  int3 probe = int3(group.x, group.y % params.probe_size.y, group.y / params.probe_size.y);
  int3 world_probe = probe + params.world_offset;
  int3 physical = HddagiWrap(world_probe, params.probe_size);
  int3 tile = int3(physical.xy + int2(0, physical.z * params.probe_size.y), params.cascade);
  accumulated[index] = uint3(0);
  if (index == 0) {
    uint frame = status[3] != 0 ? process_frame[tile] : 0;
    uint phase = frame & 0x0fffffffu;
    uint parity = (uint(world_probe.x) & 1) | ((uint(world_probe.y) & 1) << 1) | ((uint(world_probe.z) & 1) << 2);
    bool process = params.force_update != 0 || phase < 4 || proximity.Load(int4(tile, 0)) > 0.5 ||
                   (params.global_frame + parity) % params.inactive_frames == 0;
    history_index = process ? int(phase % params.history_size) : -1;
    if (process)
      process_frame[tile] = (phase + 1) & 0x0fffffffu;
    ambient_accum = uint3(0);
  }
  GroupMemoryBarrierWithGroupSync();
  if (history_index < 0)
    return;
  uint3 hashed = HddagiHash((uint3(world_probe) * 25 + index) * params.history_size + uint(history_index));
  uint hash = hashed.x ^ hashed.y ^ hashed.z;
  float2 offset = float2(hash >> 16, hash & 65535) / 65535.0;
  float3 world_direction = HddagiOctDecode((float2(local.xy) + offset) / 5);
  float3 direction = normalize(world_direction * float3(1, params.y_mult, 1));
  float3 origin = cascades.data[params.cascade].offset + float3(probe * 8) / cascades.data[params.cascade].to_cell;
  origin += direction / max(abs(direction.x), max(abs(direction.y), abs(direction.z))) * params.ray_bias /
            cascades.data[params.cascade].to_cell;
  int3 cache_pixel = int3(tile.xy * 5 + int2(local.xy), tile.z * params.history_size + history_index);
  int3 hit_cell;
  uint hit_cascade, iterations;
  bool reused;
  bool hit = HddagiTraceCached(voxels, regions, versions, hit_cache, hit_versions, cache_pixel, cascades, params.grid,
                               params.cascade_count, origin, direction, params.cascade, true, hit_cell, hit_cascade,
                               iterations, reused);
  float3 light = float3(0);
  if (hit) {
    hit_cell.y += hit_cascade * params.grid.y;
    light = light_field.Load(int4(hit_cell, 0));
  } else if (params.sky_mode == 1) {
    light = params.sky_color * params.sky_energy;
  } else if (params.sky_mode == 2) {
    float sine, cosine;
    sincos(params.sky_rotation, sine, cosine);
    float3 rotated = float3(cosine * world_direction.x - sine * world_direction.z, world_direction.y,
                            sine * world_direction.x + cosine * world_direction.z);
    light = pow(max(sky.SampleLevel(sky_sampler, rotated, 2).rgb, float3(0)), float3(params.sky_inverse_gamma)) *
            params.sky_energy;
  }
  if (!all(isfinite(light))) {
    InterlockedOr(status[1], 2u);
    light = float3(0);
  }
  offset = offset * 2 - 1;
  int2 base = int2(1) + int2(local.xy) - select(offset < 0, int2(1), int2(0));
  float2 blend = select(offset < 0, 1 + offset, offset);
  for (int y = 0; y < 2; ++y)
    for (int x = 0; x < 2; ++x) {
      uint wrapped = HDDAGI_WRAP_NEIGHBORS[(base.y + y) * 7 + base.x + x];
      uint target = (wrapped >> 16) * 5 + (wrapped & 65535);
      float weight = (y == 0 ? 1 - blend.y : blend.y) * (x == 0 ? 1 - blend.x : blend.x);
      uint3 value = uint3(clamp(light * weight * 16384, float3(0), float3(4194303)));
      InterlockedAdd(accumulated[target].x, value.x);
      InterlockedAdd(accumulated[target].y, value.y);
      InterlockedAdd(accumulated[target].z, value.z);
    }
  GroupMemoryBarrierWithGroupSync();
  uint packed = HddagiEncodeRadiance(float3(accumulated[index]) / 16384.0);
  int3 sum_pixel = int3(cache_pixel.xy * int2(3, 1), params.cascade);
  uint3 sum =
      uint3(history_sum[sum_pixel], history_sum[sum_pixel + int3(1, 0, 0)], history_sum[sum_pixel + int3(2, 0, 0)]);
  if (status[3] == 0) {
    sum = uint3(0);
    for (uint phase = 0; phase < params.history_size; ++phase)
      history[int3(cache_pixel.xy, params.cascade * params.history_size + phase)] = 0;
  }
  uint3 previous = status[3] == 0 ? uint3(0) : HddagiDecodeFixed(history[cache_pixel]);
  sum = sum - previous + HddagiDecodeFixed(packed);
  history[cache_pixel] = packed;
  history_sum[sum_pixel] = sum.x;
  history_sum[sum_pixel + int3(1, 0, 0)] = sum.y;
  history_sum[sum_pixel + int3(2, 0, 0)] = sum.z;
  sum /= params.history_size;
  InterlockedAdd(ambient_accum.x, sum.x);
  InterlockedAdd(ambient_accum.y, sum.y);
  InterlockedAdd(ambient_accum.z, sum.z);
  light = float3(sum) / 16384.0;
  neighbors[index] = light;
  GroupMemoryBarrierWithGroupSync();
  float3 diffuse_light = float3(0);
  for (uint i = 0; i < 15; ++i) {
    uint weighted = HDDAGI_NEIGHBOR_WEIGHTS[index * 15 + i];
    diffuse_light += neighbors[weighted >> 16] * (float(weighted & 65535) / 65535.0);
  }
  uint specular_packed = HddagiEncodeRadiance(light);
  uint diffuse_packed = HddagiEncodeRadiance(diffuse_light);
  for (uint pixel = 0; pixel < 49; ++pixel) {
    uint wrapped = HDDAGI_WRAP_NEIGHBORS[pixel];
    if ((wrapped & 65535) != local.x || (wrapped >> 16) != local.y)
      continue;
    int3 target = int3(tile.xy * 7 + int2(pixel % 7, pixel / 7), tile.z);
    specular[target] = specular_packed;
    diffuse[target] = diffuse_packed;
  }
  if (index == 0)
    ambient[tile] = float4(float3(ambient_accum) / (16384.0 * 25), 1);
}
