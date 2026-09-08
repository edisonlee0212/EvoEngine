// Godot hddagi_direct_light.glsl, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt. Native photometry, immutable emission and rectangular field adapters.
import EvoEngine.HddagiTrace;
import EvoEngine.HddagiVoxel;
import EvoEngine.SdfgiTypes;

struct Params {
  int3 grid;
  uint cascade_count;
  uint cascade;
  uint light_count;
  uint process_offset;
  uint process_increment;
  float bounce_feedback;
  float y_mult;
  uint refresh_static;
  uint capacity;
  int3 probe_size;
  uint padding;
};
[[vk::push_constant]] ConstantBuffer<Params> params;
[[vk::binding(0, 0)]] Texture3D<uint2> voxels;
[[vk::binding(1, 0)]][vk::image_format("r8ui")] Texture3D<uint> regions;
[[vk::binding(2, 0)]] ConstantBuffer<HddagiCascadeBlock> cascades;
[[vk::binding(3, 0)]] StructuredBuffer<uint> dispatch_data;
[[vk::binding(4, 0)]] RWStructuredBuffer<HddagiProcessVoxel> process_voxels;
[[vk::binding(5, 0)]] StructuredBuffer<SdfgiLight> lights;
[[vk::binding(6, 0)]][vk::image_format("r32ui")] RWTexture3D<uint> light_field;
[[vk::binding(7, 0)]][vk::image_format("r32ui")] RWTexture3D<uint> static_field;
[[vk::binding(8, 0)]] Texture2DArray<float3> previous_diffuse;
[[vk::binding(9, 0)]] SamplerState linear_sampler;
[[vk::binding(10, 0)]] RWStructuredBuffer<uint> status;

float3 decode_normal(uint packed) {
  float2 f = float2((packed >> 16) & 255, packed >> 24) / 255.0 * 2 - 1;
  float3 normal = float3(f, 1 - abs(f.x) - abs(f.y));
  float t = saturate(-normal.z);
  normal.xy += select(normal.xy >= 0, float2(-t), float2(t));
  return normalize(normal);
}

[numthreads(64, 1, 1)] void main(uint3 id : SV_DispatchThreadID) {
  if (status[1] != 0 || dispatch_data[4] != 0 || id.x >= min(dispatch_data[3], params.capacity))
    return;
  HddagiProcessVoxel voxel = process_voxels[id.x];
#if defined(MODE_PROCESS_STATIC)
  if (params.refresh_static == 0 && (voxel.position & 0x80000000u) == 0)
    return;
#else
  if ((voxel.position & 0x40000000u) == 0 && (id.x + params.process_offset) % params.process_increment != 0)
    return;
#endif
  int3 cell = int3(uint3(voxel.position, voxel.position >> 8, voxel.position >> 16) & 255);
  int3 physical = HddagiWrap(cell + cascades.data[params.cascade].region_world_offset * 8, params.grid);
  physical.y += params.grid.y * params.cascade;
  float3 position = (float3(cell) + 0.5) / cascades.data[params.cascade].to_cell + cascades.data[params.cascade].offset;
  float3 albedo =
      float3(uint3(voxel.albedo_normal, voxel.albedo_normal >> 5, voxel.albedo_normal >> 11) & uint3(31, 63, 31)) /
      float3(31, 63, 31);
  float3 normal = decode_normal(voxel.albedo_normal);
  float3 accumulated = float3(0);
#if !defined(MODE_PROCESS_STATIC)
  accumulated = HddagiDecodeRadiance(static_field[physical]);
  if (status[3] != 0 && params.bounce_feedback > 0.001) {
    float2 oct = float2((voxel.albedo_normal >> 16) & 255, voxel.albedo_normal >> 24) / 255.0;
    float2 inv_size = 1.0 / float2(7 * params.probe_size.x, 7 * params.probe_size.y * params.probe_size.z);
    float3 indirect = float3(0);
    for (uint i = 0; i < 8; ++i) {
      float weight = float((voxel.occlusion >> (i * 4)) & 15) / 15;
      if (weight == 0)
        continue;
      int3 probe = cell / 8 + int3(uint3(i, i >> 1, i >> 2) & 1);
      probe = HddagiWrap(probe + cascades.data[params.cascade].region_world_offset, params.probe_size);
      int2 tile = probe.xy + int2(0, probe.z * params.probe_size.y);
      float2 uv = (float2(tile * 7 + 1) + oct * 5) * inv_size;
      indirect += previous_diffuse.SampleLevel(linear_sampler, float3(uv, params.cascade), 0) * weight;
    }
    accumulated += indirect * albedo * params.bounce_feedback;
  }
#endif
  for (uint i = 0; i < params.light_count; ++i) {
    SdfgiLight light = lights[i];
    float3 world_direction = -light.direction;
    if (light.type == 0) {
      world_direction.y /= params.y_mult;
      world_direction = normalize(world_direction);
    }
    float distance = 1e20;
    float attenuation = 1;
    if (light.type != 0) {
      float3 delta = light.position - position;
      distance = length(delta);
      delta.y /= params.y_mult;
      float world_distance = length(delta);
      if (world_distance <= 0 || world_distance > light.radius)
        continue;
      world_direction = delta / world_distance;
      attenuation = 1.0 / dot(light.host_photometry.xyz, float3(1, world_distance, world_distance * world_distance));
      if (light.type == 2)
        attenuation *= saturate((dot(-world_direction, light.direction) - light.cos_spot_angle) /
                                (light.host_photometry.w - light.cos_spot_angle));
    }
    attenuation *= max(0, dot(normal, world_direction));
    if (attenuation < 0.001)
      continue;
    float3 direction = normalize(world_direction * float3(1, params.y_mult, 1));
    uint iterations;
    if (light.has_shadow != 0 && HddagiVisibility(voxels, regions, cascades, params.grid, params.cascade_count,
                                                  position, direction, distance, params.cascade, iterations))
      continue;
    accumulated += albedo * light.color * light.energy * attenuation;
  }
  if (!all(isfinite(accumulated))) {
    InterlockedOr(status[1], 2u);
    accumulated = float3(0);
  }
#if defined(MODE_PROCESS_STATIC)
  static_field[physical] = HddagiEncodeRadiance(accumulated + HddagiDecodeRadiance(voxel.emission));
  process_voxels[id.x].position &= ~0x80000000u;
#else
  light_field[physical] = HddagiEncodeRadiance(accumulated);
  process_voxels[id.x].position &= ~0x40000000u;
#endif
}
