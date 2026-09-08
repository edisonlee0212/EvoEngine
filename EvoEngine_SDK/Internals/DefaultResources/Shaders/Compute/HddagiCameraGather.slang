import EvoEngine.LightingFixedSet3;
import EvoEngine.HddagiCamera;
import EvoEngine.HddagiProbe;
import EvoEngine.HddagiVoxel;

[[vk::binding(17, 1)]] Sampler2D inDepth;
groupshared float3 positions[256];
groupshared float3 normals[256];
groupshared float4 reflections[256];

[numthreads(16, 16, 1)] void main(uint3 id : SV_DispatchThreadID, uint3 local : SV_GroupThreadID) {
  uint2 pixel = id.xy;
  uint index = local.y * 16 + local.x;
  bool active = all(pixel < EE_HDDAGI_CAMERA.gi_size);
  uint2 source = min(pixel * EE_HDDAGI_CAMERA.pixel_stride, EE_HDDAGI_CAMERA.viewport - 1);
  float2 uv = (float2(source) + 0.5) / float2(EE_HDDAGI_CAMERA.viewport);
  float4 surface = EE_HDDAGI_SURFACE.Load(int3(source, 0));
  float depth = inDepth.SampleLevel(uv, 0).x;
  float3 position = EE_DEPTH_TO_WORLD_POS(EE_BASIC_CONSTANTS.camera_index, uv, depth);
  float roughness = surface.w >= 2 ? surface.w - 2 : surface.w;
  float4 diffuse = float4(0), specular = float4(0);
  bool valid = active && dot(surface.xyz, surface.xyz) > 0.5 && isfinite(depth) && depth < 1;
  if (valid) {
    float3 view = normalize(EE_CAMERA_POSITION(EE_BASIC_CONSTANTS.camera_index) - position);
    EE_HDDAGI_CAMERA_GATHER(position, surface.xyz, reflect(-view, surface.xyz), roughness, surface.w >= 2, pixel,
                            diffuse, specular);
  }
  positions[index] = position;
  normals[index] = valid ? surface.xyz : float3(0);
  reflections[index] = specular;
  GroupMemoryBarrierWithGroupSync();
  if (!active)
    return;
  if (valid && roughness < 0.25 && EE_HDDAGI_CAMERA.reflection_capture == 0) {
    uint base = (local.y & ~1u) * 16 + (local.x & ~1u);
    uint4 group = uint4(base, base + 1, base + 16, base + 17);
    float3 p0 = positions[group.x], p1 = positions[group.y], p2 = positions[group.z], p3 = positions[group.w];
    float3 n1 = cross(p0 - p2, p0 - p1), n2 = cross(p2 - p3, p2 - p1);
    float weight = 0;
    if (dot(n1, n1) > 0 && dot(n2, n2) > 0) {
      weight = smoothstep(0.9659258262890683, 1, dot(normalize(n1), normalize(n2)));
      weight *= smoothstep(0.9659258262890683, 1,
                           length((normals[group.x] + normals[group.y] + normals[group.z] + normals[group.w]) * 0.25));
    }
    float4 average = (reflections[group.x] + reflections[group.y] + reflections[group.z] + reflections[group.w]) * 0.25;
    specular = lerp(specular, average, weight);
  }
  EE_HDDAGI_DIFFUSE_OUT[pixel] = HddagiEncodeRadiance(diffuse.rgb);
  EE_HDDAGI_SPECULAR_OUT[pixel] = HddagiEncodeRadiance(specular.rgb);
  EE_HDDAGI_BLEND_OUT[pixel] = saturate(float2(diffuse.a, specular.a));
}
