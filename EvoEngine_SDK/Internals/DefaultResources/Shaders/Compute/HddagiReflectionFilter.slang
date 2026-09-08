// Godot hddagi_filter.glsl bilateral filter, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt. Bounds and untouched pixels are explicitly initialized.
import EvoEngine.LightingFixedSet3;
import EvoEngine.HddagiCamera;
import EvoEngine.HddagiVoxel;

[[vk::binding(17, 1)]] Sampler2D inDepth;

float4 surface_at(uint2 pixel) {
  float4 surface =
      EE_HDDAGI_SURFACE.Load(int3(min(pixel * EE_HDDAGI_CAMERA.pixel_stride, EE_HDDAGI_CAMERA.viewport - 1), 0));
  if (surface.w >= 2)
    surface.w -= 2;
  return surface;
}
float depth_at(uint2 pixel) {
  uint2 source = min(pixel * EE_HDDAGI_CAMERA.pixel_stride, EE_HDDAGI_CAMERA.viewport - 1);
  float2 uv = (float2(source) + 0.5) / float2(EE_HDDAGI_CAMERA.viewport);
  float3 world = EE_DEPTH_TO_WORLD_POS(EE_BASIC_CONSTANTS.camera_index, uv, inDepth.SampleLevel(uv, 0).x);
  return mul(EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].view, float4(world, 1)).z;
}
float4 specular_at(uint2 pixel) {
#ifdef MODE_VERTICAL
  return float4(EE_HDDAGI_SPECULAR_SCRATCH.Load(int3(pixel, 0)), EE_HDDAGI_BLEND_SCRATCH.Load(int3(pixel, 0)).y);
#else
  return float4(EE_HDDAGI_CAMERA_SPECULAR.Load(int3(pixel, 0)), EE_HDDAGI_CAMERA_BLEND.Load(int3(pixel, 0)).y);
#endif
}

[numthreads(16, 16, 1)] void main(uint3 id : SV_DispatchThreadID) {
  uint2 pixel = id.xy;
  if (any(pixel >= EE_HDDAGI_CAMERA.gi_size))
    return;
  float4 surface = surface_at(pixel);
  float4 result = specular_at(pixel);
  // The pinned gather threshold is 0.25; this separate filter uses 0.3.
  if (dot(surface.xyz, surface.xyz) > 0.5 && surface.w < 0.3) {
    float depth = depth_at(pixel);
    float4 accumulated = float4(0);
    float total = 0;
    for (int i = -int(EE_HDDAGI_CAMERA.filter_radius); i <= int(EE_HDDAGI_CAMERA.filter_radius); ++i) {
#ifdef MODE_VERTICAL
      int2 offset = int2(0, i);
#else
      int2 offset = int2(i, 0);
#endif
      uint2 read = uint2(clamp(int2(pixel) + offset, int2(0), int2(EE_HDDAGI_CAMERA.gi_size) - 1));
      float4 neighbor = surface_at(read);
      float weight = exp(-abs(depth - depth_at(read))) * pow(max(0, dot(neighbor.xyz, surface.xyz)), 4) *
                     max(0, 1 - abs(neighbor.w - surface.w) * 4);
      if (weight > 0) {
        accumulated += specular_at(read) * weight;
        total += weight;
      }
    }
    if (total > 0)
      result = accumulated / total;
  }
#ifdef MODE_VERTICAL
  float diffuse_blend = EE_HDDAGI_BLEND_SCRATCH.Load(int3(pixel, 0)).x;
  EE_HDDAGI_SPECULAR_OUT[pixel] = HddagiEncodeRadiance(result.rgb);
  EE_HDDAGI_BLEND_OUT[pixel] = saturate(float2(diffuse_blend, result.a));
#else
  float diffuse_blend = EE_HDDAGI_CAMERA_BLEND.Load(int3(pixel, 0)).x;
  EE_HDDAGI_SPECULAR_SCRATCH_OUT[pixel] = HddagiEncodeRadiance(result.rgb);
  EE_HDDAGI_BLEND_SCRATCH_OUT[pixel] = saturate(float2(diffuse_blend, result.a));
#endif
}
