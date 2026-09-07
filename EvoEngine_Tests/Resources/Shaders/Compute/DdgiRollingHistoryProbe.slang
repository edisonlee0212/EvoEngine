import EvoEngine.DDGIProbeHistory;
import EvoEngine.DDGI;
import EvoEngine.DDGIGather;

[[vk::binding(31, 0)]] RWStructuredBuffer<uint4> results;

struct VisibilityFixture : IDdgiGatherResources {
  float4 loadProbeState(uint volume_index, uint probe_index) { return float4(0.0f); }
  float4 sampleIrradiance(uint volume_index, float2 uv) { return float4(0.0f); }
  float4 loadVisibility(uint volume_index, int2 pixel) {
    int index = pixel.y * 2 + pixel.x;
    if (volume_index == 1u || index == 0) return float4(-1.0f);
    return float4(float2(index * 2.0f, index * 4.0f), 0.0f, 1.0f);
  }
};

[shader("compute")]
[numthreads(1, 1, 1)]
void main(uint3 thread : SV_DispatchThreadID) {
  uint texel = thread.x;
  uint count = (texel + 1u) * 5u;
  uint errors = 0u;
  DdgiVolumeInfo cascade = (DdgiVolumeInfo)0;
  cascade.probe_counts = float4(33.0f, 17.0f, 33.0f, 5.0f);
  cascade.probe_step_x = float4(0.8f, 0.0f, 0.0f, 0.0f);
  cascade.probe_step_y = float4(0.0f, 0.8f / 1.5f, 0.0f, 0.0f);
  cascade.probe_step_z = float4(0.0f, 0.0f, 0.8f, 0.0f);
  for (uint distance = 0u; distance <= 3u; ++distance) {
    float3 position = float3(float(distance) * 0.8f, 4.0f, 4.0f);
    if (abs(EE_DDGI_VOLUME_BOUNDARY_WEIGHT(cascade, position) - saturate(1.0f - float(distance) * 0.5f)) > 0.00001f)
      ++errors;
    if (EE_DDGI_VOLUME_COVERAGE(cascade, position) != 1.0f) ++errors;
  }
  if (EE_DDGI_VOLUME_COVERAGE(cascade, float3(-0.01f, 4.0f, 4.0f)) != 0.0f) ++errors;
  for (int offset = -7; offset <= 7; ++offset) {
    int coordinate = int(texel % 3u) + offset;
    while (coordinate < 0) coordinate += 3;
    while (coordinate >= 3) coordinate -= 3;
    if (EE_DDGI_SCROLL_PROBE_GRID(uint3(texel % 3u, 0u, 0u), int3(offset, 0, 0), uint3(3u, 1u, 1u)).x != uint(coordinate))
      ++errors;
  }
  uint4 filled_irradiance = uint4(0u);
  float startup = 0.0f;
  for (uint update = 0u; update < count * 100u; ++update) {
    uint phase = update % count;
    float value = float(phase + 1u);
    float4 irradiance = EE_DDGI_ROLL_IRRADIANCE(texel, count, phase, float4(value, value * 0.5f, 64.0f, 1.0f), update == 0u);
    if (update == 0u)
      startup = irradiance.x;
    if (update == count - 1u) {
      filled_irradiance = EE_DDGI_IRRADIANCE_SUM[texel];
    }
    if (update >= count && any(filled_irradiance != EE_DDGI_IRRADIANCE_SUM[texel]))
      ++errors;
  }
  EE_DDGI_ROLL_IRRADIANCE(texel, count, 0u, float4(asfloat(0x7fc00000u), 1.0f, 1.0f, 1.0f), false);
  if (any(filled_irradiance != EE_DDGI_IRRADIANCE_SUM[texel]))
    ++errors;
  float2 visibility = EE_DDGI_FILTER_VISIBILITY(float2(-1.0f), float2(4.0f, 8.0f), 0.9f, false);
  if (any(visibility != float2(4.0f, 8.0f))) ++errors;
  visibility = EE_DDGI_FILTER_VISIBILITY(visibility, float2(14.0f, 28.0f), 0.9f, false);
  if (any(abs(visibility - float2(5.0f, 10.0f)) > 0.0001f)) ++errors;
  if (any(EE_DDGI_FILTER_VISIBILITY(visibility, float2(-1.0f), 0.9f, false) != visibility)) ++errors;
  if (any(EE_DDGI_FILTER_VISIBILITY(visibility, float2(-1.0f), 0.9f, true) >= 0.0f)) ++errors;
  if (any(EE_DDGI_FILTER_VISIBILITY(visibility, float2(2.0f, 4.0f), 0.9f, true) != float2(2.0f, 4.0f))) ++errors;
  float2 interpolated = EE_DDGI_SAMPLE_VALID_VISIBILITY(VisibilityFixture(), 0u, float2(0.5f), float2(2.0f));
  if (any(abs(interpolated - float2(4.0f, 8.0f)) > 0.0001f)) ++errors;
  if (any(EE_DDGI_SAMPLE_VALID_VISIBILITY(VisibilityFixture(), 1u, float2(0.5f), float2(2.0f)) >= 0.0f)) ++errors;
  results[texel] = uint4(errors, filled_irradiance.x, asuint(visibility.x), asuint(startup));
  EE_DDGI_ROLL_IRRADIANCE(texel, count, 0u, float4(0.0f), true);
  if (any(EE_DDGI_IRRADIANCE_SUM[texel] != uint4(0u)))
    results[texel].x += 1u;
}
