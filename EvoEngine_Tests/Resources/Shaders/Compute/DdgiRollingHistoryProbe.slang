import EvoEngine.DDGIProbeHistory;
import EvoEngine.DDGI;
import EvoEngine.DDGIGather;

[[vk::binding(31, 0)]] RWStructuredBuffer<uint4> results;

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
  float2 visibility = float2(0);
  uint2 filled_visibility = uint2(0);
  for (uint update = 0u; update < count * 100u; ++update) {
    uint phase = update % count;
    float value = float(phase + 1u);
    visibility = EE_DDGI_ROLL_VISIBILITY(texel, count, phase, float2(value, value * value), 64.0f, update == 0u);
    if (update == 0u && abs(visibility.x - 1.0f / float(count)) > 0.001f) ++errors;
    if (update == count - 1u) filled_visibility = EE_DDGI_VISIBILITY_SUM[texel];
    if (update >= count && any(filled_visibility != EE_DDGI_VISIBILITY_SUM[texel])) ++errors;
  }
  if (any(abs(EE_DDGI_ROLL_VISIBILITY(texel, count, 0u, float2(-1.0f), 64.0f, false) - visibility) > 0.0001f)) ++errors;
  if (any(filled_visibility != EE_DDGI_VISIBILITY_SUM[texel])) ++errors;
  EE_DDGI_ROLL_VISIBILITY(texel, count, 0u, float2(asfloat(0x7fc00000u)), 64.0f, false);
  if (any(filled_visibility != EE_DDGI_VISIBILITY_SUM[texel])) ++errors;
  EE_DDGI_CLEAR_VISIBILITY_HISTORY(texel, count);
  if (any(EE_DDGI_VISIBILITY_SUM[texel] != uint2(0))) ++errors;
  float2 bounded = EE_DDGI_ROLL_VISIBILITY(texel, count, 0u, float2(1000000.0f), 64.0f, true);
  if (any(abs(bounded - float2(32.0f, 2048.0f) / float(count)) > 0.001f)) ++errors;
  float2 large = EE_DDGI_ROLL_VISIBILITY(texel, count, 0u, float2(1000000.0f), 2048.0f, true);
  if (!all(isfinite(min(large, float2(65504.0f))))) ++errors;
  results[texel] = uint4(errors, filled_irradiance.x, asuint(visibility.x), asuint(startup));
  EE_DDGI_ROLL_IRRADIANCE(texel, count, 0u, float4(0.0f), true);
  if (any(EE_DDGI_IRRADIANCE_SUM[texel] != uint4(0u)))
    results[texel].x += 1u;
}
