import EvoEngine.DDGIProbeHistory;
import EvoEngine.DDGI;

[[vk::binding(0, 0)]] RWStructuredBuffer<uint4> results;

[shader("compute")]
[numthreads(1, 1, 1)]
void main(uint3 thread : SV_DispatchThreadID) {
  uint texel = thread.x;
  uint count = (texel + 1u) * 5u;
  uint errors = 0u;
  for (int offset = -7; offset <= 7; ++offset) {
    int coordinate = int(texel % 3u) + offset;
    while (coordinate < 0) coordinate += 3;
    while (coordinate >= 3) coordinate -= 3;
    if (EE_DDGI_SCROLL_PROBE_GRID(uint3(texel % 3u, 0u, 0u), int3(offset, 0, 0), uint3(3u, 1u, 1u)).x != uint(coordinate))
      ++errors;
  }
  uint4 filled_irradiance = uint4(0u);
  uint2 filled_visibility = uint2(0u);
  float startup = 0.0f;
  for (uint update = 0u; update < count * 100u; ++update) {
    uint phase = update % count;
    float value = float(phase + 1u);
    float4 irradiance = EE_DDGI_ROLL_IRRADIANCE(texel, count, phase, float4(value, value * 0.5f, 64.0f, 1.0f), update == 0u);
    EE_DDGI_ROLL_VISIBILITY(texel, count, phase, float2(value, value * value), float2(32.0f, 1024.0f), update == 0u);
    if (update == 0u)
      startup = irradiance.x;
    if (update == count - 1u) {
      filled_irradiance = EE_DDGI_IRRADIANCE_SUM[texel];
      filled_visibility = EE_DDGI_VISIBILITY_SUM[texel];
    }
    if (update >= count && (any(filled_irradiance != EE_DDGI_IRRADIANCE_SUM[texel]) ||
                             any(filled_visibility != EE_DDGI_VISIBILITY_SUM[texel])))
      ++errors;
  }
  EE_DDGI_ROLL_IRRADIANCE(texel, count, 0u, float4(asfloat(0x7fc00000u), 1.0f, 1.0f, 1.0f), false);
  EE_DDGI_ROLL_VISIBILITY(texel, count, 0u, float2(-1.0f), float2(32.0f, 1024.0f), false);
  if (any(filled_irradiance != EE_DDGI_IRRADIANCE_SUM[texel]) || any(filled_visibility != EE_DDGI_VISIBILITY_SUM[texel]))
    ++errors;
  results[texel] = uint4(errors, filled_irradiance.x, filled_visibility.y, asuint(startup));
  EE_DDGI_ROLL_IRRADIANCE(texel, count, 0u, float4(0.0f), true);
  EE_DDGI_CLEAR_VISIBILITY_HISTORY(texel, count);
  if (any(EE_DDGI_IRRADIANCE_SUM[texel] != uint4(0u)) || any(EE_DDGI_VISIBILITY_SUM[texel] != uint2(0u)))
    results[texel].x += 1u;
}
