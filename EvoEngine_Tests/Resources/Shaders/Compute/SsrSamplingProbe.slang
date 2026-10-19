import EvoEngine.SsrSampling;
import EvoEngine.Random;
[[vk::binding(0, 0)]] RWStructuredBuffer<float4> values;
[numthreads(64, 1, 1)] void main(uint3 thread : SV_DispatchThreadID) {
  uint index = thread.x;
  if (index >= 10u * 65536u) return;
  uint test = index / 65536u;
  if (test == 9u) {
    uint2 pixel = uint2((index % 65536u) / 256u, 0);
    uint frame = index % 256u;
    uint seed = EE_XXHASH32(uint3(pixel, frame));
    values[index] = float4(EE_SSR_RANDOM(pixel, frame, 0), EE_PCG_RANDOM_2(seed));
    return;
  }
  float roughness = test == 0u ? 0.0f : (test == 1u ? 0.4f : (test == 2u || test == 4u || test == 6u ? 0.65f : 1.0f));
  float nv = test == 4u || test == 5u ? 0.2f : 1.0f;
  float3 view = float3(sqrt(1.0f - nv * nv), 0, nv);
  float2 xi = EE_SSR_RANDOM(uint2(index % 65536u, 0), test == 6u ? 1u : 0u, 0u);
  float3 white_weight, dielectric_weight;
  float3 direction = EE_SSR_SAMPLE_GGX(view, float3(0, 0, 1), roughness, xi, float3(1), 1, white_weight);
  EE_SSR_SAMPLE_GGX(view, float3(0, 0, 1), roughness, xi, float3(0.04f), 1, dielectric_weight);
  if (test >= 7u) {
    float3 reference = float3(0.3f, 0.5f, 2.0f);
    values[index] = EE_SSR_WEIGHTED_CORRECTION(test == 7u ? reference : float3(0), reference,
        float3(0.04f), 0.5f);
    return;
  }
  values[index] = float4(white_weight.x, dielectric_weight.x, dot(direction, reflect(-view, float3(0, 0, 1))), length(direction));
}
