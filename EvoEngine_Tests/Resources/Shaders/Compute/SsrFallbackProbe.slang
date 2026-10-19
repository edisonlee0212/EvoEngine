import EvoEngine.Lighting;
[[vk::binding(0, 1)]] RWStructuredBuffer<float4> values;
struct ProbeResources : ILightingResources {
  float3 sampleSky(float3 d) { return float3(0); }
  float3 sampleReflectionProbe(int index, float3 d, float lod) { return float3(7, 8, 9); }
  float3 sampleIrradiance(float3 d) { return float3(0.5f); }
  float3 samplePrefiltered(float3 d, float lod) { return float3(0.25f, 0.5f, 0.75f); }
  uint prefilteredLevelCount() { return 1; }
  float2 sampleBrdf(float2 uv) { return float2(1, 0); }
}
[shader("compute")][numthreads(1, 1, 1)] void main() {
  uint mode = EE_BASIC_CONSTANTS.meshlet_culling_flags;
  float weight = mode == 0u ? 0.0f : (mode == 2u ? 0.25f : 1.0f);
  float3 irradiance = mode == 6u ? float3(0) : float3(2, 3, 4) * EE_DDGI_PI;
  EeEnvironmentalLighting environment = EE_FUNC_CALCULATE_ENVIRONMENTAL_COMPONENTS(ProbeResources(),
      float3(1), float3(0, 0, 1), float3(0, 0, 1), float3(0), 0, 0.5f, float3(0.04f), 1, 1, 1,
      irradiance, weight, 1);
  values[0] = float4(environment.specular_radiance, environment.specular_visibility);
}
