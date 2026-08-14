// @evoengine-dialect native
__exported import EvoEngine.DDGIGather;

[[vk::binding(17, 2)]] public Sampler2D<float4> EE_DDGI_IRRADIANCE_ATLAS;
[[vk::binding(18, 2)]] public Sampler2D<float4> EE_DDGI_VISIBILITY_ATLAS;
[[vk::binding(1, 2)]] public StructuredBuffer<float4, Std430DataLayout> EE_DDGI_PROBE_STATE;

struct EeDdgiSingleVolumeResources : IDdgiGatherResources {
  float4 loadProbeState(uint volume_index, uint probe_index) {
    return EE_DDGI_PROBE_STATE[probe_index];
  }
  float4 sampleIrradiance(uint volume_index, float2 uv) {
    return EE_DDGI_IRRADIANCE_ATLAS.SampleLevel(uv, 0.0f);
  }
  float4 sampleVisibility(uint volume_index, float2 uv) {
    return EE_DDGI_VISIBILITY_ATLAS.SampleLevel(uv, 0.0f);
  }
};

public EeDdgiGatherResult EE_DDGI_GATHER_IRRADIANCE(const uint selected_volume_index, const float3 normal,
                                                    const float3 view_direction, const float3 position) {
  const uint volume_index = min(selected_volume_index, EE_DDGI_MAX_VOLUME_COUNT - 1u);
  return EE_DDGI_GATHER_VOLUME_IRRADIANCE(EeDdgiSingleVolumeResources(), volume_index,
                                          EE_RENDER_INFO.ddgi_volumes[volume_index], normal, view_direction,
                                          position, false);
}
