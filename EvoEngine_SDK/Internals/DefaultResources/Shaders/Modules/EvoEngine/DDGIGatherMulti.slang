__exported import EvoEngine.DDGIGather;

[[vk::binding(17, 2)]] public Sampler2D<float4> EE_DDGI_IRRADIANCE_ATLASES[EE_DDGI_MAX_VOLUME_COUNT];
[[vk::binding(18, 2)]] public Sampler2D<float4> EE_DDGI_VISIBILITY_ATLASES[EE_DDGI_MAX_VOLUME_COUNT];
[[vk::binding(19, 2)]]
public StructuredBuffer<float4, Std430DataLayout> EE_DDGI_PROBE_STATE_BLOCKS[EE_DDGI_MAX_VOLUME_COUNT];

struct EeDdgiMultiVolumeResources : IDdgiGatherResources {
  float4 loadProbeState(uint volume_index, uint probe_index) {
    return EE_DDGI_PROBE_STATE_BLOCKS[volume_index][probe_index];
  }
  float4 sampleIrradiance(uint volume_index, float2 uv) {
    return EE_DDGI_IRRADIANCE_ATLASES[volume_index].SampleLevel(uv, 0.0f);
  }
  float4 loadVisibility(uint volume_index, int2 pixel) {
    return EE_DDGI_VISIBILITY_ATLASES[volume_index].Load(int3(pixel, 0));
  }
};

public EeDdgiGatherResult EE_DDGI_GATHER_IRRADIANCE(const float3 normal, const float3 view_direction,
                                                    const float3 position, const bool gather_linear_irradiance) {
  EeDdgiMultiVolumeResources resources;
  const uint cascade_count = min(EE_RENDER_INFO.ddgi_volume_header.x, EE_DDGI_MAX_VOLUME_COUNT);
  EeDdgiGatherResult result = EE_DDGI_EMPTY_GATHER();
  float remaining = 1.0f;
  float weight_sum = 0.0f;
  float visibility_sum = 0.0f;
  for (uint cascade = 0u; cascade < cascade_count && remaining > 0.0f; ++cascade) {
    const DdgiVolumeInfo volume = EE_RENDER_INFO.ddgi_volumes[cascade];
    if (EE_DDGI_VOLUME_COVERAGE(volume, position) <= 0.0f) continue;
    float interior = 1.0f - EE_DDGI_VOLUME_BOUNDARY_WEIGHT(volume, position);
    float allocation = remaining * interior;
    remaining *= 1.0f - interior;
    if (!(allocation > 0.0f) || !(volume.lighting_parameters.x > 0.0f)) continue;
    EeDdgiGatherResult sample = EE_DDGI_GATHER_VOLUME_IRRADIANCE(
        resources, cascade, volume, normal, view_direction, position, gather_linear_irradiance);
    float weight = allocation * sample.confidence;
    result.irradiance += sample.irradiance * weight;
    result.linear_irradiance += sample.linear_irradiance * weight;
    visibility_sum += sample.visibility * weight;
    weight_sum += weight;
  }
  if (weight_sum > 0.0f && isfinite(weight_sum)) {
    result.irradiance /= weight_sum;
    result.linear_irradiance /= weight_sum;
    result.visibility = visibility_sum / weight_sum;
    result.coverage = 1.0f;
    result.confidence = saturate(weight_sum);
  }
  return result;
}
public EeDdgiGatherResult EE_DDGI_GATHER_IRRADIANCE(const float3 normal, const float3 view_direction,
                                                    const float3 position) {
  return EE_DDGI_GATHER_IRRADIANCE(normal, view_direction, position, false);
}
