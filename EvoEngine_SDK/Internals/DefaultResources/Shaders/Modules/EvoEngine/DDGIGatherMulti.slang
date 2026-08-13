// @evoengine-dialect native
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
  float4 sampleVisibility(uint volume_index, float2 uv) {
    return EE_DDGI_VISIBILITY_ATLASES[volume_index].SampleLevel(uv, 0.0f);
  }
};

public EeDdgiGatherResult EE_DDGI_GATHER_IRRADIANCE(const float3 normal, const float3 view_direction,
                                                    const float3 position, const bool gather_linear_irradiance) {
  EeDdgiMultiVolumeResources resources;
  const uint volume_count = min(EE_RENDER_INFO.ddgi_volume_header.x, EE_DDGI_MAX_VOLUME_COUNT);
  uint primary_index = EE_DDGI_MAX_VOLUME_COUNT;
  for (uint i = 0u; i < volume_count; ++i) {
    const DdgiVolumeInfo volume = EE_RENDER_INFO.ddgi_volumes[i];
    if (volume.lighting_parameters.x > 0.0f && EE_DDGI_VOLUME_COVERAGE(volume, position) > 0.0f) {
      primary_index = i;
      break;
    }
  }
  if (primary_index == EE_DDGI_MAX_VOLUME_COUNT) return EE_DDGI_EMPTY_GATHER();

  const DdgiVolumeInfo primary_volume = EE_RENDER_INFO.ddgi_volumes[primary_index];
  EeDdgiGatherResult primary =
      EE_DDGI_GATHER_VOLUME_IRRADIANCE(resources, primary_index, primary_volume, normal, view_direction, position,
                                       gather_linear_irradiance);
  const float boundary_weight = EE_DDGI_VOLUME_BOUNDARY_WEIGHT(primary_volume, position);
  if (!(boundary_weight > 0.0f)) return primary;

  uint secondary_index = EE_DDGI_MAX_VOLUME_COUNT;
  float secondary_coverage = 0.0f;
  for (uint i = 0u; i < volume_count; ++i) {
    if (i == primary_index) continue;
    const DdgiVolumeInfo volume = EE_RENDER_INFO.ddgi_volumes[i];
    if (!(volume.lighting_parameters.x > 0.0f)) continue;
    const float coverage = EE_DDGI_VOLUME_COVERAGE(volume, position);
    if (coverage > 0.0f) {
      secondary_index = i;
      secondary_coverage = coverage;
      break;
    }
  }
  if (secondary_index == EE_DDGI_MAX_VOLUME_COUNT) return primary;

  EeDdgiGatherResult secondary = EE_DDGI_GATHER_VOLUME_IRRADIANCE(
      resources, secondary_index, EE_RENDER_INFO.ddgi_volumes[secondary_index], normal, view_direction, position,
      gather_linear_irradiance);
  const float primary_coverage = max(primary.coverage, 0.0f);
  const float weighted_secondary_coverage = max(secondary_coverage, 0.0f) * boundary_weight;
  const float coverage_sum = primary_coverage + weighted_secondary_coverage;
  if (!(coverage_sum > 0.0f) || isnan(coverage_sum) || isinf(coverage_sum)) return primary;

  const float primary_effective_weight = primary_coverage / coverage_sum * primary.confidence;
  const float secondary_effective_weight =
      weighted_secondary_coverage / coverage_sum * secondary.confidence;
  const float confidence = primary_effective_weight + secondary_effective_weight;
  EeDdgiGatherResult result = EE_DDGI_EMPTY_GATHER();
  result.coverage = clamp(coverage_sum, 0.0f, 1.0f);
  result.confidence = clamp(confidence, 0.0f, 1.0f);
  if (confidence > 0.0f && !isnan(confidence) && !isinf(confidence)) {
    result.irradiance =
        (primary.irradiance * primary_effective_weight + secondary.irradiance * secondary_effective_weight) /
        confidence;
    result.linear_irradiance =
        (primary.linear_irradiance * primary_effective_weight +
         secondary.linear_irradiance * secondary_effective_weight) /
        confidence;
    result.visibility =
        clamp((primary.visibility * primary_effective_weight + secondary.visibility * secondary_effective_weight) /
                  confidence,
              0.0f, 1.0f);
  }
  return result;
}
public EeDdgiGatherResult EE_DDGI_GATHER_IRRADIANCE(const float3 normal, const float3 view_direction,
                                                    const float3 position) {
  return EE_DDGI_GATHER_IRRADIANCE(normal, view_direction, position, false);
}
