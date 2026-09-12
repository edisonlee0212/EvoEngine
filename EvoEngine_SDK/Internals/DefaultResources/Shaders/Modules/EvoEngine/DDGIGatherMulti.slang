__exported import EvoEngine.DDGIGather;

[[vk::binding(17, 2)]] public Sampler2D<float4> EE_DDGI_IRRADIANCE_ATLASES[EE_DDGI_MAX_VOLUME_COUNT];
[[vk::binding(18, 2)]] public Sampler2D<float4> EE_DDGI_VISIBILITY_ATLASES[EE_DDGI_MAX_VOLUME_COUNT];
[[vk::binding(19, 2)]]
public StructuredBuffer<float4, Std430DataLayout> EE_DDGI_PROBE_STATE_BLOCKS[EE_DDGI_MAX_VOLUME_COUNT];

struct EeDdgiMultiVolumeResources : IDdgiGatherResources {
  uint cascadeCount() { return min(EE_RENDER_INFO.ddgi_volume_header.x, EE_DDGI_MAX_VOLUME_COUNT); }
  DdgiVolumeInfo volume(uint index) { return EE_RENDER_INFO.ddgi_volumes[index]; }
  float4 sampleVoxelOcclusion(uint plane, float3 uv) {
    return plane == 0 ? EE_DDGI_VOXEL_OCCLUSION0.SampleLevel(uv, 0) : EE_DDGI_VOXEL_OCCLUSION1.SampleLevel(uv, 0);
  }
  bool voxelGridAvailable() {
    uint width, height, depth;
    EE_DDGI_VOXEL_BITS.GetDimensions(width, height, depth);
    return width > 1;
  }
  uint2 loadVoxelBits(int3 coordinate) { return EE_DDGI_VOXEL_BITS.Load(int4(coordinate, 0)); }

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
  return EE_DDGI_GATHER_CASCADES(EeDdgiMultiVolumeResources(), normal, view_direction, position,
                                 gather_linear_irradiance);
}
public EeDdgiGatherResult EE_DDGI_GATHER_IRRADIANCE(const float3 normal, const float3 view_direction,
                                                    const float3 position) {
  return EE_DDGI_GATHER_IRRADIANCE(normal, view_direction, position, false);
}
