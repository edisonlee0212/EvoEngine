__exported import EvoEngine.GiSupport;
[[vk::binding(0, 3)]] ConstantBuffer<GiSupportParams> support_params;
[[vk::binding(1, 3)]] StructuredBuffer<GiSupportProbe> support_probes;
[[vk::binding(10, 3)]] Texture3D<uint2> support_voxels;
[[vk::binding(23, 3)]] RWStructuredBuffer<uint> support_pending;
struct TraceSupportResources : IGiSupportRequests {
  GiSupportProbe loadProbe(uint slot) {
    return support_probes[slot];
  }
  uint2 loadVoxels(int3 cell) {
    return support_voxels.Load(int4(cell, 0));
  }
  uint claim(uint slot, uint key) {
    uint old;
    InterlockedCompareExchange(support_pending[slot], 0xffffffffu, key, old);
    return old;
  }
};
public
bool EE_GI_TRACE_SUPPORT(float3 position, float3 normal, float3 geometric_normal, out float3 irradiance) {
  TraceSupportResources resources;
  int4 key;
  float3 start;
  if (GiSupportKey(resources, support_params, position, geometric_normal, 1.1, key, start))
    GiSupportRequest(resources, support_params, key);
  return GiSupportSample(resources, support_params, position, normal, geometric_normal, 1.1, irradiance);
}
