__exported import EvoEngine.GiSupport;
[[vk::binding(0, 1)]] ConstantBuffer<GiSupportParams> support_params;
[[vk::binding(1, 1)]] StructuredBuffer<GiSupportProbe> support_probes;
[[vk::binding(10, 1)]] Texture3D<uint2> support_voxels;
struct LightingSupportResources : IGiSupportRead {
  GiSupportProbe loadProbe(uint slot) {
    return support_probes[slot];
  }
  uint2 loadVoxels(int3 cell) {
    return support_voxels.Load(int4(cell, 0));
  }
};
public
bool EE_GI_LIGHT_SUPPORT(float3 position, float3 normal, out float3 irradiance) {
  // Direct-light cells use absolute scaled coordinates, unlike camera gather coordinates.
  float3 world = position / float3(1, support_params.field.y_mult, 1);
  return GiSupportSample(LightingSupportResources(), support_params, world, normal, normal, 0, irradiance);
}
