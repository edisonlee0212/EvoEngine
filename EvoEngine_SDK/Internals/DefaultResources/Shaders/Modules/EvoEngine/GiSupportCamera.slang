import EvoEngine.GiSupport;
__exported import EvoEngine.GiSupportOverride;
[[vk::binding(0, 6)]] ConstantBuffer<GiSupportParams> support_params;
[[vk::binding(1, 6)]] StructuredBuffer<GiSupportProbe> support_probes;
[[vk::binding(9, 6)]][vk::image_format("r8ui")] RWTexture2D<uint> support_mask;
[[vk::binding(10, 6)]] Texture3D<uint2> support_voxels;
struct CameraSupportResources : IGiSupportRead {
  GiSupportProbe loadProbe(uint slot) {
    return support_probes[slot];
  }
  uint2 loadVoxels(int3 cell) {
    return support_voxels.Load(int4(cell, 0));
  }
};
public
void EE_GI_CAMERA_SUPPORT(uint2 pixel, float3 position, float3 normal, float3 geometric_normal) {
  uint demand = support_mask[pixel];
  float3 irradiance = float3(0);
  bool valid = false;
  if (demand != 0)
    valid =
        GiSupportSample(CameraSupportResources(), support_params, position, normal, geometric_normal, 1.1, irradiance);
  EE_GI_SUPPORT_SET(position, normal, irradiance * support_params.field.energy, valid, float(demand) / 255);
}
