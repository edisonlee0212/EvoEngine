__exported import EvoEngine.HddagiGather;
import EvoEngine.HddagiProbe;

public
struct HddagiCameraParams {
 public
  uint2 viewport;
 public
  uint2 gi_size;
 public
  uint pixel_stride;
 public
  uint reflection_capture;
 public
  uint filter_radius;
 public
  uint padding;
};
[[vk::binding(0, 5)]] public
ConstantBuffer<HddagiGatherData> EE_HDDAGI;
[[vk::binding(1, 5)]] public
Texture2DArray<float3> EE_HDDAGI_DIFFUSE;
[[vk::binding(2, 5)]] public
Texture2DArray<float3> EE_HDDAGI_SPECULAR;
[[vk::binding(3, 5)]] public
Texture3D<float4> EE_HDDAGI_OCCLUSION0;
[[vk::binding(4, 5)]] public
Texture3D<float4> EE_HDDAGI_OCCLUSION1;
[[vk::binding(5, 5)]] public
SamplerState EE_HDDAGI_SAMPLER;
[[vk::binding(6, 5)]] public
Texture3D<uint2> EE_HDDAGI_VOXELS;
[[vk::binding(7, 5)]][vk::image_format("r8ui")] public Texture3D<uint> EE_HDDAGI_REGIONS;
[[vk::binding(8, 5)]] public
Texture3D<float3> EE_HDDAGI_LIGHT;
[[vk::binding(9, 5)]][vk::image_format("r8ui")] public Texture3D<uint> EE_HDDAGI_DISOCCLUSION;
[[vk::binding(10, 5)]] public
Texture3D<uint> EE_HDDAGI_NEIGHBORS;
[[vk::binding(11, 5)]] public
StructuredBuffer<uint> EE_HDDAGI_STATUS;
[[vk::binding(12, 5)]][vk::image_format("rgba16f")] public RWTexture2D<float4> EE_HDDAGI_SURFACE_OUT;
[[vk::binding(13, 5)]] public
Texture2D<float4> EE_HDDAGI_SURFACE;
[[vk::binding(14, 5)]][vk::image_format("r32ui")] public RWTexture2D<uint> EE_HDDAGI_DIFFUSE_OUT;
[[vk::binding(15, 5)]][vk::image_format("r32ui")] public RWTexture2D<uint> EE_HDDAGI_SPECULAR_OUT;
[[vk::binding(16, 5)]][vk::image_format("rg8")] public RWTexture2D<float2> EE_HDDAGI_BLEND_OUT;
[[vk::binding(17, 5)]] public
Texture2D<float3> EE_HDDAGI_CAMERA_DIFFUSE;
[[vk::binding(18, 5)]] public
Texture2D<float3> EE_HDDAGI_CAMERA_SPECULAR;
[[vk::binding(19, 5)]] public
Texture2D<float2> EE_HDDAGI_CAMERA_BLEND;
[[vk::binding(20, 5)]] public
StructuredBuffer<uint> EE_HDDAGI_RECEIVER_CLASS;
[[vk::binding(21, 5)]] public
ConstantBuffer<HddagiCameraParams> EE_HDDAGI_CAMERA;
[[vk::binding(22, 5)]] public
Texture2D<float3> EE_HDDAGI_SPECULAR_SCRATCH;
[[vk::binding(23, 5)]] public
Texture2D<float2> EE_HDDAGI_BLEND_SCRATCH;
[[vk::binding(24, 5)]][vk::image_format("r32ui")] public RWTexture2D<uint> EE_HDDAGI_SPECULAR_SCRATCH_OUT;
[[vk::binding(25, 5)]][vk::image_format("rg8")] public RWTexture2D<float2> EE_HDDAGI_BLEND_SCRATCH_OUT;

public
void EE_HDDAGI_CAMERA_GATHER(float3 position, float3 normal, float3 reflection, float roughness, bool dynamic_receiver,
                             uint2 pixel, out float4 diffuse, out float4 specular) {
  diffuse = specular = float4(0);
  if (EE_HDDAGI_STATUS[2] == 0 || EE_HDDAGI_STATUS[1] != 0)
    return;
  HddagiGatherLighting(EE_HDDAGI, EE_HDDAGI_DIFFUSE, EE_HDDAGI_SPECULAR, EE_HDDAGI_OCCLUSION0, EE_HDDAGI_OCCLUSION1,
                       EE_HDDAGI_SAMPLER, EE_HDDAGI_VOXELS, EE_HDDAGI_REGIONS, EE_HDDAGI_LIGHT, EE_HDDAGI_DISOCCLUSION,
                       EE_HDDAGI_NEIGHBORS, position, normal, reflection, roughness, dynamic_receiver,
                       EE_HDDAGI_CAMERA.reflection_capture == 0, pixel, diffuse, specular);
}
