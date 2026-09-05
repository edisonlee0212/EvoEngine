// Bindings adapted from Godot shaders/environment/sdfgi_*.glsl at
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
// ABI-only entry point: not a transport/voxelization implementation and never dispatched by the renderer.
#ifndef EE_SDFGI_ABI_ONLY
#error SDFGI algorithm entry point is not implemented yet
#endif
import EvoEngine.SdfgiTypes;


[[vk::binding(1, 0)]] Texture3D<float> sdf_cascades[8];
[[vk::binding(2, 0)]] SamplerState linear_sampler;
[[vk::binding(3, 0)]] SamplerState linear_sampler_with_mipmaps;
[[vk::binding(4, 0)]] StructuredBuffer<SdfgiDispatchData> dispatch_data;
[[vk::binding(5, 0)]] RWStructuredBuffer<SdfgiSolidCell> process_voxels;
[[vk::binding(6, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> dst_light;
[[vk::binding(7, 0)]] [vk::image_format("rgba8")] RWTexture3D<float4> dst_aniso0;
[[vk::binding(8, 0)]] [vk::image_format("rg8")] RWTexture3D<float2> dst_aniso1;

[[vk::binding(9, 0)]] ConstantBuffer<SdfgiCascadeBlock> cascades;
[[vk::binding(10, 0)]] StructuredBuffer<SdfgiLight> lights;
[[vk::binding(11, 0)]] Texture2DArray<float3> lightprobe_texture;
[[vk::binding(12, 0)]] Texture3D<float4> occlusion_texture;
[[vk::binding(13, 0)]] RWStructuredBuffer<SdfgiFieldStatus> field_status;
[[vk::push_constant]] ConstantBuffer<SdfgiDirectLightPushConstant> params;

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
#ifdef EE_SDFGI_RESOURCE_CHECK
  if (any(id != 0)) return;
  uint errors = 0;
  errors |= lights[1].host_photometry.w != 13.25 ? 8192u : 0u;
  for (uint c = 0; c < params.max_cascades; ++c) {
    float3 light = lightprobe_texture.SampleLevel(linear_sampler, float3(0.5, 0.5, c), 0);
    errors |= any(abs(light - 1.0) > 0.001) ? 1u : 0u;
  }
  float4 occlusion = occlusion_texture.SampleLevel(linear_sampler, 0.5, 0);
  errors |= any(abs(occlusion - float4(1, 0, 10.0 / 15.0, 5.0 / 15.0)) > 0.001) ? 2u : 0u;
  field_status[0].failure_flags |= errors;
  field_status[0].generation = asuint(lights[1].host_photometry.w);
#endif
}
