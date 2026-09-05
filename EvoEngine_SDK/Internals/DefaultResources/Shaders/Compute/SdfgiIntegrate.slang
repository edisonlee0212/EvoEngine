// Bindings adapted from Godot shaders/environment/sdfgi_*.glsl at
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
// ABI-only entry point: not a transport/voxelization implementation and never dispatched by the renderer.
#ifndef EE_SDFGI_ABI_ONLY
#error SDFGI algorithm entry point is not implemented yet
#endif
import EvoEngine.SdfgiTypes;


[[vk::binding(1, 0)]] Texture3D<float> sdf_cascades[8];
[[vk::binding(2, 0)]] Texture3D<float3> light_cascades[8];
[[vk::binding(3, 0)]] Texture3D<float4> aniso0_cascades[8];
[[vk::binding(4, 0)]] Texture3D<float2> aniso1_cascades[8];
[[vk::binding(6, 0)]] SamplerState linear_sampler;
[[vk::binding(7, 0)]] ConstantBuffer<SdfgiCascadeBlock> cascades;
[[vk::binding(8, 0)]] [vk::image_format("r32ui")] RWTexture2DArray<uint> lightprobe_texture_data;
[[vk::binding(9, 0)]] [vk::image_format("rgba16i")] RWTexture2DArray<int4> lightprobe_history_texture;
[[vk::binding(10, 0)]] [vk::image_format("rgba32i")] RWTexture2D<int4> lightprobe_average_texture;
[[vk::binding(11, 0)]] [vk::image_format("rgba16i")] RWTexture2DArray<int4> lightprobe_history_scroll_texture;
[[vk::binding(12, 0)]] [vk::image_format("rgba32i")] RWTexture2D<int4> lightprobe_average_scroll_texture;
[[vk::binding(13, 0)]] [vk::image_format("rgba32i")] RWTexture2D<int4> lightprobe_average_parent_texture;
[[vk::binding(14, 0)]] [vk::image_format("rgba16f")] RWTexture2DArray<float4> lightprobe_ambient_texture;

[[vk::binding(15, 0)]] RWStructuredBuffer<SdfgiFieldStatus> field_status;
[[vk::binding(0, 1)]] TextureCube<float4> sky_irradiance;
[[vk::binding(1, 1)]] SamplerState linear_sampler_mipmaps;
[[vk::push_constant]] ConstantBuffer<SdfgiIntegratePushConstant> params;

[numthreads(8, 8, 1)]
void main(uint3 id : SV_DispatchThreadID) {
#ifdef EE_SDFGI_RESOURCE_CHECK
  if (any(id != 0)) return;
  uint errors = 0;
  for (uint c = 0; c < 8; ++c) {
    errors |= sdf_cascades[c].SampleLevel(linear_sampler, 0.5, 0) != 0 ? 4u : 0u;
    errors |= any(abs(light_cascades[c].SampleLevel(linear_sampler, 0.5, 0) - 1.0) > 0.001) ? 8u : 0u;
    errors |= any(aniso0_cascades[c].SampleLevel(linear_sampler, 0.5, 0) != 0) ? 16u : 0u;
    errors |= any(aniso1_cascades[c].SampleLevel(linear_sampler, 0.5, 0) != 0) ? 32u : 0u;
  }
  for (uint h = 0; h < params.history_size; ++h) {
    errors |= any(lightprobe_history_texture[int3(288, 271, h)] != 0) ? 64u : 0u;
    errors |= any(lightprobe_history_scroll_texture[int3(288, 271, h)] != 0) ? 128u : 0u;
  }
  errors |= any(lightprobe_average_texture[int2(288, 271)] != 0) ? 256u : 0u;
  errors |= any(lightprobe_average_scroll_texture[int2(288, 271)] != 0) ? 512u : 0u;
  errors |= any(lightprobe_average_parent_texture[int2(288, 271)] != 0) ? 1024u : 0u;
  for (uint c = 0; c < params.max_cascades; ++c)
    errors |= any(lightprobe_ambient_texture[int3(288, 16, c)] != 0) ? 2048u : 0u;
  errors |= any(sky_irradiance.SampleLevel(linear_sampler_mipmaps, float3(0, 1, 0), 0) != 0) ? 4096u : 0u;
  field_status[0].failure_flags |= errors;
  field_status[0].generation = asuint(cascades.data[7].pad2.w);
#endif
}
