// Bindings adapted from Godot shaders/environment/sdfgi_*.glsl at
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
// ABI-only entry point: not a transport/voxelization implementation and never dispatched by the renderer.
#ifndef EE_SDFGI_ABI_ONLY
#error SDFGI algorithm entry point is not implemented yet
#endif
import EvoEngine.SdfgiTypes;


[[vk::binding(0, 5)]] ConstantBuffer<SdfgiGatherData> sdfgi;
[[vk::binding(1, 5)]] Texture2DArray<float3> lightprobe_texture;
[[vk::binding(2, 5)]] Texture3D<float4> occlusion_texture;
[[vk::binding(3, 5)]] SamplerState linear_sampler;
#ifdef EE_SDFGI_RESOURCE_CHECK
[[vk::binding(4, 5)]] RWStructuredBuffer<SdfgiFieldStatus> field_status;
#else
[[vk::binding(4, 5)]] StructuredBuffer<SdfgiFieldStatus> field_status;
#endif

[numthreads(16, 16, 1)]
void main(uint3 id : SV_DispatchThreadID) {
#ifdef EE_SDFGI_RESOURCE_CHECK
  if (any(id != 0)) return;
  uint error = sdfgi.cascades[7].exposure_normalization != 2 || sdfgi.anchor_origin.z != 3 || sdfgi.generation != 5;
  field_status[0].failure_flags |= error != 0 ? 16384u : 0u;
#endif
}
