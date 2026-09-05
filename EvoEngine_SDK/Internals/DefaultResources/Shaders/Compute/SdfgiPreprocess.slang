// Bindings adapted from Godot shaders/environment/sdfgi_*.glsl at
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
// ABI-only entry point: not a transport/voxelization implementation and never dispatched by the renderer.
#ifndef EE_SDFGI_ABI_ONLY
#error SDFGI algorithm entry point is not implemented yet
#endif
import EvoEngine.SdfgiTypes;

[[vk::push_constant]] ConstantBuffer<SdfgiPreprocessPushConstant> params;

#if defined(MODE_INITIALIZE_JUMP_FLOOD) || defined(MODE_INITIALIZE_JUMP_FLOOD_HALF)
[[vk::binding(1, 0)]] [vk::image_format("r16ui")] RWTexture3D<uint> src_color;
[[vk::binding(2, 0)]] [vk::image_format("rgba8ui")] RWTexture3D<uint4> dst_positions;

#elif defined(MODE_JUMPFLOOD) || defined(MODE_JUMPFLOOD_OPTIMIZED)
[[vk::binding(1, 0)]] [vk::image_format("rgba8ui")] RWTexture3D<uint4> src_positions;
[[vk::binding(2, 0)]] [vk::image_format("rgba8ui")] RWTexture3D<uint4> dst_positions;

#elif defined(MODE_UPSCALE_JUMP_FLOOD)
[[vk::binding(1, 0)]] [vk::image_format("r16ui")] RWTexture3D<uint> src_color;
[[vk::binding(2, 0)]] [vk::image_format("rgba8ui")] RWTexture3D<uint4> src_positions_half;
[[vk::binding(3, 0)]] [vk::image_format("rgba8ui")] RWTexture3D<uint4> dst_positions;

#elif defined(MODE_OCCLUSION)
[[vk::binding(1, 0)]] [vk::image_format("r16ui")] RWTexture3D<uint> src_color;
[[vk::binding(2, 0)]] [vk::image_format("r8")] RWTexture3D<float> dst_occlusion[8];
[[vk::binding(3, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> src_facing;

#elif defined(MODE_STORE)
[[vk::binding(1, 0)]] [vk::image_format("rgba8ui")] RWTexture3D<uint4> src_positions;
[[vk::binding(2, 0)]] [vk::image_format("r16ui")] RWTexture3D<uint> src_color;
[[vk::binding(3, 0)]] [vk::image_format("r8")] RWTexture3D<float> src_occlusion[8];
[[vk::binding(4, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> src_emission;
[[vk::binding(5, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> src_emission_aniso;
[[vk::binding(6, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> src_facing;
[[vk::binding(7, 0)]] [vk::image_format("r8")] RWTexture3D<float> dst_sdf;
[[vk::binding(8, 0)]] [vk::image_format("r16ui")] RWTexture3D<uint> dst_occlusion;

[[vk::binding(10, 0)]] RWStructuredBuffer<SdfgiDispatchData> dispatch_data;
[[vk::binding(11, 0)]] RWStructuredBuffer<SdfgiSolidCell> dst_process_voxels;
[[vk::binding(12, 0)]] RWStructuredBuffer<SdfgiFieldStatus> field_status;
#elif defined(MODE_SCROLL)
[[vk::binding(1, 0)]] [vk::image_format("r16ui")] RWTexture3D<uint> dst_albedo;
[[vk::binding(2, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> dst_facing;
[[vk::binding(3, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> dst_light;
[[vk::binding(4, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> dst_light_aniso;

[[vk::binding(5, 0)]] StructuredBuffer<SdfgiDispatchData> dispatch_data;
[[vk::binding(6, 0)]] StructuredBuffer<SdfgiSolidCell> src_process_voxels;
#elif defined(MODE_SCROLL_OCCLUSION)
[[vk::binding(1, 0)]] [vk::image_format("r8")] RWTexture3D<float> dst_occlusion[8];
[[vk::binding(2, 0)]] [vk::image_format("r16ui")] RWTexture3D<uint> src_occlusion;

#endif

#ifdef MODE_JUMPFLOOD_OPTIMIZED
[numthreads(8, 8, 8)]
#elif defined(MODE_OCCLUSION) || defined(MODE_SCROLL)
[numthreads(64, 1, 1)]
#else
[numthreads(4, 4, 4)]
#endif
void main(uint3 id : SV_DispatchThreadID) {}
