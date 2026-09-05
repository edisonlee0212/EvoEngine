// Bindings adapted from Godot shaders/environment/sdfgi_*.glsl at
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
// ABI-only entry point: not a transport/voxelization implementation and never dispatched by the renderer.
#ifndef EE_SDFGI_ABI_ONLY
#error SDFGI algorithm entry point is not implemented yet
#endif
import EvoEngine.SdfgiTypes;


[[vk::binding(0, 1)]] ConstantBuffer<SdfgiVoxelData> voxel;
[[vk::binding(1, 1)]] [vk::image_format("r16ui")] RWTexture3D<uint> albedo;
[[vk::binding(2, 1)]] [vk::image_format("r32ui")] RWTexture3D<uint> emission;
[[vk::binding(3, 1)]] [vk::image_format("r32ui")] RWTexture3D<uint> emission_aniso;
[[vk::binding(4, 1)]] [vk::image_format("r32ui")] RWTexture3D<uint> facing;

[shader("fragment")]
void main() {}
