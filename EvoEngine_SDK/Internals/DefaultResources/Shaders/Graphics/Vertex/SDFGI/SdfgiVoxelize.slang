// Bindings adapted from Godot shaders/environment/sdfgi_*.glsl at
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
// ABI-only entry point: not a transport/voxelization implementation and never dispatched by the renderer.
#ifndef EE_SDFGI_ABI_ONLY
#error SDFGI algorithm entry point is not implemented yet
#endif
import EvoEngine.SdfgiTypes;


[[vk::binding(0, 1)]] ConstantBuffer<SdfgiVoxelData> voxel;
[shader("vertex")]
float4 main(uint id : SV_VertexID) : SV_Position {
  return float4(0, 0, 0, 1);
}
