// Debug resource/coordinate adapter for Godot SDFGI, 34d06658a85845111a50db9e485ec4a0701d4298.
// See docs/licenses/Godot-MIT.txt.
import EvoEngine.SdfgiTypes;

public struct SdfgiDebugCameraData {
  public float4x4 view_projection;
  public float4x4 inverse_view_projection;
  public float4 origin_y_mult;
  public uint4 selection; // view, cascade, logical XZY probe, slice
  public uint4 field; // cascade count, box count, depth test, unused
  public SdfgiCascadeData cascades[8];
};
public struct SdfgiDebugBox { public float4 minimum; public float4 maximum; public float4 color; };
[[vk::binding(1, 0)]] public Texture3D<float> sdf_textures[8];
[[vk::binding(2, 0)]] public Texture3D<float3> light_textures[8];
[[vk::binding(3, 0)]] public Texture3D<float4> aniso0_textures[8];
[[vk::binding(4, 0)]] public Texture3D<float2> aniso1_textures[8];
[[vk::binding(5, 0)]] public Texture3D<float4> occlusion_texture;
[[vk::binding(6, 0)]] public Texture2DArray<float3> probe_texture;
[[vk::binding(7, 0)]] public SamplerState linear_sampler;
[[vk::binding(8, 0)]] public ConstantBuffer<SdfgiDebugCameraData> data;
[[vk::binding(9, 0)]] [vk::image_format("rgba32f")] public RWTexture2D<float4> output_image;
[[vk::binding(10, 0)]] public Texture2D<float> scene_depth;
[[vk::binding(11, 0)]] public StructuredBuffer<SdfgiDebugBox> boxes;

public float3 debug_world(float2 uv, float depth) {
  float4 world = mul(float4(uv * 2 - 1, depth, 1), data.inverse_view_projection);
  return world.xyz / world.w;
}
public int3 debug_probe_cell(uint index) {
  return int3(index % 17, index / (17 * 17), (index / 17) % 17);
}
public float3 debug_probe_world(uint index) {
  uint c = data.selection.y;
  return (data.cascades[c].offset + float3(debug_probe_cell(index)) * 8 / data.cascades[c].to_cell) /
      float3(1, data.origin_y_mult.w, 1);
}
public float2 debug_oct(float3 normal) {
  normal /= dot(abs(normal), float3(1));
  if (normal.z < 0) normal.xy = (1 - abs(normal.yx)) * float2(normal.x >= 0 ? 1 : -1, normal.y >= 0 ? 1 : -1);
  return normal.xy * 0.5 + 0.5;
}
public float3 debug_probe_color(uint index, float3 normal) {
  int3 cell = debug_probe_cell(index);
  float2 texel = float2(cell.x + cell.z * 17, cell.y) * 8 + 1 + debug_oct(normal) * 6;
  return probe_texture.SampleLevel(linear_sampler, float3(texel / float2(2312, 136), data.selection.y), 0);
}
