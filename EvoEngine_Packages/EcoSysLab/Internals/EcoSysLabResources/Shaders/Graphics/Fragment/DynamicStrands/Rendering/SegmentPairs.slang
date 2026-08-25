import EvoEngine.PerFrame;
import EvoEngine.GltfBindlessMaterial;

[[vk::push_constant]] cbuffer STRANDS_RENDER_CONSTANTS {
  float4 color0;
  float4 color1;

  float4 position_scale;

  int EE_CAMERA_INDEX;
  uint segment_pairs_size;
  uint color_mode;
  int material_index;
  float multiplier;
  float factor;
};
struct EcoSysLabRasterInput {
  [[vk::location(0)]] float3 FragPos : TEXCOORD0;
  [[vk::location(1)]] float3 Normal : TEXCOORD1;
  [[vk::location(2)]] float3 Tangent : TEXCOORD2;
  [[vk::location(3)]] float2 TexCoord : TEXCOORD3;
  [[vk::location(4)]] float4 Color : TEXCOORD4;
};

struct EcoSysLabFragmentOutput {
  [[vk::location(0)]] float4 out_color : SV_Target0;
};
[shader("fragment")]
EcoSysLabFragmentOutput main(EcoSysLabRasterInput fs_in, bool is_front_face : SV_IsFrontFace) {
  EcoSysLabFragmentOutput output;
  float2 tex_coord = fs_in.TexCoord;
  float tex_grad = max(length(ddx(tex_coord)), length(ddy(tex_coord)));
  GltfRasterMaterial surface =
      EE_EVALUATE_GLTF_RASTER_SURFACE(uint(material_index), tex_coord, tex_coord, tex_grad);
  if (EE_GLTF_RASTER_SHOULD_DISCARD(surface))
    discard;

  float3 normal = EE_EVALUATE_GLTF_RASTER_NORMAL(uint(material_index), tex_coord, tex_coord, fs_in.Normal, fs_in.Tangent,
                                                 1.0, tex_grad);

  // also store the per-fragment normals into the gbuffer
  normal = normalize((is_front_face ? 1.0 : -1.0) * normal);

  output.out_color = float4(1, 0, 0, 0.5);
  return output;
}
