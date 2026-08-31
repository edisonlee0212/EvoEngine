import EvoEngine.GltfBindlessMaterial;

struct FragmentInput {
  [[vk::location(0)]] float2 uv0 : TEXCOORD0;
  [[vk::location(1)]] float2 uv1 : TEXCOORD1;
  [[vk::location(2)]] float2 uv2 : TEXCOORD2;
  [[vk::location(3)]] float2 uv3 : TEXCOORD3;
  [[vk::location(4)]] nointerpolation uint material_index : TEXCOORD4;
};

float4 main(FragmentInput input) : SV_Target0 {
  const GltfTexCoords tex_coords = EE_GLTF_MAKE_TEX_COORDS_WITH_GRADIENTS(
      input.uv0, input.uv1, input.uv2, input.uv3, ddx(input.uv0), ddx(input.uv1), ddx(input.uv2), ddx(input.uv3),
      ddy(input.uv0), ddy(input.uv1), ddy(input.uv2), ddy(input.uv3));
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[input.material_index];
  return EE_GLTF_SAMPLE_TEXTURE_SLOT(material.pbr_base_color_texture, 0, tex_coords, float4(1.0));
}
