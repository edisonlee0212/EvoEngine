import EvoEngine.GltfAlphaMask;

struct FoliageRasterInput {
  [[vk::location(0)]] float3 normal : TEXCOORD0;
  [[vk::location(1)]] float3 tangent : TEXCOORD1;
  [[vk::location(2)]] float2 uv : TEXCOORD2;
  [[vk::location(3)]] nointerpolation uint instance_index : TEXCOORD3;
  [[vk::location(4)]] nointerpolation uint material_index : TEXCOORD4;
};

[shader("fragment")]
void main(FoliageRasterInput input) {
  const GltfAlphaTexCoords tex = EE_GLTF_MAKE_ALPHA_TEX_COORDS(input.uv, input.uv, ddx(input.uv), ddx(input.uv),
                                                              ddy(input.uv), ddy(input.uv));
  if (!EE_GLTF_ALPHA_MASK_PASSES(input.material_index, tex, 1.0)) discard;
}
