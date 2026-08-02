// @evoengine-dialect native
struct TexturePassThroughFragmentInput {
  [[vk::location(0)]]
  float2 tex_coord : TEXCOORD0;
};

[[vk::binding(0, 0)]]
Sampler2D screenTexture;

[shader("fragment")]
float4 main(TexturePassThroughFragmentInput input) : SV_Target {
  return screenTexture.Sample(float2(input.tex_coord.x, 1.0f - input.tex_coord.y));
}
