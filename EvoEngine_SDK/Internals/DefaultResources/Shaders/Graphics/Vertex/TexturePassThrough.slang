// @evoengine-dialect native
struct TexturePassThroughVertexInput {
  [[vk::location(0)]]
  float3 position : POSITION;

  [[vk::location(3)]]
  float2 tex_coord : TEXCOORD0;
};

struct TexturePassThroughVertexOutput {
  float4 position : SV_Position;

  [[vk::location(0)]]
  float2 tex_coord : TEXCOORD0;
};

[shader("vertex")]
TexturePassThroughVertexOutput main(TexturePassThroughVertexInput input) {
  TexturePassThroughVertexOutput output;
  output.tex_coord = input.tex_coord;
  output.position = float4(input.position.x, input.position.y, 0.0f, 1.0f);
  return output;
}
