struct RuntimeGuiCompositeInput {
  [[vk::location(0)]]
  float2 tex_coord : TEXCOORD0;
};

[[vk::binding(0, 0)]]
Sampler2D overlay;

[shader("fragment")]
float4 main(RuntimeGuiCompositeInput input) : SV_Target {
  return overlay.Sample(input.tex_coord);
}
