// @evoengine-dialect native
import EvoEngine.SMAA;

[[vk::binding(0, 0)]] Sampler2D<float4> color_tex;
[[vk::binding(1, 0)]] Sampler2D<float4> blend_tex;
[[vk::binding(2, 0)]] Sampler2D<float4> edges_tex;

float3 LinearToSrgb(float3 color) {
  float3 low = color * 12.92f;
  float3 high = 1.055f * pow(max(color, float3(0.0f)), float3(1.0f / 2.4f)) - 0.055f;
  return lerp(low, high, color > float3(0.0031308f));
}

[shader("fragment")]
float4 main([[vk::location(0)]] float2 texcoord : TEXCOORD0,
            [[vk::location(1)]] float4 offset : TEXCOORD1) : SV_Target0 {
  if (EE_SMAA_CONSTANTS.debug_mode == 1) {
    float2 edges = edges_tex.Sample(texcoord).rg;
    return float4(edges.r, edges.g, 0.0f, 1.0f);
  }
  if (EE_SMAA_CONSTANTS.debug_mode == 2) {
    return float4(blend_tex.Sample(texcoord).rgb, 1.0f);
  }
  float4 color = SMAANeighborhoodBlendingPS(texcoord, offset, color_tex, blend_tex);
  if (EE_SMAA_CONSTANTS.tone_mapped != 0) {
    color.rgb = LinearToSrgb(max(color.rgb, float3(0.0f)));
  }
  return color;
}
