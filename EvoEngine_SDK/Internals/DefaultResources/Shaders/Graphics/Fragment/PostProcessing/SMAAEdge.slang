// @evoengine-dialect native
import EvoEngine.SMAA;

[[vk::binding(0, 0)]]
Sampler2D<float4> color_tex;

[shader("fragment")]
float4 main([[vk::location(0)]] float2 texcoord : TEXCOORD0,
            [[vk::location(1)]] float4 offset0 : TEXCOORD1,
            [[vk::location(2)]] float4 offset1 : TEXCOORD2,
            [[vk::location(3)]] float4 offset2 : TEXCOORD3) : SV_Target0 {
  float4 offsets[3] = {offset0, offset1, offset2};
#if defined(SMAA_PRESET_LOW)
  return float4(SMAALumaEdgeDetectionPS(SmaaLowPreset(), texcoord, offsets, color_tex), 0.0f, 0.0f);
#elif defined(SMAA_PRESET_MEDIUM)
  return float4(SMAALumaEdgeDetectionPS(SmaaMediumPreset(), texcoord, offsets, color_tex), 0.0f, 0.0f);
#elif defined(SMAA_PRESET_HIGH)
  return float4(SMAALumaEdgeDetectionPS(SmaaHighPreset(), texcoord, offsets, color_tex), 0.0f, 0.0f);
#else
  return float4(SMAALumaEdgeDetectionPS(SmaaUltraPreset(), texcoord, offsets, color_tex), 0.0f, 0.0f);
#endif
}
