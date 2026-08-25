import EvoEngine.SMAA;

[[vk::binding(0, 0)]] Sampler2D<float4> edges_tex;
[[vk::binding(1, 0)]] Sampler2D<float4> area_tex;
[[vk::binding(2, 0)]] Sampler2D<float4> search_tex;

[shader("fragment")]
float4 main([[vk::location(0)]] float2 texcoord : TEXCOORD0,
            [[vk::location(1)]] float2 pixcoord : TEXCOORD1,
            [[vk::location(2)]] float4 offset0 : TEXCOORD2,
            [[vk::location(3)]] float4 offset1 : TEXCOORD3,
            [[vk::location(4)]] float4 offset2 : TEXCOORD4) : SV_Target0 {
  float4 offsets[3] = {offset0, offset1, offset2};
#if defined(SMAA_PRESET_LOW)
  return SMAABlendingWeightCalculationPS(SmaaLowPreset(), texcoord, pixcoord, offsets, edges_tex, area_tex,
                                         search_tex, EE_SMAA_CONSTANTS.subsample_indices);
#elif defined(SMAA_PRESET_MEDIUM)
  return SMAABlendingWeightCalculationPS(SmaaMediumPreset(), texcoord, pixcoord, offsets, edges_tex, area_tex,
                                         search_tex, EE_SMAA_CONSTANTS.subsample_indices);
#elif defined(SMAA_PRESET_HIGH)
  return SMAABlendingWeightCalculationPS(SmaaHighPreset(), texcoord, pixcoord, offsets, edges_tex, area_tex,
                                         search_tex, EE_SMAA_CONSTANTS.subsample_indices);
#else
  return SMAABlendingWeightCalculationPS(SmaaUltraPreset(), texcoord, pixcoord, offsets, edges_tex, area_tex,
                                         search_tex, EE_SMAA_CONSTANTS.subsample_indices);
#endif
}
