import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;
#if defined(EE_ALPHA_MASKED_SHADOW)
import EvoEngine.ShadowRaster;
#define EE_SHADOW_VERTEX_OUTPUT ShadowRasterOutput
#else
#define EE_SHADOW_VERTEX_OUTPUT float4
#endif

struct ShadowVertexInput {
  [[vk::location(0)]] float3 position : POSITION;
#if defined(EE_ALPHA_MASKED_SHADOW)
  [[vk::location(3)]] float2 tex_coord0 : TEXCOORD0;
  [[vk::location(4)]] float4 color : COLOR0;
  [[vk::location(10)]] float2 tex_coord1 : TEXCOORD1;
#endif
};

[shader("vertex")]
EE_SHADOW_VERTEX_OUTPUT main(ShadowVertexInput input, uint draw_id : SV_DrawIndex) {
  uint instance_index =
      EE_RASTER_DRAW_INSTANCE_INDEX(draw_id, EE_BASIC_CONSTANTS.instance_index, EE_BASIC_CONSTANTS.meshlet_culling_flags);
  float4 position = mul(float4(input.position, 1.0f), EE_INSTANCES[instance_index].model);
  float4 clip_position = mul(position, EE_DIRECTIONAL_LIGHTS[EE_BASIC_CONSTANTS.camera_index]
                                          .light_space_matrix[EE_BASIC_CONSTANTS.light_split_index]);
#if defined(EE_ALPHA_MASKED_SHADOW)
  return EE_BUILD_SHADOW_RASTER_OUTPUT(clip_position, input.tex_coord0, input.tex_coord1, input.color, instance_index);
#else
  return clip_position;
#endif
}
