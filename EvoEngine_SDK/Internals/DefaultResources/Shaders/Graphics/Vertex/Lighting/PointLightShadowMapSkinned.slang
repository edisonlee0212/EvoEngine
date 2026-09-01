import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;
import EvoEngine.Skinning;
#if defined(EE_ALPHA_MASKED_SHADOW)
import EvoEngine.ShadowRaster;
#define EE_SHADOW_VERTEX_OUTPUT ShadowRasterOutput
#else
#define EE_SHADOW_VERTEX_OUTPUT float4
#endif

struct SkinnedShadowVertexInput {
  [[vk::location(0)]] float3 position : POSITION;
#if defined(EE_ALPHA_MASKED_SHADOW)
  [[vk::location(3)]] float2 tex_coord0 : TEXCOORD0;
  [[vk::location(4)]] float4 color : COLOR0;
#endif
  [[vk::location(5)]] int4 bone_ids : BLENDINDICES0;
  [[vk::location(6)]] float4 weights : BLENDWEIGHT0;
  [[vk::location(7)]] int4 bone_ids2 : BLENDINDICES1;
  [[vk::location(8)]] float4 weights2 : BLENDWEIGHT1;
#if defined(EE_ALPHA_MASKED_SHADOW)
  [[vk::location(10)]] float2 tex_coord1 : TEXCOORD1;
#endif
};

[shader("vertex")]
EE_SHADOW_VERTEX_OUTPUT main(SkinnedShadowVertexInput input, uint draw_id : SV_DrawIndex) {
  uint instance_index = draw_id + uint(EE_BASIC_CONSTANTS.instance_index);
  float4 position = mul(float4(input.position, 1.0f),
                        EE_SKINNING_MATRIX(input.bone_ids, input.weights, input.bone_ids2, input.weights2));
  position = mul(position, EE_INSTANCES[instance_index].model);
  float4 clip_position =
      mul(position,
          EE_POINT_LIGHTS[EE_BASIC_CONSTANTS.camera_index].light_space_matrix[EE_BASIC_CONSTANTS.light_split_index]);
#if defined(EE_ALPHA_MASKED_SHADOW)
  return EE_BUILD_SHADOW_RASTER_OUTPUT(clip_position, input.tex_coord0, input.tex_coord1, input.color, instance_index);
#else
  return clip_position;
#endif
}
