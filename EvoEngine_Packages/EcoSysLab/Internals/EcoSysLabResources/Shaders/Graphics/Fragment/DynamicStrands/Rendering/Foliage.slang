import EvoEngine.PerFrame;
import EcoSysLab.DynamicStrandsFoliageRenderingConstants;
import EvoEngine.RawGBuffer;

struct EcoSysLabRasterInput {
  [[vk::location(0)]] float3 FragPos : TEXCOORD0;
  [[vk::location(1)]] float3 Normal : TEXCOORD1;
  [[vk::location(2)]] float3 Tangent : TEXCOORD2;
  [[vk::location(3)]] float2 TexCoord : TEXCOORD3;
  [[vk::location(4)]] float4 Color : TEXCOORD4;
};

[shader("fragment")]
RawGBufferOutput main(EcoSysLabRasterInput input, bool is_front_face : SV_IsFrontFace) {
  const Instance instance = EE_INSTANCES[EE_INSTANCE_INDEX];
  const float facing_sign = (is_front_face ? 1.0f : -1.0f) * EE_TRANSFORM_HANDEDNESS(instance.model);
  const bool replace_base_color = input.Color.a > 0.5f;
  const uint info_index = uint(instance.info_index + (replace_base_color ? 2 : 0));
  return EE_BUILD_RAW_GBUFFER(input.TexCoord, input.TexCoord, facing_sign * input.Normal,
                              facing_sign * input.Tangent, facing_sign, uint(EE_INSTANCE_INDEX),
                              uint(instance.material_index), info_index, input.Color, replace_base_color);
}
