import EvoEngine.PerFrame;
import EcoSysLab.DynamicStrandsSmallSegmentsRenderingConstants;
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
  const int material_index = input.Color.a > 0.7f ? splinter_material_index : instance.material_index;
  if (material_index != render_material_index) {
    discard;
  }
  const float facing_sign = (is_front_face ? 1.0f : -1.0f) * EE_TRANSFORM_HANDEDNESS(instance.model);
  const bool replace_base_color = input.Color.a > 0.9f;
  const uint info_index = uint(instance.info_index + (replace_base_color ? 2 : 0));
  return EE_BUILD_RAW_GBUFFER(input.TexCoord, input.TexCoord, facing_sign * input.Normal,
                              facing_sign * input.Tangent, facing_sign, uint(EE_INSTANCE_INDEX),
                              uint(material_index), info_index, input.Color, replace_base_color);
}
