import EvoEngine.PerFrame;
import EvoEngine.RawGBuffer;

struct TreeRasterInput {
  [[vk::location(0)]] float3 normal : TEXCOORD0;
  [[vk::location(1)]] float3 tangent : TEXCOORD1;
  [[vk::location(2)]] float2 uv : TEXCOORD2;
  [[vk::location(3)]] nointerpolation uint instance_index : TEXCOORD3;
  [[vk::location(4)]] nointerpolation uint material_index : TEXCOORD4;
};

[shader("fragment")]
RawGBufferOutput main(TreeRasterInput input, bool is_front_face : SV_IsFrontFace) {
  Instance instance = EE_INSTANCES[input.instance_index];
  float facing = (is_front_face ? 1.0 : -1.0) * EE_TRANSFORM_HANDEDNESS(instance.model);
  return EE_BUILD_RAW_GBUFFER(input.uv, input.uv, input.normal * facing, input.tangent * facing, facing,
                              input.instance_index, input.material_index, instance.info_index,
                              float4(1, 1, 1, 1), false);
}
