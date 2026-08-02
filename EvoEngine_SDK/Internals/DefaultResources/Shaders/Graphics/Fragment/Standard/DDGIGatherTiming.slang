// @evoengine-dialect native
import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;
import EvoEngine.DDGIGatherMulti;

struct DdgiGatherTimingInput {
  [[vk::location(0)]] float2 tex_coord : TEXCOORD0;
};

[[vk::binding(17, 1)]] Sampler2D inDepth;
[[vk::binding(20, 1)]] Sampler2D inBaseColorAO;
[[vk::binding(21, 1)]] Sampler2D inNormalRoughness;

[shader("fragment")]
float4 main(DdgiGatherTimingInput input) : SV_Target {
  float ndc_depth = inDepth.Sample(input.tex_coord).x;
  if (ndc_depth == 1.0f) return float4(0.0f);
  int camera_index = EE_BASIC_CONSTANTS.camera_index;
  float3 world_position = EE_DEPTH_TO_WORLD_POS(camera_index, input.tex_coord, ndc_depth);
  float3 albedo = inBaseColorAO.Sample(input.tex_coord).rgb;
  float3 normal = normalize(inNormalRoughness.Sample(input.tex_coord).xyz);
  float3 view_direction = normalize(EE_CAMERA_POSITION(camera_index) - world_position);
  return float4(EE_DDGI_WEIGHTED_DIFFUSE(
                    EE_DDGI_GATHER_IRRADIANCE(normal, view_direction, world_position), albedo),
                1.0f);
}
