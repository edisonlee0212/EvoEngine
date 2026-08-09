// @evoengine-dialect native
struct DdgiProbeRayFragmentInput {
  [[vk::location(0)]]
  float4 color : COLOR0;
};

[shader("fragment")]
float4 main(DdgiProbeRayFragmentInput input) : SV_Target {
  return input.color;
}
