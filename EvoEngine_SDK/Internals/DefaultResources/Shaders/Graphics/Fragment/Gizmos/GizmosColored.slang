struct GizmoColoredFragmentInput {
  [[vk::location(0)]]
  float4 color : COLOR0;
};

[shader("fragment")]
float4 main(GizmoColoredFragmentInput input) : SV_Target {
  return input.color;
}
