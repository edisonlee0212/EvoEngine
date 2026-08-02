// @evoengine-dialect native
import EvoEngine.PerFrame;
import EvoEngine.GizmosConstants;

struct GizmoVertexColoredInput {
  [[vk::location(0)]]
  float3 position : POSITION;
  [[vk::location(4)]]
  float4 color : COLOR0;
};

struct GizmoColoredVertexOutput {
  float4 position : SV_Position;
  [[vk::location(0)]]
  float4 color : COLOR0;
};

[shader("vertex")]
GizmoColoredVertexOutput main(GizmoVertexColoredInput input) {
  GizmoColoredVertexOutput output;
  output.color = input.color;
  float4 position = mul(float4(input.position, 1.0f), EE_GET_GIZMO_SCALE_MATRIX());
  position = mul(position, EE_GIZMOS_CONSTANTS.model_matrix);
  output.position = mul(position, EE_CAMERAS[EE_GIZMOS_CONSTANTS.camera_index].projection_view);
  return output;
}
