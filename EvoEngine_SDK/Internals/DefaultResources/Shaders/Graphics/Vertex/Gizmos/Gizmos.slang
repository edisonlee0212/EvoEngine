import EvoEngine.PerFrame;
import EvoEngine.GizmosConstants;

struct GizmoVertexInput {
  [[vk::location(0)]]
  float3 position : POSITION;
};

[shader("vertex")]
float4 main(GizmoVertexInput input) : SV_Position {
  float4 position = mul(float4(input.position, 1.0f), EE_GET_GIZMO_SCALE_MATRIX());
  position = mul(position, EE_GIZMOS_CONSTANTS.model_matrix);
  return mul(position, EE_CAMERAS[EE_GIZMOS_CONSTANTS.camera_index].projection_view);
}
