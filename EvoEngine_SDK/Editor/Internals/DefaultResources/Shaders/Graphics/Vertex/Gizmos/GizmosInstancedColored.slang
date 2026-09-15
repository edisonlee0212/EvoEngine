import EvoEngine.PerFrame;
import EvoEngine.GizmosConstants;
import EvoEngine.InstancedData;

struct GizmoInstancedVertexInput {
  [[vk::location(0)]]
  float3 position : POSITION;
};

struct GizmoColoredVertexOutput {
  float4 position : SV_Position;
  [[vk::location(0)]]
  float4 color : COLOR0;
};

[shader("vertex")]
GizmoColoredVertexOutput main(GizmoInstancedVertexInput input, uint instance_index : SV_InstanceID) {
  GizmoColoredVertexOutput output;
  InstancedData instance = EE_INSTANCED_DATA[instance_index];
  output.color = instance.color;
  float4 position = mul(float4(input.position, 1.0f), EE_GET_GIZMO_SCALE_MATRIX());
  position = mul(position, instance.instance_matrix);
  position = mul(position, EE_GIZMOS_CONSTANTS.model_matrix);
  output.position = mul(position, EE_CAMERAS[EE_GIZMOS_CONSTANTS.camera_index].projection_view);
  return output;
}
