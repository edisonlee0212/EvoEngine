import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;
import EvoEngine.StrandMeshlet;
import EvoEngine.MeshletCulling;

groupshared StrandShadowTaskPayload strand_shadow_task;

[numthreads(1, 1, 1)]
[shader("amplification")]
void main(uint3 group_id : SV_GroupID, uint draw_id : SV_DrawIndex) {
  uint instance_index = draw_id + uint(EE_BASIC_CONSTANTS.instance_index);
  uint meshlet_index = EE_INSTANCES[instance_index].meshlet_offset + group_id.x;
  StrandMeshlet meshlet = EE_STRAND_MESHLETS[meshlet_index];
  float4x4 projection_view;
  uint view_type = EE_BASIC_CONSTANTS.meshlet_culling_flags & 0x300u;
  if (view_type == EE_MESHLET_VIEW_DIRECTIONAL) {
    projection_view = EE_DIRECTIONAL_LIGHTS[EE_BASIC_CONSTANTS.camera_index]
                          .light_space_matrix[EE_BASIC_CONSTANTS.light_split_index];
  } else if (view_type == EE_MESHLET_VIEW_POINT) {
    projection_view = EE_POINT_LIGHTS[EE_BASIC_CONSTANTS.camera_index]
                          .light_space_matrix[EE_BASIC_CONSTANTS.light_split_index];
  } else {
    projection_view = EE_SPOT_LIGHTS[EE_BASIC_CONSTANTS.camera_index].light_space_matrix;
  }
  if ((EE_BASIC_CONSTANTS.meshlet_culling_flags & EE_MESHLET_CULL_FRUSTUM) != 0u &&
      !EE_MESHLET_FRUSTUM_VISIBLE(meshlet.bounding_sphere, EE_INSTANCES[instance_index].model, projection_view, false)) {
    DispatchMesh(0u, 1u, 1u, strand_shadow_task);
    return;
  }
  strand_shadow_task.instance_index = instance_index;
  strand_shadow_task.meshlet_index = meshlet_index;
  DispatchMesh(meshlet.segment_size, 1, 1, strand_shadow_task);
}
