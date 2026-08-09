// @evoengine-dialect native
import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;
import EvoEngine.StrandMeshlet;

groupshared StrandShadowTaskPayload strand_shadow_task;

[numthreads(1, 1, 1)]
[shader("amplification")]
void main(uint3 group_id : SV_GroupID, uint draw_id : SV_DrawIndex) {
  uint instance_index = draw_id + uint(EE_BASIC_CONSTANTS.instance_index);
  uint meshlet_index = EE_INSTANCES[instance_index].meshlet_offset + group_id.x;
  strand_shadow_task.instance_index = instance_index;
  strand_shadow_task.meshlet_index = meshlet_index;
  DispatchMesh(EE_STRAND_MESHLETS[meshlet_index].segment_size, 1, 1, strand_shadow_task);
}
