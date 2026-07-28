#extension GL_EXT_mesh_shader : require
#extension GL_ARB_shader_draw_parameters : enable
#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.slangh"
#include "PerFrame.slangh"
#include "StrandMeshlet.slangh"

groupshared StrandShadowTaskPayload strand_shadow_task;

[numthreads(1, 1, 1)]
[shader("amplification")]
void main() {
  uint instance_index = uint(gl_DrawID + EE_INSTANCE_INDEX);
  uint meshlet_index = EE_INSTANCES[instance_index].meshlet_offset + gl_WorkGroupID.x;
  strand_shadow_task.instance_index = instance_index;
  strand_shadow_task.meshlet_index = meshlet_index;
  DispatchMesh(EE_STRAND_MESHLETS[meshlet_index].segment_size, 1, 1, strand_shadow_task);
}
