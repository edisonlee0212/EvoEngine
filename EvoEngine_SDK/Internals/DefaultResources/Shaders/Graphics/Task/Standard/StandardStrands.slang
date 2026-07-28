#extension GL_EXT_mesh_shader : require
#extension GL_ARB_shader_draw_parameters : enable
#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.slangh"
#include "PerFrame.slangh"
#include "Strands.slangh"
#include "StrandMeshlet.slangh"

groupshared StrandTaskPayload strand_task;

[numthreads(1, 1, 1)]
[shader("amplification")]
void main() {
  uint instance_index = uint(gl_DrawID + EE_INSTANCE_INDEX);
  uint meshlet_index = EE_INSTANCES[instance_index].meshlet_offset + gl_WorkGroupID.x;
  StrandMeshlet meshlet = EE_STRAND_MESHLETS[meshlet_index];
  mat4 model = EE_INSTANCES[instance_index].model;
  uint interval_count = 0;

  for (uint segment_index = 0; segment_index < meshlet.segment_size; ++segment_index) {
    strand_task.interval_offsets[segment_index] = interval_count;
    u8vec4 segment = meshlet.segments[segment_index];
    vec3 world_start = vec3(model * vec4(
        EE_STRAND_POINT_DATA_CHUNKS[meshlet.strand_point_chunk_index].strand_points[segment.x].position, 1.0));
    vec3 world_end = vec3(model * vec4(
        EE_STRAND_POINT_DATA_CHUNKS[meshlet.strand_point_chunk_index].strand_points[segment.w].position, 1.0));
    interval_count += uint(EE_STRANDS_SEGMENT_SUBDIVISION(world_start, world_end));
  }

  strand_task.interval_offsets[meshlet.segment_size] = interval_count;
  strand_task.instance_index = instance_index;
  strand_task.meshlet_index = meshlet_index;
  strand_task.segment_count = meshlet.segment_size;
  DispatchMesh(interval_count, 1, 1, strand_task);
}
