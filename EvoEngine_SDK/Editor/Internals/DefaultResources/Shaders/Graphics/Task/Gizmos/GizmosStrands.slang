import EvoEngine.PerFrame;
import EvoEngine.GizmosConstants;
import EvoEngine.Strands;
import EvoEngine.StrandMeshlet;

groupshared StrandTaskPayload strand_task;

[numthreads(1, 1, 1)]
[shader("amplification")]
void main(uint3 group_id : SV_GroupID) {
  uint meshlet_index = EE_GIZMOS_CONSTANTS.strand_meshlet_offset + group_id.x;
  StrandMeshlet meshlet = EE_STRAND_MESHLETS[meshlet_index];
  float4x4 model = mul(EE_GET_GIZMO_SCALE_MATRIX(), EE_GIZMOS_CONSTANTS.model_matrix);
  uint interval_count = 0u;
  for (uint segment_index = 0u; segment_index < meshlet.segment_size; ++segment_index) {
    strand_task.interval_offsets[segment_index] = interval_count;
    uint8_t4 segment = meshlet.segments[segment_index];
    float3 world_start = mul(
        float4(EE_STRAND_POINT_DATA_CHUNKS[meshlet.strand_point_chunk_index].strand_points[segment.x].position, 1.0f),
        model).xyz;
    float3 world_end = mul(
        float4(EE_STRAND_POINT_DATA_CHUNKS[meshlet.strand_point_chunk_index].strand_points[segment.w].position, 1.0f),
        model).xyz;
    interval_count += uint(
        EE_STRANDS_SEGMENT_SUBDIVISION(EE_GIZMOS_CONSTANTS.camera_index, world_start, world_end));
  }
  strand_task.interval_offsets[meshlet.segment_size] = interval_count;
  strand_task.instance_index = 0u;
  strand_task.meshlet_index = meshlet_index;
  strand_task.segment_count = meshlet.segment_size;
  DispatchMesh(interval_count, 1, 1, strand_task);
}
