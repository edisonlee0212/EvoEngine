// @evoengine-dialect native
import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;

static const uint WORKGROUP_SIZE = EXT_TASK_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint TASK_MESHLET_ITERATIONS =
    (EXT_TASK_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1u) / WORKGROUP_SIZE;

struct EEMeshTaskPayload {
  uint instanceIndex;
  uint baseID;
  uint8_t deltaIDs[EXT_TASK_WORK_GROUP_INVOCATIONS];
};

groupshared EEMeshTaskPayload task_payload;
groupshared uint shared_meshlet_count;

[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("amplification")]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex, uint draw_id : SV_DrawIndex) {
  uint instance_index = draw_id + uint(EE_BASIC_CONSTANTS.instance_index);
  uint meshlet_offset = EE_INSTANCES[instance_index].meshlet_offset;
  uint meshlet_count = EE_INSTANCES[instance_index].meshlet_size;
  if (lane_id == 0u) shared_meshlet_count = 0u;
  GroupMemoryBarrierWithGroupSync();

  uint group_meshlet_offset = group_id.x * EXT_TASK_WORK_GROUP_INVOCATIONS;
  uint base_id = group_meshlet_offset + meshlet_offset;
  uint output_count = 0u;
  for (uint iteration = 0u; iteration < TASK_MESHLET_ITERATIONS; ++iteration) {
    uint meshlet_local = lane_id + iteration * WORKGROUP_SIZE;
    bool render = group_meshlet_offset + meshlet_local < meshlet_count;
    uint wave_count = WaveActiveCountBits(render);
    uint wave_offset = 0u;
    if (EXT_TASK_SUBGROUP_COUNT > 1u) {
      if (WaveGetLaneIndex() == 0u) InterlockedAdd(shared_meshlet_count, wave_count, wave_offset);
      wave_offset = WaveReadLaneFirst(wave_offset);
    }
    uint output_index = WavePrefixCountBits(render) + wave_offset;
    if (render) task_payload.deltaIDs[output_index] = uint8_t(meshlet_local);
    if (EXT_TASK_SUBGROUP_COUNT == 1u) output_count += wave_count;
  }

  if (EXT_TASK_SUBGROUP_COUNT > 1u) {
    GroupMemoryBarrierWithGroupSync();
    output_count = shared_meshlet_count;
  }
  if (lane_id == 0u) {
    task_payload.baseID = base_id;
    task_payload.instanceIndex = instance_index;
  }
  DispatchMesh(output_count, 1, 1, task_payload);
}
