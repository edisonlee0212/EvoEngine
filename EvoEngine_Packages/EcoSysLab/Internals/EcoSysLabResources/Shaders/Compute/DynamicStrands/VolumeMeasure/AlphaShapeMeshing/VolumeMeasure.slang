import EcoSysLab.AlphaShapeVolumeMeasure;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint tetrahedron_size;
  uint frame_index;
  int padding0;
  int padding1;
};

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; ++i) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint tet_id = baseID + local_index;
    if (tet_id >= tetrahedron_size) {
      break;
    }
    float volume = MeasureAlphaTetrahedronVolume(tet_id);
    alpha_tet_current_volumes[tet_id] = volume;
    if (volume > 0.0) {
      InterlockedAdd(alpha_volume_result[0].cumulative_volume, volume);
    }
  }
}
