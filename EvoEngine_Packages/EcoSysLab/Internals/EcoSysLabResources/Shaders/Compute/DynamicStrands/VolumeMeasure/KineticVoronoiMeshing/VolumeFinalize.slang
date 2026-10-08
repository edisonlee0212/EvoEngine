import EcoSysLab.KineticVoronoiVolumeMeasure;
import EcoSysLab.VolumeMeasure;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint segment_size;
  uint frame_index;
  int padding0;
  int padding1;
};

// Per-segment |signed sum| → cumulative (matches closed-meshlet divergence volumes).
[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; ++i) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint segment_index = baseID + local_index;
    if (segment_index >= segment_size) {
      break;
    }
    float volume = abs(kinetic_segment_volumes[segment_index]);
    kinetic_segment_volumes[segment_index] = volume;
    if (volume > VOLUME_MEASURE_EPSILON) {
      InterlockedAdd(kinetic_volume_result[0].cumulative_volume, volume);
    }
  }
}
