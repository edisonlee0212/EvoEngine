import EcoSysLab.KineticVoronoiVolumeMeasure;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint triangle_size;
  uint segment_size;
  uint frame_index;
  int padding0;
};

// Accumulate signed divergence contributions into per-segment scratch.
[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; ++i) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint tri_id = baseID + local_index;
    if (tri_id >= triangle_size) {
      break;
    }
    int segment_index = -1;
    float signed_volume = MeasureKineticTriangleSignedVolume(tri_id, segment_index);
    if (segment_index < 0 || uint(segment_index) >= segment_size) {
      continue;
    }
    if (signed_volume != 0.0) {
      InterlockedAdd(kinetic_segment_volumes[segment_index], signed_volume);
    }
  }
}
