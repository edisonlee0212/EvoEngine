import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet1;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = EXT_TASK_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((EXT_TASK_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
struct Task {
  uint baseID;
  uint8_t deltaIDs[EXT_TASK_WORK_GROUP_INVOCATIONS];
};

groupshared Task ts_out;

#if EXT_TASK_SUBGROUP_COUNT > 1
groupshared uint s_out_uniform_particles_count;
#endif
[[vk::push_constant]] cbuffer STRANDS_RENDER_CONSTANTS {
  float4 min_color;
  float4 max_color;

  uint camera_index;
  uint uniform_particles_size;
  uint color_mode;
  uint multiplier;
};

[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("amplification")]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * EXT_TASK_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
#if EXT_TASK_SUBGROUP_COUNT > 1
  if (laneID == 0) {
    s_out_uniform_particles_count = 0;
  }
  GroupMemoryBarrierWithGroupSync();
#endif
  uint out_uniform_particles_count = 0;
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint uniform_particle_index_local = laneID + i * WORKGROUP_SIZE;
    uint uniform_particle_index_global = baseID + uniform_particle_index_local;

    bool render = !(uniform_particle_index_global > uniform_particles_size - 1);

    uint4 vote_uniform_particles = WaveActiveBallot(render);
    uint num_uniform_particles = (countbits(vote_uniform_particles.x) + countbits(vote_uniform_particles.y) + countbits(vote_uniform_particles.z) + countbits(vote_uniform_particles.w));

#if EXT_TASK_SUBGROUP_COUNT > 1
    if (WaveGetLaneIndex() == 0) {
      InterlockedAdd(s_out_uniform_particles_count, num_uniform_particles, out_uniform_particles_count);
    }
    out_uniform_particles_count = WaveReadLaneFirst(out_uniform_particles_count);
#endif
    uint idx_offset = WavePrefixCountBits(render) + out_uniform_particles_count;
    if (render) {
      ts_out.deltaIDs[idx_offset] = uint8_t(uniform_particle_index_local);
    }
#if EXT_TASK_SUBGROUP_COUNT == 1
    out_uniform_particles_count += num_uniform_particles;
#endif
  }

#if EXT_TASK_SUBGROUP_COUNT > 1
  GroupMemoryBarrierWithGroupSync();
  out_uniform_particles_count = s_out_uniform_particles_count;
#endif
  if (laneID == 0) {
    ts_out.baseID = baseID;
  }
  DispatchMesh(out_uniform_particles_count, 1, 1, ts_out);
}
