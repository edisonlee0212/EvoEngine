import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet1;
import EcoSysLab.Math;
import EcoSysLab.DynamicStrandsSmallSegmentsRenderingConstants;

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

    UniformParticle uniform_particle = uniform_particles[uniform_particle_index_global];

    bool single = uniform_particle.is_single_strand_particle != 0;
    bool prev_single = false;
    bool prev_prev_single = false;
    bool next_single = false;

    bool length_valid = true;
    if (uniform_particle.prev_particle_handle != -1) {
      UniformParticle prev_uniform_particle = uniform_particles[uniform_particle.prev_particle_handle];

      int group_index = segments[uniform_particle.segment_handle].group_index;
      int prev_group_index = segments[prev_uniform_particle.segment_handle].group_index;

      float3 start_position = prev_uniform_particle.position_t.xyz;
      float3 end_position = uniform_particle.position_t.xyz;

      float3 p0 = segments[uniform_particle.segment_handle].particle0.x0;
      float3 p1 = segments[uniform_particle.segment_handle].particle1.x0;

      float3 uniform_particle_p0 =
          lerp(segments[uniform_particle.segment_handle].particle0.x0,
              segments[uniform_particle.segment_handle].particle1.x0, uniform_particle.position_t.w);
      float3 prev_uniform_particle_p0 =
          lerp(segments[prev_uniform_particle.segment_handle].particle0.x0,
              segments[prev_uniform_particle.segment_handle].particle1.x0, prev_uniform_particle.position_t.w);

      float current_length = distance(start_position, end_position);
      float original_length = distance(uniform_particle_p0, prev_uniform_particle_p0);

      length_valid = current_length / original_length < 1.25f;
      prev_single = prev_uniform_particle.is_single_strand_particle != 0;
      if (prev_uniform_particle.prev_particle_handle != -1) {
        prev_prev_single = uniform_particles[prev_uniform_particle.prev_particle_handle].is_single_strand_particle != 0;
      }
    }

    if (uniform_particle.next_particle_handle != -1) {
      next_single = uniform_particles[uniform_particle.next_particle_handle].is_single_strand_particle != 0;
    }

    bool render = !(uniform_particle_index_global > uniform_particles_size - 1) &&
                  uniform_particle.prev_particle_handle != -1 && length_valid;
    // render = render && (single || prev_single || prev_prev_single || next_single);

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
