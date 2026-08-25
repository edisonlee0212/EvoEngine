import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EcoSysLab.KineticVoronoiMeshingSet1;
import EcoSysLab.KineticVoronoiMeshing;
import EcoSysLab.KineticVoronoiRenderingConstants;

static const uint WORKGROUP_SIZE = EXT_TASK_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((EXT_TASK_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
struct Task {
  uint baseID;
  uint8_t deltaIDs[EXT_TASK_WORK_GROUP_INVOCATIONS];
};

groupshared Task ts_out;

#if EXT_TASK_SUBGROUP_COUNT > 1
groupshared uint s_out_triangle_count;
#endif
[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("amplification")]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * EXT_TASK_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
#if EXT_TASK_SUBGROUP_COUNT > 1
  if (laneID == 0) {
    s_out_triangle_count = 0;
  }
  GroupMemoryBarrierWithGroupSync();
#endif
  uint out_triangle_count = 0;
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint triangle_index_local = laneID + i * WORKGROUP_SIZE;
    uint triangle_index_global = baseID + triangle_index_local;

    if (triangle_index_global >= triangle_count)
      break;

    bool render = true;

    uint4 vote_triangles = WaveActiveBallot(render);
    uint num_triangles = (countbits(vote_triangles.x) + countbits(vote_triangles.y) + countbits(vote_triangles.z) + countbits(vote_triangles.w));

#if EXT_TASK_SUBGROUP_COUNT > 1
    if (WaveGetLaneIndex() == 0) {
      InterlockedAdd(s_out_triangle_count, num_triangles, out_triangle_count);
    }
    out_triangle_count = WaveReadLaneFirst(out_triangle_count);
#endif
    uint idx_offset = WavePrefixCountBits(render) + out_triangle_count;
    if (render) {
      ts_out.deltaIDs[idx_offset] = uint8_t(triangle_index_local);
    }
#if EXT_TASK_SUBGROUP_COUNT == 1
    out_triangle_count += num_triangles;
#endif
  }

#if EXT_TASK_SUBGROUP_COUNT > 1
  GroupMemoryBarrierWithGroupSync();
  out_triangle_count = s_out_triangle_count;
#endif
  if (laneID == 0) {
    ts_out.baseID = baseID;
  }
  DispatchMesh(out_triangle_count, 1, 1, ts_out);
}