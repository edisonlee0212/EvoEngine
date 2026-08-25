import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet1;
import EcoSysLab.AlphaShapeRenderingConstants;

static const uint WORKGROUP_SIZE = EXT_TASK_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((EXT_TASK_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
struct Task {
  uint baseID;
  uint8_t deltaIDs[EXT_TASK_WORK_GROUP_INVOCATIONS];
};

groupshared Task ts_out;

#if EXT_TASK_SUBGROUP_COUNT > 1
groupshared uint s_out_tetrahedron_count;
#endif
[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("amplification")]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * EXT_TASK_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
#if EXT_TASK_SUBGROUP_COUNT > 1
  if (laneID == 0) {
    s_out_tetrahedron_count = 0;
  }
  GroupMemoryBarrierWithGroupSync();
#endif
  uint out_tetrahedron_count = 0;
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint tetrahedron_index_local = laneID + i * WORKGROUP_SIZE;
    uint tetrahedron_index_global = baseID + tetrahedron_index_local;

    if (tetrahedron_index_global >= tetrahedrons_size)
      break;
    bool render = true;

    delaunay_tetrahedrons[tetrahedron_index_global].mesh_looked_at = 0;
    delaunay_tetrahedrons[tetrahedron_index_global].task_looked_at = 0;
    delaunay_tetrahedrons[tetrahedron_index_global].triangles_accepted = 0;

    InterlockedAdd(delaunay_tetrahedrons[tetrahedron_index_global].task_looked_at, 1);
    DelaunayTetrahedron delaunay_tetrahedron = delaunay_tetrahedrons[tetrahedron_index_global];

    if (delaunay_tetrahedrons[tetrahedron_index_global].inside == 0) {
      render = false;
    } else {
      // determine which triangles need to be rendered
      [[unroll]]
      for (uint triangle_index = 0; triangle_index < 4; ++triangle_index) {
        if (delaunay_tetrahedrons[tetrahedron_index_global].render_neighbor[triangle_index] == 1) {
          InterlockedAdd(delaunay_tetrahedrons[tetrahedron_index_global].triangles_accepted, 1);
        }
      }
    }

    uint4 vote_tetrahedrons = WaveActiveBallot(render);
    uint num_tetrahedrons = (countbits(vote_tetrahedrons.x) + countbits(vote_tetrahedrons.y) + countbits(vote_tetrahedrons.z) + countbits(vote_tetrahedrons.w));

#if EXT_TASK_SUBGROUP_COUNT > 1
    if (WaveGetLaneIndex() == 0) {
      InterlockedAdd(s_out_tetrahedron_count, num_tetrahedrons, out_tetrahedron_count);
    }
    out_tetrahedron_count = WaveReadLaneFirst(out_tetrahedron_count);
#endif
    uint idx_offset = WavePrefixCountBits(render) + out_tetrahedron_count;
    if (render) {
      ts_out.deltaIDs[idx_offset] = uint8_t(tetrahedron_index_local);
    }
#if EXT_TASK_SUBGROUP_COUNT == 1
    out_tetrahedron_count += num_tetrahedrons;
#endif
  }

#if EXT_TASK_SUBGROUP_COUNT > 1
  GroupMemoryBarrierWithGroupSync();
  out_tetrahedron_count = s_out_tetrahedron_count;
#endif
  if (laneID == 0) {
    ts_out.baseID = baseID;
  }
  DispatchMesh(out_tetrahedron_count, 1, 1, ts_out);
}
