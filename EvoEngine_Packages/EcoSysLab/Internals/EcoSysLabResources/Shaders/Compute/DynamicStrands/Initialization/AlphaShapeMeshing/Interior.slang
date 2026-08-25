import EcoSysLab.AlphaShape;
import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet0;
import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint ITERATIONS_PER_TASK =
    (COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE;
static const uint2 TET_EDGES[] = {
    uint2(0, 1), uint2(0, 2), uint2(0, 3), uint2(1, 2), uint2(1, 3), uint2(2, 3),
};

struct PushConstants {
  uint tetrahedrons_size;
  float alpha;
  float bifurcation_alpha;
  float max_dist_squared;
};

[[vk::push_constant]] ConstantBuffer<PushConstants> constants;

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint base_id = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  [[unroll]] for (uint i = 0; i < ITERATIONS_PER_TASK; ++i) {
    const uint tetrahedron_index = base_id + lane_id + i * WORKGROUP_SIZE;
    if (tetrahedron_index >= constants.tetrahedrons_size)
      break;

    DelaunayTetrahedron tetrahedron = delaunay_tetrahedrons[tetrahedron_index];
    [[unroll]] for (uint j = 0; j < 4; ++j)
      uniform_particles[tetrahedron.indices[j]].normal_deg = float4(0.0f);

    [[unroll]] for (uint edge = 0; edge < 6; ++edge) {
      const uint j = TET_EDGES[edge].x;
      const uint k = TET_EDGES[edge].y;
      tetrahedron.segment_pair_index[edge] = -1;
      const uint segment_handle0 = uniform_particles[tetrahedron.indices[j]].segment_handle;
      const uint segment_handle1 = uniform_particles[tetrahedron.indices[k]].segment_handle;

      [[unroll]] for (uint l = 0; l < BUNDLE_MAX_CONNECTION; ++l) {
        const int pair_index0 = segment_data_list[segment_handle0].pair_handles[l];
        if (pair_index0 == -1 || tetrahedron.segment_pair_index[edge] != -1)
          break;
        [[unroll]] for (uint m = 0; m < BUNDLE_MAX_CONNECTION; ++m) {
          const int pair_index1 = segment_data_list[segment_handle1].pair_handles[m];
          if (pair_index1 == -1)
            break;
          if (pair_index0 == pair_index1) {
            tetrahedron.segment_pair_index[edge] = pair_index0;
            break;
          }
        }
      }
    }

    float min_distance_squared;
    tetrahedron.inside = int(InsideAlpha(tetrahedron, min_distance_squared, constants.alpha,
                                        constants.bifurcation_alpha, constants.max_dist_squared));
    tetrahedron.inside_at_init = delaunay_tetrahedrons[tetrahedron_index].inside;
    delaunay_tetrahedrons[tetrahedron_index] = tetrahedron;
  }
}
