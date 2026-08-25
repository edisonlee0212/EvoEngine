import EcoSysLab.Math;
import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet0;
import EcoSysLab.AlphaShape;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint delaunay_tetrahedron_size;
  float alpha;
  float bifurcation_alpha;
  float max_dist_squared;
  int render_complex;
  float degen_triangle_threshold;
};
[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint delaunay_tetrahedron_index = baseID + local_index;

    if (delaunay_tetrahedron_index >= delaunay_tetrahedron_size)
      break;

    DelaunayTetrahedron delaunay_tetrahedron = delaunay_tetrahedrons[delaunay_tetrahedron_index];

    if (delaunay_tetrahedron.inside == 0)
      continue;

    // determine which triangles need to be rendered
    [[unroll]]
    for (uint triangle_index = 0; triangle_index < 4; ++triangle_index) {
      // check if we are at the boundary of the alpha shape
      if (render_complex == 0 && delaunay_tetrahedron.neighbor_tet_ids[triangle_index] != -1) {
        delaunay_tetrahedron.render_neighbor[triangle_index] =
            1 - delaunay_tetrahedrons[delaunay_tetrahedron.neighbor_tet_ids[triangle_index]].inside;
      } else {
        delaunay_tetrahedron.render_neighbor[triangle_index] = 1;
      }
    }
    // filter rogue tetrahedrons
    if (render_complex == 0 && delaunay_tetrahedron.render_neighbor[0] == 1 &&
        delaunay_tetrahedron.render_neighbor[1] == 1 && delaunay_tetrahedron.render_neighbor[2] == 1 &&
        delaunay_tetrahedron.render_neighbor[3] == 1) {
      // delaunay_tetrahedron.inside = 0;
    } else {
      // count how many tetrahedrons are adjacent to each uniform particle
      [[unroll]]
      for (uint j = 0; j < 4; ++j) {
        uniform_particles[delaunay_tetrahedron.indices[j]].is_single_strand_particle = 0;
      }

      [[unroll]]
      for (uint triangle_index = 0; triangle_index < 4; ++triangle_index) {
        if (delaunay_tetrahedron.render_neighbor[triangle_index] != 1)
          continue;

        float4 normal_deg = float4(ComputeTetTriangleNormal(delaunay_tetrahedron, triangle_index), 1.0f);

        // add normal to all triangle vertices, so we can later compute an average
        [[unroll]]
        for (int j = 0; j < 4; j++) {
          if (j != triangle_index && uniform_particles[delaunay_tetrahedron.indices[j]].is_on_surface == 0) {
            InterlockedAdd(uniform_particles[delaunay_tetrahedron.indices[j]].normal_deg.x, normal_deg.x);
            InterlockedAdd(uniform_particles[delaunay_tetrahedron.indices[j]].normal_deg.y, normal_deg.y);
            InterlockedAdd(uniform_particles[delaunay_tetrahedron.indices[j]].normal_deg.z, normal_deg.z);
            // DEBUG: count how many normals are added, disabled for efficiency
            // InterlockedAdd(uniform_particles[delaunay_tetrahedron.indices[j]].normal_deg.w, normal_deg.w);
          }
        }
      }
    }

    delaunay_tetrahedrons[delaunay_tetrahedron_index] = delaunay_tetrahedron;
  }
}
