#include "MarchingCubeMeshGenerator.hpp"
#include "MeshGenUtils.hpp"

using namespace evo_engine;
namespace eco_sys_lab_plugin {

void MarchingCubeMeshGenerator::Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                         std::vector<unsigned int>& indices,
                                         const StrandModelMeshGeneratorSettings& settings) {
  MarchingCube(strand_model, vertices, indices, settings);
}

void MarchingCubeMeshGenerator::Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                         std::vector<glm::vec2>& tex_coords,
                                         std::vector<std::pair<unsigned int, unsigned int>>& index_pairs,
                                         const StrandModelMeshGeneratorSettings& settings) {
  // so far, we don't support texture coordinates
  std::vector<unsigned> indices;

  Generate(strand_model, vertices, indices, settings);

  for (Vertex& v : vertices) {
    tex_coords.emplace_back(v.tex_coord);
  }

  for (size_t index : indices) {
    index_pairs.emplace_back(index, index);
  }
}

std::vector<glm::ivec3> MarchingCubeMeshGenerator::VoxelizeLineSeg(glm::vec3 start, glm::vec3 end,
                                                                 float voxel_side_length) {
  // Based on Amanatides, J., & Woo, A. (1987, August). A fast voxel traversal algorithm for ray tracing. In
  // Eurographics (Vol. 87, No. 3, pp. 3-10).
  std::vector<glm::ivec3> ret_val;
  glm::vec3 dir = end - start;
  glm::vec3 dir_in_voxels = dir / voxel_side_length;
  glm::ivec3 step = glm::sign(dir);

  // determine voxel of start point
  glm::vec3 pos_in_voxels = start / voxel_side_length;
  glm::ivec3 start_voxel = glm::floor(pos_in_voxels);
  ret_val.push_back(start_voxel);
  glm::ivec3 end_voxel = glm::floor(end / voxel_side_length);

  // compute t deltas
  glm::vec3 t_delta = glm::vec3(voxel_side_length) / glm::abs(dir);

  // compute max value for t within voxel
  glm::ivec3 next_border = RoundInDir(pos_in_voxels, step);
  glm::vec3 dist_to_next_border = glm::vec3(next_border) - pos_in_voxels;
  glm::vec3 t_max = glm::abs(dist_to_next_border / dir_in_voxels);
  if (dir_in_voxels.x <= glm::epsilon<float>())
    t_max.x = 0.0f;
  if (dir_in_voxels.y <= glm::epsilon<float>())
    t_max.y = 0.0f;
  if (dir_in_voxels.z <= glm::epsilon<float>())
    t_max.z = 0.0f;
  // min and max for debug assert
  glm::vec3 min_voxel = glm::min(start_voxel, end_voxel);
  glm::vec3 max_voxel = glm::max(start_voxel, end_voxel);

  // now traverse voxels until we reach the end point
  glm::ivec3 voxel = start_voxel;
  while (voxel != end_voxel) {
    size_t min_index = 0;

    for (size_t i = 1; i < 3; i++) {
      if (t_max[i] < t_max[min_index]) {
        min_index = i;
      }
    }

    // update tMax and determine next voxel;
    voxel[min_index] += step[min_index];

    // check that we do not run out of range
    // This can happen due to numerical inaccuracies when adding tDelta
    for (size_t dim = 0; dim < 3; dim++) {
      if (!(min_voxel[dim] <= voxel[dim] && voxel[dim] <= max_voxel[dim])) {
        ret_val.push_back(end_voxel);
        return ret_val;
      }
    }

    ret_val.push_back(voxel);
    t_max[min_index] += t_delta[min_index];
  }

  return ret_val;
}

void MarchingCubeMeshGenerator::MarchingCube(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                            std::vector<unsigned>& indices,
                                            const StrandModelMeshGeneratorSettings& settings) {
  const auto& skeleton = strand_model.strand_model_skeleton;
  const auto& strand_group = skeleton.data.strand_group;
  // first compute extreme points
  auto min = glm::vec3(std::numeric_limits<float>::infinity());
  auto max = glm::vec3(-std::numeric_limits<float>::infinity());
  bool need_triangulation = false;
  for (uint32_t strand_segment_index = 0; strand_segment_index < strand_group.PeekStrandSegments().size();
       strand_segment_index++) {
    const auto& strand_segment = strand_group.PeekStrandSegment(strand_segment_index);
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(strand_segment_index);
    const auto& node = skeleton.PeekNode(strand_segment_data.node_handle);
    const auto& profile = node.data.profile;
    if (profile.PeekParticles().size() < settings.min_cell_count_for_major_branches)
      continue;
    need_triangulation = true;
    min = glm::min(strand_segment.end_position, min);
    max = glm::max(strand_segment.end_position, max);
  }
  if (need_triangulation) {
    min -= glm::vec3(0.1f);
    max += glm::vec3(0.1f);
    const auto box_size = max - min;
    Octree<bool> octree;
    if (settings.auto_level) {
      const float max_radius =
          glm::max(glm::max(box_size.x, box_size.y), box_size.z) * 0.5f + 2.0f * settings.marching_cube_radius;
      int subdivision_level = -1;
      float test_radius = settings.marching_cube_radius;
      while (test_radius <= max_radius) {
        subdivision_level++;
        test_radius *= 2.f;
      }
      EVOENGINE_LOG("Mesh formation: Auto set level to " + std::to_string(subdivision_level))
      octree.Reset(max_radius, subdivision_level, (min + max) * 0.5f);
    } else {
      octree.Reset(glm::max((box_size.x, box_size.y), glm::max(box_size.y, box_size.z)) * 0.5f,
                   glm::clamp(settings.voxel_subdivision_level, 4, 16), (min + max) / 2.0f);
    }
    float subdivision_length = settings.marching_cube_radius * 0.5f;

    for (StrandSegmentHandle strand_segment_handle = 0;
         strand_segment_handle < strand_group.PeekStrandSegments().size(); strand_segment_handle++) {
      const auto& strand_segment = strand_group.PeekStrandSegment(strand_segment_handle);
      const auto& strand_segment_data = strand_group.PeekStrandSegmentData(strand_segment_handle);
      const auto& node = skeleton.PeekNode(strand_segment_data.node_handle);
      const auto& profile = node.data.profile;
      if (profile.PeekParticles().size() < settings.min_cell_count_for_major_branches)
        continue;

      // Get interpolated position on pipe segment. Example to get middle point here:
      const auto start_position = strand_model.InterpolateStrandSegmentPosition(strand_segment_handle, 0.0f);
      const auto end_position = strand_model.InterpolateStrandSegmentPosition(strand_segment_handle, 1.0f);
      const auto distance = glm::distance(start_position, end_position);
      const auto step_size = glm::max(1, static_cast<int>(distance / subdivision_length));

      const auto polar_x =
          profile.PeekParticle(strand_segment_data.profile_particle_handle).GetInitialPolarPosition().y /
          glm::radians(360.0f);
      for (int step = 0; step < step_size; step++) {
        const auto a = static_cast<float>(step) / step_size;
        const auto position = strand_model.InterpolateStrandSegmentPosition(strand_segment_handle, a);

        octree.Occupy(position, [&](OctreeNode&) {
        });
      }
    }
    octree.TriangulateField(vertices, indices, settings.remove_duplicate);
  }
}
}  // namespace eco_sys_lab_plugin
