#include "StrandModelMeshGenerator.hpp"
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/io.hpp>
#include <queue>
#include "Curve.hpp"
#include "Delaunator2D.hpp"
#include "Jobs.hpp"
#include "MeshGenUtils.hpp"
#include "Octree.hpp"
#include "TreeDescriptor.hpp"
#include "TreeMeshGenerator.hpp"

#include "EcoSysLabLayer.hpp"

#define DEBUG_OUTPUT false

using namespace eco_sys_lab;

typedef std::vector<std::pair<StrandHandle, glm::vec3>> Slice;
typedef std::vector<StrandHandle> PipeCluster;

void StrandModelMeshGeneratorSettings::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Combo("Mode", {"Recursive Slicing", "Marching Cube"}, generator_type);
  if (generator_type == 0 && ImGui::TreeNode("Recursive Slicing settings")) {
    ImGui::DragInt("Steps per segment", &steps_per_segment, 1.0f, 1, 99);

    // ImGui::Checkbox("[DEBUG] Limit Profile Iterations", &m_limitProfileIterations);
    // ImGui::DragInt("[DEBUG] Limit", &m_maxProfileIterations);

    ImGui::DragFloat("[DEBUG] MaxParam", &max_param);
    // ImGui::Checkbox("Compute branch joints", &branch_connections);
    ImGui::DragInt("uCoord multiplier", &u_multiplier, 1, 1);
    ImGui::DragFloat("vCoord multiplier", &v_multiplier, 0.1f);
    ImGui::DragFloat("cluster distance factor", &cluster_distance, 0.1f, 1.0f, 10.0f);
    ImGui::TreePop();
  }

  if (generator_type == 1 && ImGui::TreeNode("Marching Cube settings")) {
    ImGui::Checkbox("Auto set level", &auto_level);
    if (!auto_level)
      ImGui::DragInt("Voxel subdivision level", &voxel_subdivision_level, 1, 5, 16);
    else
      ImGui::DragFloat("Min Cube size", &marching_cube_radius, 0.0001f, 0.001f, 1.0f);
    if (smooth_iteration == 0)
      ImGui::Checkbox("Remove duplicate", &remove_duplicate);
    ImGui::ColorEdit4("Marching cube color", &marching_cube_color.x);
    ImGui::ColorEdit4("Cylindrical color", &cylindrical_color.x);
    ImGui::DragInt("uCoord multiplier", &root_distance_multiplier, 1, 1, 100);
    ImGui::DragFloat("vCoord multiplier", &circle_multiplier, 0.1f);
    ImGui::TreePop();
  }

  ImGui::DragInt("Major branch cell min", &min_cell_count_for_major_branches, 1, 0, 1000);
  ImGui::DragInt("Minor branch cell max", &max_cell_count_for_minor_branches, 1, 0, 1000);

  ImGui::Checkbox("Recalculate UV", &recalculate_uv);
  ImGui::Checkbox("Fast UV", &fast_uv);
  ImGui::DragInt("Smooth iteration", &smooth_iteration, 0, 0, 10);
  ImGui::Checkbox("Branch", &enable_branch);
  ImGui::Checkbox("Foliage", &enable_foliage);
}

void StrandModelMeshGenerator::Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                        std::vector<unsigned>& indices,
                                        const StrandModelMeshGeneratorSettings& settings) {
  switch (settings.generator_type) {
    case StrandModelMeshGeneratorType::RecursiveSlicing: {
      RecursiveSlicing(strand_model, vertices, indices, settings);
    } break;
    case StrandModelMeshGeneratorType::MarchingCube: {
      MarchingCube(strand_model, vertices, indices, settings);
    } break;
  }

  if (settings.recalculate_uv ||
      settings.generator_type == static_cast<unsigned>(StrandModelMeshGeneratorType::MarchingCube)) {
    CalculateUv(strand_model, vertices, settings);
  }

  for (int i = 0; i < settings.smooth_iteration; i++) {
    MeshSmoothing(vertices, indices);
  }
  CylindricalMeshing(strand_model, vertices, indices, settings);

  CalculateNormal(vertices, indices);
}

void StrandModelMeshGenerator::Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                        std::vector<glm::vec2>& tex_coords,
                                        std::vector<std::pair<unsigned, unsigned>>& indices,
                                        const StrandModelMeshGeneratorSettings& settings) {
  RecursiveSlicing(strand_model, vertices, tex_coords, indices, settings);
  for (int i = 0; i < settings.smooth_iteration; i++) {
    MeshSmoothing(vertices, indices);
  }
  std::vector<unsigned int> temp_indices{};
  CylindricalMeshing(strand_model, vertices, temp_indices, settings);
  for (const auto& index : temp_indices) {
    const auto& vertex = vertices.at(index);
    const auto tex_coords_index = tex_coords.size();
    tex_coords.emplace_back(vertex.tex_coord);
    indices.emplace_back(index, tex_coords_index);
  }

  CalculateNormal(vertices, indices);
}

auto RoundInDir(float val, const int dir) -> int {
  if (dir > 0) {
    return static_cast<int>(std::ceilf(val));
  } 
  return static_cast<int>(std::floorf(val));
}

glm::ivec3 RoundInDir(glm::vec3 val, glm::ivec3 dir) {
  return glm::ivec3(RoundInDir(val[0], dir[0]), RoundInDir(val[1], dir[1]), RoundInDir(val[2], dir[2]));
}

void VerifyMesh(std::vector<Vertex>& vertices, std::vector<unsigned>& indices) {
  if (DEBUG_OUTPUT)
    std::cout << "checking indices for " << vertices.size() << " vertices..." << std::endl;
  for (size_t index : indices) {
    if (index >= vertices.size()) {
      std::cerr << "index " << index << " is out of range" << std::endl;
    }
  }
}

std::vector<glm::ivec3> VoxelizeLineSeg(glm::vec3 start, glm::vec3 end, float voxel_side_length) {
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

std::vector<StrandSegmentHandle> GetNextSegGroup(const StrandModelStrandGroup& pipes,
                                                 const std::vector<StrandSegmentHandle>& seg_group) {
  std::vector<StrandSegmentHandle> next_seg_group;

  for (const StrandSegmentHandle& seg_handle : seg_group) {
    auto& seg = pipes.PeekStrandSegment(seg_handle);
    if (!seg.IsEnd()) {
      next_seg_group.push_back(seg.GetNextHandle());
    }
  }

  return next_seg_group;
}

glm::vec3 GetSegDir(const StrandModel& strand_model, const StrandSegmentHandle seg_handle, float t = 1.0f) {
  return strand_model.InterpolateStrandSegmentAxis(seg_handle, t);
}

auto GetSegPos(const StrandModel& strand_model, const StrandSegmentHandle seg_handle, float t = 1.0f) -> glm::vec3 {
  auto ret_val = strand_model.InterpolateStrandSegmentPosition(seg_handle, t);
  for (size_t i = 0; i < 3; i++) {
    if (std::isinf(ret_val[i])) {
      std::cerr << "Error: Interpolated segment position is infinity" << std::endl;
    }

    if (std::isnan(ret_val[i])) {
      std::cerr << "Error: Interpolated segment position is not a number" << std::endl;
    }
  }
  return ret_val;
}

SkeletonNodeHandle GetNodeHandle(const StrandModelStrandGroup& pipe_group, const StrandHandle& pipe_handle, float t) {
  const size_t lookup_index = glm::round(t) < pipe_group.PeekStrand(pipe_handle).PeekStrandSegmentHandles().size()
                                ? glm::round(t)
                                : (pipe_group.PeekStrand(pipe_handle).PeekStrandSegmentHandles().size() - 1);
  auto& pipe_segment_handle = pipe_group.PeekStrand(pipe_handle).PeekStrandSegmentHandles()[lookup_index];
  auto& pipe_segment = pipe_group.PeekStrandSegment(pipe_segment_handle);

  return pipe_segment.data.node_handle;
}

bool IsValidPipeParam(const StrandModel& strand_model, const StrandHandle& pipe_handle, float t) {
  const auto& pipe = strand_model.strand_model_skeleton.data.strand_group.PeekStrand(pipe_handle);
  return pipe.PeekStrandSegmentHandles().size() > glm::floor(t);
}

glm::vec3 GetPipeDir(const StrandModel& strand_model, const StrandHandle& pipe_handle, float t) {
  const auto& pipe = strand_model.strand_model_skeleton.data.strand_group.PeekStrand(pipe_handle);
  auto seg_handle = pipe.PeekStrandSegmentHandles()[t];
  return GetSegDir(strand_model, seg_handle, fmod(t, 1.0));
}

glm::vec3 GetPipePos(const StrandModel& strand_model, const StrandHandle& pipe_handle, float t) {
  const auto& pipe = strand_model.strand_model_skeleton.data.strand_group.PeekStrand(pipe_handle);
  auto seg_handle = pipe.PeekStrandSegmentHandles()[t];
  return GetSegPos(strand_model, seg_handle, fmod(t, 1.0));
}

const Particle2D<CellParticlePhysicsData>& GetEndParticle(const StrandModel& strand_model,
                                                          const StrandHandle& pipe_handle, size_t index) {
  if (!IsValidPipeParam(strand_model, pipe_handle, index)) {
    std::cerr << "Error: Strand " << pipe_handle << " does not exist at " << index << std::endl;
  }

  const auto& skeleton = strand_model.strand_model_skeleton;
  const auto& pipe = skeleton.data.strand_group.PeekStrand(pipe_handle);
  StrandSegmentHandle seg_handle = pipe.PeekStrandSegmentHandles()[index];
  auto& pipe_segment = skeleton.data.strand_group.PeekStrandSegment(seg_handle);

  const auto& node = skeleton.PeekNode(pipe_segment.data.node_handle);
  const auto& start_profile = node.data.profile;
  // To access the user's defined constraints (attractors, etc.)
  const auto& profile_constraints = node.data.profile_constraints;

  // To access the position of the start of the pipe segment within a profile:
  const auto parent_handle = node.GetParentHandle();
  const auto& end_particle = start_profile.PeekParticle(pipe_segment.data.profile_particle_handle);

  return end_particle;
}

const Particle2D<CellParticlePhysicsData>& GetStartParticle(const StrandModel& strand_model,
                                                            const StrandHandle& pipe_handle, size_t index) {
  if (!IsValidPipeParam(strand_model, pipe_handle, index)) {
    std::cerr << "Error: Strand " << pipe_handle << " does not exist at " << index << std::endl;
  }

  const auto& skeleton = strand_model.strand_model_skeleton;
  const auto& pipe = skeleton.data.strand_group.PeekStrand(pipe_handle);
  const auto seg_handle = pipe.PeekStrandSegmentHandles()[index];
  auto& pipe_segment = skeleton.data.strand_group.PeekStrandSegment(seg_handle);

  const auto& node = skeleton.PeekNode(pipe_segment.data.node_handle);
  const auto& start_profile = node.data.profile;
  // To access the user's defined constraints (attractors, etc.)
  const auto& profile_constraints = node.data.profile_constraints;
  // To access the position of the start of the pipe segment within a profile:
  const auto& start_particle = start_profile.PeekParticle(pipe_segment.data.profile_particle_handle);
  return start_particle;
}

float GetPipePolar(const StrandModel& strand_model, const StrandHandle& pipe_handle, float t) {
  // cheap interpolation, maybe improve this later ?
  const auto& p0 = GetStartParticle(strand_model, pipe_handle, std::floor(t));
  const auto& p1 = GetEndParticle(strand_model, pipe_handle, std::floor(t));
  float a1 = p1.GetPolarPosition().y;

  if (IsValidPipeParam(strand_model, pipe_handle, std::ceil(t))) {
    const auto& p1 = GetStartParticle(strand_model, pipe_handle, std::ceil(t));
    a1 = p1.GetPolarPosition().y;
  }

  float a0 = p0.GetPolarPosition().y;

  float interpolation_param = fmod(t, 1.0f);

  // we will just assume that the difference cannot exceed 180 degrees
  if (a1 < a0) {
    std::swap(a0, a1);
    interpolation_param = 1 - interpolation_param;
  }

  float angle;

  if (a1 - a0 > glm::pi<float>()) {
    // rotation wraps around
    angle =
        fmod((a0 + 2 * glm::pi<float>()) * (1 - interpolation_param) + a1 * interpolation_param, 2 * glm::pi<float>());

    if (angle > glm::pi<float>()) {
      angle -= 2 * glm::pi<float>();
    }
  } else {
    angle = a0 * (1 - interpolation_param) + a1 * interpolation_param;

    if (angle > glm::pi<float>()) {
      angle -= 2 * glm::pi<float>();
    }
  }

  return angle;
}

void Dfs(const Graph& g, size_t v, std::vector<size_t>& component_members, std::vector<bool>& visited) {
  std::vector<size_t> stack;

  stack.push_back(v);

  while (!stack.empty()) {
    size_t u = stack.back();
    stack.pop_back();

    if (!visited[u]) {
      component_members.push_back(u);
      visited[u] = true;
    }

    for (size_t av : g.adjacentVertices(u)) {
      if (!visited[av]) {
        stack.push_back(av);
      }
    }
  }
}

void Delaunay(Graph& g, float removal_length, std::vector<size_t>& candidates) {
  std::vector<float> positions;

  for (size_t index : candidates) {
    // if(DEBUG_OUTPUT) std::cout << "adding vertex " << index << std::endl;
    positions.push_back(g[index].x);
    positions.push_back(g[index].y);
  }

  // if(DEBUG_OUTPUT) std::cout << "calling delaunator" << std::endl;
  const Delaunator::Delaunator2D d(positions);
  // if(DEBUG_OUTPUT) std::cout << "collecting edges" << std::endl;
  for (std::size_t i = 0; i < d.triangles.size(); i += 3) {
    const auto& v0 = candidates[d.triangles[i]];
    const auto& v1 = candidates[d.triangles[i + 1]];
    const auto& v2 = candidates[d.triangles[i + 2]];

    // make an exception if the three indices belong to the same skeleton part
    // does not work properly, just creates artefacts
    /*StrandHandle p0 = prevPipes[v0];
    StrandHandle p1 = prevPipes[v1];
    StrandHandle p2 = prevPipes[v2];

    SkeletonNodeHandle n0 = getNodeHandle(pipeGroup, p0, t);
    SkeletonNodeHandle n1 = getNodeHandle(pipeGroup, p1, t);
    SkeletonNodeHandle n2 = getNodeHandle(pipeGroup, p2, t);

    if (n0 == n1 && n1 == n2)
    {
            g.addEdge(v0, v1);
            g.addEdge(v1, v2);
            g.addEdge(v2, v0);

            continue; // Don't add twice
    }*/

    if (glm::distance(g[v0], g[v1]) > removal_length || glm::distance(g[v1], g[v2]) > removal_length ||
        glm::distance(g[v0], g[v2]) > removal_length)
      continue;

    // if(DEBUG_OUTPUT) std::cout << "adding triangle " << v0 << ", " << v1 << ", " << v2 << std::endl;
    g.addEdge(v0, v1);
    g.addEdge(v1, v2);
    g.addEdge(v2, v0);
  }
}

std::vector<size_t> CollectComponent(const Graph& g, size_t start_index) {
  std::vector<size_t> component_members;

  // just do a dfs to collect everything
  std::vector<bool> visited(g.m_vertices.size(), false);

  Dfs(g, start_index, component_members, visited);
  return component_members;
}

std::vector<StrandSegmentHandle> GetSegGroup(const StrandModelStrandGroup& pipes, size_t index) {
  std::vector<StrandSegmentHandle> seg_group;

  for (auto& pipe : pipes.PeekStrands()) {
    if (pipe.PeekStrandSegmentHandles().size() > index) {
      seg_group.push_back(pipe.PeekStrandSegmentHandles()[index]);
    } else {
      seg_group.push_back(-1);
    }
  }

  return seg_group;
}

void ObtainProfiles(const StrandModelStrandGroup& pipes, std::vector<StrandSegmentHandle> seg_group) {
  std::vector visited(seg_group.size(), false);

  std::vector<std::vector<StrandSegmentHandle>> profiles;

  for (auto& seg_handle : seg_group) {
    if (seg_handle == -1) {
      continue;
    }

    auto& seg = pipes.PeekStrandSegment(seg_handle);
    StrandHandle pipe_handle = seg.GetStrandHandle();

    if (seg.info.is_boundary && !visited[pipe_handle]) {
      // traverse boundary
      std::vector<StrandSegmentHandle> profile;
      auto handle = seg_handle;
      do {
        profile.push_back(handle);
        auto& seg = pipes.PeekStrandSegment(handle);
        StrandHandle pipe_handle = seg.GetStrandHandle();
        visited[pipe_handle] = true;

        // get next
        // seg.info.

      } while (handle != seg_handle);

      profiles.push_back(profile);
    }
  }
}

size_t GetNextOnBoundary(Graph& g, size_t cur, size_t prev, float& prev_angle) {
  // first rotate prevAngel by 180 degrees because we are looking from the other side of the edge now

  if (prev_angle < 0)  // TODO: should this be <=
  {
    prev_angle += glm::pi<float>();
  } else {
    prev_angle -= glm::pi<float>();
  }
  // if(DEBUG_OUTPUT) std::cout << "Flipped angle to " << prevAngle << std::endl;

  size_t next = -1;
  float next_angle = std::numeric_limits<float>::infinity();

  // The next angle must either be the smallest that is larger than the current one or if such an angle does not exist
  // the smallest overall
  for (size_t av : g.adjacentVertices(cur)) {
    if (av == prev) {
      continue;
    }

    glm::vec2 dir = g[av] - g[cur];
    float angle = atan2f(dir.y, dir.x);
    // if(DEBUG_OUTPUT) std::cout << " checking neighbor " << av << " with angle " << angle << std::endl;

    // remap such that all angles are larger than the previous
    float test_angle = angle;
    if (test_angle < prev_angle) {
      test_angle += 2 * glm::pi<float>();
    }

    if (test_angle < next_angle) {
      next_angle = test_angle;
      next = av;
    }
  }

  if (next == -1) {
    std::cout << "Warning: reached a leaf" << std::endl;

    return prev;
  }

  // confine to range
  if (next_angle > glm::pi<float>()) {
    next_angle -= 2 * glm::pi<float>();
  }

  // if(DEBUG_OUTPUT) std::cout << "Selected neighbor " << next << " with angle " << nextAngle << std::endl;

  // TODO: could we have a situation where we reach a leaf?
  prev_angle = next_angle;
  return next;
}

Slice ProfileToSlice(const StrandModel& strand_model, const std::vector<size_t>& profile, const PipeCluster& pipe_cluster,
                     const float t) {
  Slice slice;
  for (unsigned long long i : profile) {
    StrandHandle pipe_handle = pipe_cluster[i];
    slice.emplace_back(pipe_handle, GetPipePos(strand_model, pipe_handle, t));
  }

  return slice;
}

std::vector<size_t> ComputeComponent(Graph& strand_graph, glm::vec3 min, glm::vec3 max, const float max_dist,
                                     const PipeCluster& pipes_in_previous, size_t index, float t,
                                     std::vector<bool>& visited) {
  Grid2D grid(max_dist, min, max);

  // if(DEBUG_OUTPUT) std::cout << "inserting " << strandGraph.m_vertices.size() << " vertices into grid" << std::endl;
  for (size_t i = 0; i < strand_graph.m_vertices.size(); i++) {
    if (!visited[i]) {
      grid.insert(strand_graph, i);
    }
  }

  // if(DEBUG_OUTPUT) std::cout << "connecting neighbors" << std::endl;
  grid.connectNeighbors(strand_graph, max_dist);

  // if(DEBUG_OUTPUT) std::cout << "outputting graph" << std::endl;
  // outputGraph(strandGraph, "strandGraph_" + std::to_string(pipesInPrevious[index]) + "_" + std::to_string(t),
  // pipesInPrevious);

  // TODO:: maybe also use visited here
  // if(DEBUG_OUTPUT) std::cout << "collecting component" << std::endl;
  return CollectComponent(strand_graph, index);
}

std::pair<Graph, std::vector<size_t>> ComputeCluster(const StrandModel& strand_model, const PipeCluster& pipes_in_previous,
                                                     size_t index, std::vector<bool>& visited, float t, float max_dist,
                                                     size_t min_strand_count) {
  if (!IsValidPipeParam(strand_model, pipes_in_previous[index], t)) {
    return std::make_pair<>(Graph(), std::vector<size_t>());
  }
  // sweep over tree from root to leaves to reconstruct a skeleton with bark outlines
  // if(DEBUG_OUTPUT) std::cout << "obtaining profile of segGroup with " << pipesInPrevious.size() << " segments" <<
  // std::endl;
  glm::vec3 plane_pos = GetPipePos(strand_model, pipes_in_previous[index], t);
  glm::vec3 plane_norm = glm::normalize(GetPipeDir(strand_model, pipes_in_previous[index], t));

  // first we need to transform into a basis in the cross section plane. This will reduce the problem to 2D
  // To do so, first find suitable orthogonal vectors to the plane's normal vector
  size_t min_dim = 0;

  for (size_t i = 1; i < 3; i++) {
    if (plane_norm[i] < plane_norm[min_dim]) {
      min_dim = i;
    }
  }
  glm::vec3 e(0, 0, 0);
  e[min_dim] = 1.0f;

  glm::vec3 basis[3];

  basis[2] = plane_norm;
  basis[0] = glm::normalize(glm::cross(plane_norm, e));
  basis[1] = glm::cross(basis[0], basis[2]);

  // if(DEBUG_OUTPUT) std::cout << "new basis vectors: " << std::endl;

  for (size_t i = 0; i < 3; i++) {
    // if(DEBUG_OUTPUT) std::cout << "basis vector " << i << ": " << basis[i] << std::endl;
  }

  glm::mat3x3 basis_trans = {{basis[0][0], basis[1][0], basis[2][0]},
                            {basis[0][1], basis[1][1], basis[2][1]},
                            {basis[0][2], basis[1][2], basis[2][2]}};

  // TODO: could we do this more efficiently? E.g. the Intel Embree library provides efficent ray casts and allows for
  // ray bundling Also, this is an experimental extension to glm and might not be stable

  // now project all of them onto plane
  // if(DEBUG_OUTPUT) std::cout << "building strand graph" << std::endl;
  Graph strand_graph;
  glm::vec3 min(std::numeric_limits<float>::infinity());
  glm::vec3 max(-std::numeric_limits<float>::infinity());

  for (auto& pipe_handle : pipes_in_previous) {
    // if(DEBUG_OUTPUT) std::cout << "processing pipe with handle no. " << pipe_handle << std::endl;
    if (!IsValidPipeParam(strand_model, pipe_handle, t)) {
      // discard this pipe
      size_t v_index = strand_graph.addVertex();
      visited[v_index] = true;
      continue;
    }

    glm::vec3 seg_pos = GetPipePos(strand_model, pipe_handle, t);
    glm::vec3 seg_dir = glm::normalize(GetPipeDir(strand_model, pipe_handle, t));

    // There appears to be a bug (at least in debug compilations) where this value is not set if the result is close to
    // 0, so we need to set it.
    float param = 0.0f;

    glm::intersectRayPlane(seg_pos, seg_dir, plane_pos, plane_norm, param);
    // if(DEBUG_OUTPUT) std::cout << "line: " << segPos << " + t * " << segDir << " intersects plane " << planeNorm << "
    // * (p - " << planePos << ") = 0 at t = " << param << std::endl;
    //  store the intersection point in a graph
    size_t v_index = strand_graph.addVertex();
    glm::vec3 pos = basis_trans * (seg_pos + seg_dir * param);
    // if(DEBUG_OUTPUT) std::cout << "mapped point " << (segPos + segDir * param) << " to " << pos << std::endl;
    strand_graph[v_index] = pos;

    min = glm::min(min, pos);
    max = glm::max(max, pos);
  }
  // if(DEBUG_OUTPUT) std::cout << "built graph of size " << strandGraph.m_vertices.size() << std::endl;

  // TODO: we should check that maxDist is at least as large as the thickest strand
  // now cluster anything below maxDist
  std::vector<size_t> cluster;

  std::vector<size_t> candidates;

  for (size_t i = 0; i < strand_graph.m_vertices.size(); i++) {
    if (!visited[i]) {
      candidates.push_back(i);
    }
  }

  if (candidates.size() < min_strand_count) {
    // std::cout << "Cluster is too small, will be discarded" << std::endl;
  } else if (candidates.size() == 3) {
    if (DEBUG_OUTPUT)
      std::cout << "Defaulting to triangle" << std::endl;
    for (size_t i = 0; i < candidates.size(); i++) {
      cluster.push_back(candidates[i]);
      strand_graph.addEdge(candidates[i], candidates[(i + 1) % candidates.size()]);
    }
  } else {
    if (DEBUG_OUTPUT)
      std::cout << "computing cluster..." << std::endl;
    Delaunay(strand_graph, max_dist, candidates);

    cluster = CollectComponent(strand_graph, index);
  }
  // write cluster

  for (size_t index_in_component : cluster) {
    if (visited[index_in_component]) {
      std::cerr << "Error: index is already part of a different component" << std::endl;
    }

    visited[index_in_component] = true;
  }

  return std::make_pair<>(strand_graph, cluster);
}

std::pair<Slice, PipeCluster> ComputeSlice(const StrandModel& strand_model, const PipeCluster& pipes_in_previous,
                                           Graph& strand_graph, std::vector<size_t>& cluster, float t, float max_dist) {
  const StrandModelStrandGroup& pipes = strand_model.strand_model_skeleton.data.strand_group;

  PipeCluster pipes_in_component;

  for (const size_t index_in_component : cluster) {
    pipes_in_component.push_back(pipes_in_previous[index_in_component]);
  }

  if (DEBUG_OUTPUT)
    std::cout << "finding leftmost" << std::endl;
  // Now find an extreme point in the cluster. It must lie on the boundary. From here we can start traversing the
  // boundary
  size_t leftmost_index = 0;
  float leftmost_coord = std::numeric_limits<float>::infinity();

  for (size_t index : cluster) {
    if (strand_graph[index].x < leftmost_coord) {
      leftmost_coord = strand_graph[index].x;
      leftmost_index = index;
    }
  }

  // traverse the boundary using the angles to its neighbors
  if (DEBUG_OUTPUT)
    std::cout << "traversing boundary" << std::endl;
  // if(DEBUG_OUTPUT) std::cout << "starting at index: " << leftmostIndex << std::endl;
  std::vector<size_t> profile{leftmost_index};

  // going counterclockwise from here, the next point must be the one with the smallest angle
  size_t next = -1;
  float min_angle = glm::pi<float>();

  for (size_t av : strand_graph.adjacentVertices(leftmost_index)) {
    glm::vec2 dir = strand_graph[av] - strand_graph[leftmost_index];
    float angle = atan2f(dir.y, dir.x);
    // if(DEBUG_OUTPUT) std::cout << "checking neighbor " << *ai << " with angle " << angle << std::endl;
    if (angle < min_angle) {
      min_angle = angle;
      next = av;
    }
  }

  if (next == -1) {
    std::cerr << "Error: leftmost index has no neighbors" << std::endl;
  }

  // from now on we will have to find the closest neighboring edge in counter-clockwise order for each step until we
  // reach the first index again
  size_t prev = leftmost_index;
  size_t cur = next;
  float prev_angle = min_angle;

  size_t debug_counter = 0;
  while (next != leftmost_index && debug_counter < 1000) {
    // if(DEBUG_OUTPUT) std::cout << " Iteration no. " << debugCounter << std::endl;
    if (cur == -1) {
      std::cerr << "Error: cur is -1" << std::endl;
      break;
    }

    // if(DEBUG_OUTPUT) std::cout << "cur: " << cur << std::endl;
    profile.push_back(cur);

    next = GetNextOnBoundary(strand_graph, cur, prev, prev_angle);
    prev = cur;
    cur = next;

    debug_counter++;
  }

  Slice slice = ProfileToSlice(strand_model, profile, pipes_in_previous, t);

  return std::make_pair<>(slice, pipes_in_component);
}

std::pair<std::vector<Graph>, std::vector<std::vector<size_t>>> ComputeClusters(const StrandModel& strand_model,
                                                                                const PipeCluster& pipes_in_previous,
                                                                                float t, const float max_dist,
                                                                                size_t min_strand_count) {

  std::vector visited(pipes_in_previous.size(), false);
  std::vector<std::vector<size_t>> clusters;
  std::vector<Graph> graphs;

  std::vector<std::vector<size_t>> too_small_clusters;
  std::vector<Graph> too_small_graphs;

  for (std::size_t i = 0; i < pipes_in_previous.size(); i++) {
    if (visited[i]) {
      continue;
    }

    if (auto graph_and_cluster = ComputeCluster(strand_model, pipes_in_previous, i, visited, t, max_dist, min_strand_count); graph_and_cluster.second.size() >= min_strand_count) {
      graphs.push_back(graph_and_cluster.first);
      clusters.push_back(graph_and_cluster.second);
    } else if (graph_and_cluster.second.size() != 0) {
      too_small_graphs.push_back(graph_and_cluster.first);
      too_small_clusters.push_back(graph_and_cluster.second);
    }
  }

  return std::pair(graphs, clusters);
}

std::vector<std::pair<Slice, PipeCluster>> ComputeSlices(const StrandModel& strand_model,
                                                         const PipeCluster& pipes_in_previous, float t, float step_size,
                                                         float max_dist, size_t min_strand_count) {
  const auto& skeleton = strand_model.strand_model_skeleton;
  const auto& pipe_group = skeleton.data.strand_group;

  // first check if there are any pipes that might be needed for merging
  auto nh = GetNodeHandle(pipe_group, pipes_in_previous.front(), glm::floor(t - step_size));

  const auto& node = skeleton.PeekNode(nh);

  PipeCluster all_pipes_with_same_node;
  for (auto& kv : node.data.particle_map) {
    all_pipes_with_same_node.push_back(kv.first);
  }

  if (all_pipes_with_same_node.size() != pipes_in_previous.size()) {
    /*std::cout << "Potential for merge at t = " << t << ". Previous slice had " << pipesInPrevious.size()
            << " strand, but this one has potentially " << allPipesWithSameNode.size() << std::endl;*/
  }

  // compute clusters
  auto graphs_and_clusters = ComputeClusters(strand_model, pipes_in_previous, t, max_dist, min_strand_count);

  // then loop over the clusters to compute slices
  std::vector<std::pair<Slice, PipeCluster>> slices;

  for (std::size_t i = 0; i < graphs_and_clusters.first.size(); i++) {
    // if not visited, determine connected component around this
    // std::cout << "computing slice containing pipe no. " << i << " with handle " << pipesInPrevious[i] << " at t = "
    // << t << std::endl;
    auto slice =
        ComputeSlice(strand_model, pipes_in_previous, graphs_and_clusters.first[i], graphs_and_clusters.second[i], t, max_dist);
    slices.push_back(slice);
  }

  return slices;
}

void ForEachSegment(const StrandModelStrandGroup& pipes, const std::vector<StrandSegmentHandle>& seg_group,
                    std::function<void(const StrandSegment<StrandModelStrandSegmentData>&)> func) {
  for (auto& seg_handle : seg_group) {
    auto& seg = pipes.PeekStrandSegment(seg_handle);
    func(seg);
  }
}

void Connect(std::vector<std::pair<StrandHandle, glm::vec3>>& slice0, size_t i0, size_t j0,
             std::pair<size_t, size_t> offset0, std::vector<std::pair<StrandHandle, glm::vec3>>& slice1, size_t i1,
             size_t j1, std::pair<size_t, size_t> offset1, std::vector<Vertex>& vertices,
             std::vector<glm::vec2>& tex_coords, std::vector<std::pair<unsigned, unsigned>>& indices,
             const StrandModelMeshGeneratorSettings& settings) {
  if (DEBUG_OUTPUT)
    std::cout << "connecting " << i0 << ", " << j0 << " to " << i1 << ", " << j1 << std::endl;
  const size_t vert_between0 = (j0 + slice0.size() - i0) % slice0.size();
  const size_t vert_between1 = (j1 + slice1.size() - i1) % slice1.size();
  if (DEBUG_OUTPUT)
    std::cout << vert_between0 << " and " << vert_between1 << " steps, respectively " << std::endl;

  if (vert_between0 > slice0.size() / 2) {
    if (DEBUG_OUTPUT)
      std::cout << "Warning: too many steps for slice 0, should probably be swapped." << std::endl;
  }

  if (vert_between1 > slice1.size() / 2) {
    if (DEBUG_OUTPUT)
      std::cout << "Warning: too many steps for slice 1, should probably be swapped." << std::endl;
  }

  // merge the two
  if (DEBUG_OUTPUT)
    std::cout << "connecting slices with triangles" << std::endl;
  size_t k0 = 0;
  size_t k1 = 0;

  while (k0 < vert_between0 || k1 < vert_between1) {
    if (k0 / static_cast<double>(vert_between0) < k1 / static_cast<double>(vert_between1)) {
      size_t tex_index0 = offset0.second + (i0 + k0) % slice0.size() + 1;  // use end texture coordinate
      size_t tex_index1 = offset0.second + (i0 + k0) % slice0.size();
      size_t tex_index2 = offset1.second + (i1 + k1) % slice1.size();

      if ((i0 + k0) % slice0.size() + 1 == slice0.size())  // use end texture coordinate
      {
        tex_index2 = offset1.second + (i1 + k1 - 1) % slice1.size() + 1;
      }

      // make triangle consisting of k0, k0 + 1 and k1
      // assign new texture coordinates to the third corner if necessary
      glm::vec2 tex_coord2 = tex_coords[tex_index2];
      float avg_x = 0.5 * (tex_coords[tex_index0].x + tex_coords[tex_index1].x);

      float diff = avg_x - tex_coord2.x;
      float move = settings.u_multiplier * glm::round(diff / settings.u_multiplier);

      if (move != 0.0) {
        tex_index2 = tex_coords.size();
        tex_coord2.x += move;
        tex_coords.push_back(tex_coord2);
      }

      indices.emplace_back(std::make_pair<>(offset0.first + (i0 + k0 + 1) % slice0.size(), tex_index0));
      indices.emplace_back(std::make_pair<>(offset0.first + (i0 + k0) % slice0.size(), tex_index1));
      indices.emplace_back(std::make_pair<>(offset1.first + (i1 + k1) % slice1.size(), tex_index2));

      k0++;
    } else {
      size_t tex_index0 = offset1.second + (i1 + k1) % slice1.size();  // use end texture coordinate
      size_t tex_index1 = offset1.second + (i1 + k1) % slice1.size() + 1;
      size_t tex_index2 = offset0.second + (i0 + k0) % slice0.size();

      if ((i1 + k1) % slice1.size() + 1 == slice1.size())  // use end texture coordinate
      {
        tex_index2 = offset0.second + (i0 + k0 - 1) % slice0.size() + 1;
      }

      glm::vec2 tex_coord2 = tex_coords[tex_index2];
      float avg_x = 0.5 * (tex_coords[tex_index0].x + tex_coords[tex_index1].x);

      float diff = avg_x - tex_coord2.x;
      float move = settings.u_multiplier * glm::round(diff / settings.u_multiplier);

      if (move != 0.0) {
        tex_index2 = tex_coords.size();
        tex_coord2.x += move;
        tex_coords.push_back(tex_coord2);
      }

      indices.emplace_back(std::make_pair<>(offset1.first + (i1 + k1) % slice1.size(), tex_index0));
      indices.emplace_back(std::make_pair<>(offset1.first + (i1 + k1 + 1) % slice1.size(),
                                         tex_index1  // use end texture coordinate
                                         ));

      indices.emplace_back(std::make_pair<>(offset0.first + (i0 + k0) % slice0.size(), tex_index2));

      k1++;
    }
  }
}

size_t MidIndex(size_t a, size_t b, size_t size) {
  int mid = b - a;

  if (mid < 0) {
    mid += size;
  }

  return (a + mid / 2) % size;
}

/* We need to deal with inversions of the strand order on the outside somehow.
 * We have two different situations:
 *
 * Case 1: A split into two branches occurs. In this case we have a well defined linear order for each bottom section
 * that corresponds to only one branch. Here, we can easily identify inversions. (TODO: Unintended interleaving could
 * occur here, i.e. bottom vertices correspond to the branches A and B in order A B A B instead of A A B B)
 *
 * Case 2: No branching, the order here is cyclic. We can resolve this by cutting the cyclic order to obtain a linear
 * order. But we should choose a good cutting point to minimize the amount of inversions in order to preserve twisting.
 * A heuristic to achieve this is to define a family of permutations sigma_i which introduce an offset i. Then identify
 * the permutation that has the most fixed points.
 */
void CyclicOrderUntangle(std::vector<size_t>& permutation) {
  size_t n = permutation.size();

  std::vector<size_t> offset_histogram(n, 0);

  for (size_t i = 0; i < n; i++) {
    size_t offset = (permutation[i] - i + n) % n;

    offset_histogram[offset]++;
  }

  size_t index_with_most_fixed_points = 0;

  for (size_t i = 1; i < n; i++) {
    if (offset_histogram[index_with_most_fixed_points] > offset_histogram[i]) {
      index_with_most_fixed_points = i;
    }
  }
}

bool ConnectSlices(const StrandModelStrandGroup& pipes, Slice& bottom_slice, std::pair<unsigned, unsigned> bottom_offset,
                   std::vector<Slice>& top_slices, const std::vector<std::pair<unsigned, unsigned>>& top_offsets,
                   std::vector<Vertex>& vertices, std::vector<glm::vec2>& tex_coords,
                   std::vector<std::pair<unsigned, unsigned>>& indices, bool branch_connections,
                   const StrandModelMeshGeneratorSettings& settings) {
  // we want to track whether we actually produced any geometry
  const size_t size_before = indices.size();

  // compute (incomplete) permutation that turns 0 into 1

  // map of pipe handle index to top slice and index in top slice
  std::vector<std::pair<size_t, size_t>> top_pipe_handle_index_map(pipes.PeekStrands().size(), std::make_pair<>(-1, -1));

  // map of pipe handle index to index in bottom slice
  std::vector<size_t> bottom_pipe_handle_index_map(pipes.PeekStrands().size(), -1);

  for (size_t s = 0; s < top_slices.size(); s++) {
    for (size_t i = 0; i < top_slices[s].size(); i++) {
      // if(DEBUG_OUTPUT) std::cout << "inserting pipe handle no. " << topSlices[s][i].first << std::endl;
      top_pipe_handle_index_map[top_slices[s][i].first] = std::make_pair<>(s, i);
    }
  }

  for (size_t i = 0; i < bottom_slice.size(); i++) {
    bottom_pipe_handle_index_map[bottom_slice[i].first] = i;
  }

  // map index in bottom slice to top slice and index in top slice
  std::vector<std::pair<size_t, size_t>> bottom_permutation(bottom_slice.size(), std::make_pair<>(-1, -1));

  // map top slice and index in top slice to index in bottom slice
  std::vector<std::vector<size_t>> top_permutations(top_slices.size());

  for (std::size_t s = 0; s < top_slices.size(); s++) {
    top_permutations[s] = std::vector<size_t>(top_slices[s].size(), -1);

    for (size_t i = 0; i < top_permutations[s].size(); i++) {
      top_permutations[s][i] = bottom_pipe_handle_index_map[top_slices[s][i].first];
    }
  }

  if (DEBUG_OUTPUT)
    std::cout << "mapping back to permutation vector..." << std::endl;
  for (size_t i = 0; i < bottom_permutation.size(); i++) {
    bottom_permutation[i] = top_pipe_handle_index_map[bottom_slice[i].first];
  }

  // now we can more or less just connect them, except that we need to figure out the correct alignment and also get rid
  // of inversions
  // TODO: for now just assume that inversions do not happen
  size_t prev_i = -1;
  // if(DEBUG_OUTPUT) std::cout << "Finding first index..." << std::endl;
  for (size_t i = bottom_permutation.size(); i > 0; i--) {
    if (bottom_permutation[i - 1].second != -1) {
      prev_i = i - 1;
      break;
    }
  }
  if (DEBUG_OUTPUT)
    std::cout << "Found first index " << prev_i << std::endl;
  // need to find a start index where correspondence changes
  // TODO: only need to do this if there is a branching
  size_t start_index = prev_i;  // set prevI as default because this will work if there is no branching
  for (size_t i = 0; i < bottom_permutation.size(); i++) {
    if (bottom_permutation[i].second == -1) {
      continue;
    }

    if (bottom_permutation[prev_i].first != bottom_permutation[i].first) {
      start_index = i;
      break;
    }
    prev_i = i;
  }
  if (DEBUG_OUTPUT)
    std::cout << "Found start index " << start_index << std::endl;

  std::vector<size_t> indices_with_same_branch_correspondence;

  // shift this by one, otherwise the last section is not handled
  indices_with_same_branch_correspondence.push_back(start_index);
  prev_i = start_index;
  for (size_t counter = start_index + 1; counter != start_index + bottom_permutation.size() + 1; counter++) {
    size_t i = counter % bottom_permutation.size();

    if (bottom_permutation[i].second == -1) {
      // if(DEBUG_OUTPUT) std::cout << "No correspondence at index " << i << std::endl;
      continue;
    }

    if (bottom_permutation[prev_i].first == bottom_permutation[i].first) {
      indices_with_same_branch_correspondence.push_back(i);
    } else {
      size_t next_index = -1;

      for (size_t j = (bottom_permutation[prev_i].second + 1) % top_slices[bottom_permutation[prev_i].first].size();
           next_index == -1; j = (j + 1) % top_slices[bottom_permutation[prev_i].first].size()) {
        if (top_permutations[bottom_permutation[prev_i].first][j] != -1) {
          next_index = j;
        }
      }

      // now do the same for the other slice
      size_t prev_index = -1;

      for (size_t j = (bottom_permutation[i].second == 0 ? top_slices[bottom_permutation[i].first].size() - 1
                                                        : bottom_permutation[i].second - 1);
           prev_index == -1; j = (j == 0 ? top_slices[bottom_permutation[i].first].size() - 1 : j - 1)) {
        if (top_permutations[bottom_permutation[i].first][j] != -1) {
          prev_index = j;
        }
      }

      size_t bottom_mid = MidIndex(prev_i, i, bottom_slice.size());

      size_t next_mid =
          MidIndex(bottom_permutation[prev_i].second, next_index, top_slices[bottom_permutation[prev_i].first].size());
      size_t prev_mid = MidIndex(prev_index, bottom_permutation[i].second, top_slices[bottom_permutation[i].first].size());

      // TODO: we could do a very simple test if we selected the correct indices
      if (branch_connections) {
        Connect(bottom_slice, prev_i, bottom_mid, bottom_offset, top_slices[bottom_permutation[prev_i].first],
                bottom_permutation[prev_i].second, next_mid, top_offsets[bottom_permutation[prev_i].first], vertices,
                tex_coords, indices, settings);
        if (DEBUG_OUTPUT)
          std::cout << "Connected bottom indices " << prev_i << " to " << bottom_mid << " with "
                    << bottom_permutation[prev_i].second << " to " << next_mid << " of top profile no. "
                    << bottom_permutation[prev_i].first << std::endl;

        // connect mid indices with triangle
        // TODO: Think about texture coordinates
        indices.emplace_back(std::make_pair<>(bottom_offset.first + bottom_mid, bottom_offset.second + bottom_mid));
        indices.emplace_back(std::make_pair<>(top_offsets[bottom_permutation[prev_i].first].first + next_mid,
                                           top_offsets[bottom_permutation[prev_i].first].second + next_mid));
        indices.emplace_back(std::make_pair<>(top_offsets[bottom_permutation[i].first].first + prev_mid,
                                           top_offsets[bottom_permutation[i].first].second + prev_mid));

        // TODO: connecting with the same top slice looks better

        Connect(bottom_slice, bottom_mid, i, bottom_offset, top_slices[bottom_permutation[i].first], prev_mid,
                bottom_permutation[i].second, top_offsets[bottom_permutation[i].first], vertices, tex_coords, indices,
                settings);

        if (DEBUG_OUTPUT)
          std::cout << "Connected bottom indices " << bottom_mid << " to " << i << " with " << prev_mid << " to "
                    << bottom_permutation[i].second << " of top profile no. " << bottom_permutation[i].first << std::endl;
      }
    }

    // TODO: I'm pretty sure the topSlices.size() check is redundant
    if (((bottom_permutation[prev_i].first != bottom_permutation[i].first && top_slices.size() > 1) ||
         counter == start_index + bottom_permutation.size()) &&
        !indices_with_same_branch_correspondence.empty()) {
      std::vector<size_t> top_indices;

      size_t branch_index = bottom_permutation[indices_with_same_branch_correspondence.front()].first;

      for (unsigned long long j : indices_with_same_branch_correspondence) {
        top_indices.push_back(bottom_permutation[j].second);
      }

      // now check for errors and swap until there are no more errors
      // this is essentially bubble sort. We cannot use a conventional sorting algorithm here
      // because there is no global order - the comparison does not satisfy transitivity.
      // However, there is a local order and we hope that the elements are close enough to this that bubble sort works
      // as a heuristic
      bool found_error;
      size_t attempts = 0;

      do {
        attempts++;

        if (attempts >
            top_indices.size() + 10)  // add a constant because I'm not sure how small sizes of topIndices behave
        {
          std::cerr << "Error: Untangling meshing errors failed. This will most likely result in artifacts."
                    << std::endl;
          break;
        }

        found_error = false;
        for (size_t j = 1; j < top_indices.size(); j++) {
          size_t steps =
              (top_indices[j] + top_slices[branch_index].size() - top_indices[j - 1]) % top_slices[branch_index].size();

          if (top_indices[j] >= top_slices[branch_index].size() || top_indices[j - 1] >= top_slices[branch_index].size()) {
            std::cerr << "Error: Looks like an incorrect index ended up in the index list. Ignoring this pair."
                      << std::endl;
          } else if (steps > (top_slices[branch_index].size() + 1) / 2) {
            found_error = true;
            if (DEBUG_OUTPUT)
              std::cout << "found error, correcting by swapping " << top_indices[j - 1] << " and " << top_indices[j]
                        << "; steps: " << steps << "; element count: " << top_slices[branch_index].size() << std::endl;
            size_t tmp = top_indices[j];
            top_indices[j] = top_indices[j - 1];
            top_indices[j - 1] = tmp;
          }
        }

      } while (found_error);

      for (size_t j = 1; j < indices_with_same_branch_correspondence.size(); j++) {
        size_t prev_i = indices_with_same_branch_correspondence[j - 1];
        size_t i = indices_with_same_branch_correspondence[j];

        Connect(bottom_slice, prev_i, i, bottom_offset, top_slices[branch_index], top_indices[j - 1], top_indices[j],
                top_offsets[branch_index], vertices, tex_coords, indices, settings);
      }
    }

    // need to always do this
    if (bottom_permutation[prev_i].first != bottom_permutation[i].first) {
      indices_with_same_branch_correspondence.clear();
      indices_with_same_branch_correspondence.push_back(i);
    }

    prev_i = i;
  }

  return size_before != indices.size();
}

void CreateTwigTip(const StrandModel& strand_model, std::pair<Slice, PipeCluster>& prev_slice,
                   std::pair<unsigned, unsigned> prev_offset, float t, std::vector<Vertex>& vertices,
                   std::vector<glm::vec2>& tex_coords, std::vector<std::pair<unsigned, unsigned>>& indices) {
  // compute average positions of all slice points
  glm::vec3 pos(0, 0, 0);

  for (auto& el : prev_slice.second) {
    if (!IsValidPipeParam(strand_model, el, t)) {
      t -= 0.01;
    }

    pos += GetPipePos(strand_model, el, t);
  }

  Vertex v;
  v.position = pos / float(prev_slice.second.size());

  size_t tip_vert_index = vertices.size();
  vertices.push_back(v);

  size_t tip_tex_index = tex_coords.size();
  tex_coords.emplace_back(glm::vec2(t, 0.0));

  for (size_t i = 0; i < prev_slice.second.size(); i++) {
    indices.emplace_back(std::make_pair<>(prev_offset.first + i, prev_offset.second + i));
    indices.emplace_back(std::make_pair<>(tip_vert_index, tip_tex_index));
    indices.emplace_back(
        std::make_pair<>(prev_offset.first + (i + 1) % prev_slice.second.size(), prev_offset.second + i + 1));
  }
}

struct SlicingData {
  std::pair<Slice, PipeCluster> slice;
  size_t offset_vert;
  size_t offset_tex;
  float t;
  float accumulated_angle;
};

std::vector<SlicingData> Slicing(const StrandModel& strand_model, std::pair<Slice, PipeCluster>& prev_slice,
                               std::pair<unsigned, unsigned> prev_offset, float t, float step_size, float max_dist,
                               std::vector<Vertex>& vertices, std::vector<glm::vec2>& tex_coords,
                               std::vector<std::pair<unsigned, unsigned>>& indices,
                               const StrandModelMeshGeneratorSettings& settings, float accumulated_angle = 0.0f) {
  const auto& skeleton = strand_model.strand_model_skeleton;
  const auto& pipe_group = skeleton.data.strand_group;

  // prevent problems with floating point arithmetics
  if (t + 0.01 > glm::ceil(t)) {
    t = glm::ceil(t);
  }

  if (t > settings.max_param) {
    return {};
  }

  auto slices_and_clusters =
      ComputeSlices(strand_model, prev_slice.second, t, step_size, max_dist, settings.min_cell_count_for_major_branches);
  std::vector<Slice> top_slices;

  bool all_empty = true;

  for (auto& s : slices_and_clusters) {
    top_slices.push_back(s.first);

    if (!s.first.empty()) {
      all_empty = false;
    }
  }

  if (all_empty) {
    if (DEBUG_OUTPUT)
      std::cout << "=== Ending branch at t = " << t << " ===" << std::endl;

    // createTwigTip(strandModel, prevSlice, prevOffset, t - stepSize, vertices, indices);

    return {};
  }

  if (t <= 1) {
    std::cout << "accumulated angle at t = " << t << ": " << accumulated_angle << std::endl;
  }

  std::vector<std::pair<unsigned, unsigned>> offsets;

  // create vertices
  for (Slice& s : top_slices) {
    offsets.emplace_back(std::make_pair<>(vertices.size(), tex_coords.size()));

    bool is_first = true;

    for (auto& el : s) {
      Vertex v;
      v.position = el.second;

      glm::vec2 tex_coord;
      tex_coord.y = t * settings.v_multiplier;
      tex_coord.x = (GetPipePolar(strand_model, el.first, t) / (2 * glm::pi<float>()) + accumulated_angle / 360.0f) *
                   settings.u_multiplier;

      // add twisting to uv-Coordinates
      auto node_handle = GetNodeHandle(pipe_group, el.first, glm::floor(t + 1));
      const auto& node = skeleton.PeekNode(node_handle);

      float frac = fmod(t, 1.0);
      tex_coord.x += frac * node.data.twist_angle * settings.u_multiplier / 360.0f;

      // need to do proper wraparound
      if (!is_first && tex_coord.x + (settings.u_multiplier * 0.5) < tex_coords.back().x) {
        tex_coord.x += settings.u_multiplier;
      }

      v.tex_coord = tex_coord;  // legacy support
      vertices.push_back(v);
      tex_coords.push_back(tex_coord);

      is_first = false;
    }

    // texCoord for final vertex
    glm::vec2 tex_coord = tex_coords[offsets.back().second];

    if (tex_coord.x + (settings.u_multiplier * 0.5) < tex_coords.back().x) {
      tex_coord.x += settings.u_multiplier;
    }

    tex_coords.push_back(tex_coord);
  }

  bool connected = ConnectSlices(pipe_group, prev_slice.first, prev_offset, top_slices, offsets, vertices, tex_coords,
                                 indices, settings.branch_connections, settings);

  if (!connected) {
    std::cerr << "Error: did not connect the slices at t = " << t << " ---" << std::endl;
  }

  if (DEBUG_OUTPUT)
    std::cout << "--- Done with slice at t = " << t << " ---" << std::endl;
  // accumulate next slices
  t += step_size;
  std::vector<SlicingData> next_slices;

  for (size_t i = 0; i < slices_and_clusters.size(); i++) {
    if (!slices_and_clusters[i].first.empty()) {
      if (DEBUG_OUTPUT)
        std::cout << "___ Slice at t = " << t << " ___" << std::endl;

      float new_accumulated_angle = accumulated_angle;

      if (std::floor(t - step_size) < std::floor(t)) {
        // need to compute new accumulated angle
        auto node_handle = GetNodeHandle(pipe_group, slices_and_clusters[i].second[0], t);
        const auto& node = skeleton.PeekNode(node_handle);
        new_accumulated_angle += node.data.twist_angle;
      }

      next_slices.push_back(
          SlicingData{slices_and_clusters[i], offsets[i].first, offsets[i].second, t, new_accumulated_angle});
    }
  }

  return next_slices;
}

void SliceIteratively(const StrandModel& strand_model, std::vector<SlicingData>& start_slices, float step_size,
                      const float max_dist, std::vector<Vertex>& vertices, std::vector<glm::vec2>& tex_coords,
                      std::vector<std::pair<unsigned, unsigned>>& indices,
                      const StrandModelMeshGeneratorSettings& settings) {
  std::queue<SlicingData> queue;

  for (SlicingData& s : start_slices) {
    queue.push(s);
  }

  float accumulated_angle = 0.0f;

  while (!queue.empty()) {
    SlicingData cur = queue.front();
    queue.pop();

    if (DEBUG_OUTPUT)
      std::cout << "Took next slice with t = " << cur.t << " out of the queue" << std::endl;

    std::vector<SlicingData> slices =
        Slicing(strand_model, cur.slice, std::make_pair<>(cur.offset_vert, cur.offset_tex), cur.t, step_size, max_dist,
              vertices, tex_coords, indices, settings, cur.accumulated_angle);

    for (SlicingData& s : slices) {
      queue.push(s);
    }
  }
}

void StrandModelMeshGenerator::RecursiveSlicing(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                                std::vector<unsigned>& indices,
                                                const StrandModelMeshGeneratorSettings& settings) {
  // support mesh generation in framework
  std::vector<glm::vec2> dummy_tex_coords;
  std::vector<std::pair<unsigned, unsigned>> index_pairs;

  RecursiveSlicing(strand_model, vertices, dummy_tex_coords, index_pairs, settings);

  for (auto& pair : index_pairs) {
    indices.push_back(pair.first);
  }
}

void StrandModelMeshGenerator::RecursiveSlicing(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                                std::vector<glm::vec2>& tex_coords,
                                                std::vector<std::pair<unsigned, unsigned>>& indices,
                                                const StrandModelMeshGeneratorSettings& settings) {
  const auto& skeleton = strand_model.strand_model_skeleton;
  const auto& pipe_group = skeleton.data.strand_group;

  if (pipe_group.PeekStrands().size() == 0) {
    return;
  }

  if (DEBUG_OUTPUT)
    std::cout << "getting first seg group" << std::endl;
  std::vector<StrandSegmentHandle> seg_group0 = GetSegGroup(pipe_group, 0);

  if (DEBUG_OUTPUT)
    std::cout << "determining max thickness" << std::endl;
  float max_thickness = 0.0f;
  ForEachSegment(pipe_group, seg_group0, [&](const StrandSegment<StrandModelStrandSegmentData>& seg) {
    if (seg.info.thickness > max_thickness) {
      max_thickness = seg.info.thickness;
    }
  });

  float max_dist = 2 * max_thickness * sqrt(2) * 2.5f * settings.cluster_distance;
  // initial slice at root:
  std::vector<bool> visited(pipe_group.PeekStrands().size(), false);

  // prepare initial pipe cluster:
  PipeCluster pipe_cluster;

  for (size_t i = 0; i < pipe_group.PeekStrands().size(); i++) {
    pipe_cluster.push_back(pipe_group.PeekStrands()[i].GetHandle());
  }

  float step_size = 1.0f / settings.steps_per_segment;
  // float max = settings.max_param;

  // auto firstCluster = computeCluster(strandModel, pipeCluster, 0, visited, 0.0, maxDist);
  // auto firstSlice = computeSlice(strandModel, pipeCluster, firstCluster.first, firstCluster.second, 0.0, maxDist);

  auto first_slices = ComputeSlices(strand_model, pipe_cluster, 0, 0, max_dist, 3.0);  // TODO: magic number
  std::vector<SlicingData> start_slices;

  for (auto& slice : first_slices) {
    // create initial vertices
    size_t offset_vert = vertices.size();
    size_t offset_tex = tex_coords.size();

    bool is_first = true;

    for (auto& el : slice.first) {
      Vertex v;
      v.position = el.second;

      glm::vec2 tex_coord;
      tex_coord.y = 0.0;
      tex_coord.x = GetPipePolar(strand_model, el.first, 0.0) / (2 * glm::pi<float>()) * settings.u_multiplier;

      // add twisting to uv-Coordinates
      auto node_handle = GetNodeHandle(pipe_group, el.first, 0);
      const auto& node = skeleton.PeekNode(node_handle);

      tex_coord.x += node.data.twist_angle * settings.u_multiplier / 360.0f;
      v.tex_coord = tex_coord;  // legacy support
      vertices.push_back(v);

      if (!is_first && tex_coord.x + (settings.u_multiplier * 0.5) < tex_coords.back().x) {
        tex_coord.x += settings.u_multiplier;
      }

      tex_coords.push_back(tex_coord);

      is_first = false;
    }

    // need two different texture coordinates for the first and final vertex
    glm::vec2 tex_coord = tex_coords[offset_tex];

    if (tex_coord.x + (settings.u_multiplier * 0.5) < tex_coords.back().x) {
      tex_coord.x += settings.u_multiplier;
    }

    tex_coords.push_back(tex_coord);

    start_slices.push_back(SlicingData{slice, offset_vert, offset_tex, step_size, 0.0});
  }

  SliceIteratively(strand_model, start_slices, step_size, max_dist, vertices, tex_coords, indices, settings);
}

void StrandModelMeshGenerator::MarchingCube(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                            std::vector<unsigned>& indices,
                                            const StrandModelMeshGeneratorSettings& settings) {
  const auto& skeleton = strand_model.strand_model_skeleton;
  const auto& pipe_group = skeleton.data.strand_group;
  // first compute extreme points
  auto min = glm::vec3(std::numeric_limits<float>::infinity());
  auto max = glm::vec3(-std::numeric_limits<float>::infinity());
  bool need_triangulation = false;
  for (const auto& pipe_segment : pipe_group.PeekStrandSegments()) {
    const auto& node = skeleton.PeekNode(pipe_segment.data.node_handle);
    const auto& profile = node.data.profile;
    if (profile.PeekParticles().size() < settings.min_cell_count_for_major_branches)
      continue;
    need_triangulation = true;
    min = glm::min(pipe_segment.info.global_position, min);
    max = glm::max(pipe_segment.info.global_position, max);
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

    for (const auto& pipe_segment : pipe_group.PeekStrandSegments()) {
      const auto& node = skeleton.PeekNode(pipe_segment.data.node_handle);
      const auto& profile = node.data.profile;
      if (profile.PeekParticles().size() < settings.min_cell_count_for_major_branches)
        continue;

      // Get interpolated position on pipe segment. Example to get middle point here:
      const auto start_position = strand_model.InterpolateStrandSegmentPosition(pipe_segment.GetHandle(), 0.0f);
      const auto end_position = strand_model.InterpolateStrandSegmentPosition(pipe_segment.GetHandle(), 1.0f);
      const auto distance = glm::distance(start_position, end_position);
      const auto step_size = glm::max(1, static_cast<int>(distance / subdivision_length));

      const auto polar_x = profile.PeekParticle(pipe_segment.data.profile_particle_handle).GetInitialPolarPosition().y /
                          glm::radians(360.0f);
      for (int step = 0; step < step_size; step++) {
        const auto a = static_cast<float>(step) / step_size;
        const auto position = strand_model.InterpolateStrandSegmentPosition(pipe_segment.GetHandle(), a);

        octree.Occupy(position, [&](OctreeNode&) {
        });
      }
    }
    octree.TriangulateField(vertices, indices, settings.remove_duplicate);
  }
}

void StrandModelMeshGenerator::CylindricalMeshing(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                                  std::vector<unsigned>& indices,
                                                  const StrandModelMeshGeneratorSettings& settings) {
  const auto& skeleton = strand_model.strand_model_skeleton;
  const auto& sorted_internode_list = skeleton.PeekSortedNodeList();
  std::unordered_set<SkeletonNodeHandle> node_handles;
  for (const auto& node_handle : sorted_internode_list) {
    const auto& internode = skeleton.PeekNode(node_handle);
    if (const int particle_size = internode.data.profile.PeekParticles().size(); particle_size > settings.max_cell_count_for_minor_branches)
      continue;
    node_handles.insert(node_handle);
  }
  const auto current_vertices_size = vertices.size();
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  CylindricalMeshGenerator<StrandModelSkeletonData, StrandModelFlowData, StrandModelNodeData>::GeneratePartially(
      node_handles, skeleton, vertices, indices, eco_sys_lab_layer->m_meshGeneratorSettings,
      [&](glm::vec3&, const glm::vec3&, const float, const float) {
      },
      [&](glm::vec2& tex_coords, const float, const float) {
        tex_coords.x *= 2.0f;
        tex_coords.y *= 12.0f * settings.v_multiplier;
      });
  for (auto i = current_vertices_size; i < vertices.size(); i++) {
    vertices.at(i).color = glm::vec4(0, 1, 0, 1);
  }
}

void StrandModelMeshGenerator::MeshSmoothing(std::vector<Vertex>& vertices, std::vector<unsigned>& indices) {
  std::vector<std::vector<unsigned>> connectivity;
  connectivity.resize(vertices.size());
  for (int i = 0; i < indices.size() / 3; i++) {
    auto a = indices[3 * i];
    auto b = indices[3 * i + 1];
    auto c = indices[3 * i + 2];
    // a
    {
      bool found1 = false;
      bool found2 = false;
      for (const auto& index : connectivity.at(a)) {
        if (b == index)
          found1 = true;
        if (c == index)
          found2 = true;
      }
      if (!found1) {
        connectivity.at(a).emplace_back(b);
      }
      if (!found2) {
        connectivity.at(a).emplace_back(c);
      }
    }
    // b
    {
      bool found1 = false;
      bool found2 = false;
      for (const auto& index : connectivity.at(b)) {
        if (a == index)
          found1 = true;
        if (c == index)
          found2 = true;
      }
      if (!found1) {
        connectivity.at(b).emplace_back(a);
      }
      if (!found2) {
        connectivity.at(b).emplace_back(c);
      }
    }
    // c
    {
      bool found1 = false;
      bool found2 = false;
      for (const auto& index : connectivity.at(c)) {
        if (a == index)
          found1 = true;
        if (b == index)
          found2 = true;
      }
      if (!found1) {
        connectivity.at(c).emplace_back(a);
      }
      if (!found2) {
        connectivity.at(c).emplace_back(b);
      }
    }
  }
  std::vector<glm::vec3> new_positions;
  std::vector<glm::vec2> new_uvs;
  for (int i = 0; i < vertices.size(); i++) {
    auto position = glm::vec3(0.0f);
    auto uv = glm::vec2(0.f);
    for (const auto& index : connectivity.at(i)) {
      const auto& vertex = vertices.at(index);
      position += vertex.position;
      uv += vertex.tex_coord;
    }
    new_positions.push_back(position / static_cast<float>(connectivity.at(i).size()));
    new_uvs.push_back(uv / static_cast<float>(connectivity.at(i).size()));
  }
  for (int i = 0; i < vertices.size(); i++) {
    if (vertices[i].position.y > 0.001f)
      vertices[i].position = new_positions[i];
    else {
      vertices[i].position.x = new_positions[i].x;
      vertices[i].position.z = new_positions[i].z;
    }
    vertices[i].tex_coord = new_uvs[i];
  }
}

void StrandModelMeshGenerator::MeshSmoothing(std::vector<Vertex>& vertices,
                                             std::vector<std::pair<unsigned, unsigned>>& indices) {
  std::vector<std::vector<unsigned>> connectivity;
  connectivity.resize(vertices.size());
  for (int i = 0; i < indices.size() / 3; i++) {
    auto a = indices[3 * i];
    auto b = indices[3 * i + 1];
    auto c = indices[3 * i + 2];
    // a
    {
      bool found1 = false;
      bool found2 = false;
      for (const auto& index : connectivity.at(a.first)) {
        if (b.first == index)
          found1 = true;
        if (c.first == index)
          found2 = true;
      }
      if (!found1) {
        connectivity.at(a.first).emplace_back(b.first);
      }
      if (!found2) {
        connectivity.at(a.first).emplace_back(c.first);
      }
    }
    // b
    {
      bool found1 = false;
      bool found2 = false;
      for (const auto& index : connectivity.at(b.first)) {
        if (a.first == index)
          found1 = true;
        if (c.first == index)
          found2 = true;
      }
      if (!found1) {
        connectivity.at(b.first).emplace_back(a.first);
      }
      if (!found2) {
        connectivity.at(b.first).emplace_back(c.first);
      }
    }
    // c
    {
      bool found1 = false;
      bool found2 = false;
      for (const auto& index : connectivity.at(c.first)) {
        if (a.first == index)
          found1 = true;
        if (b.first == index)
          found2 = true;
      }
      if (!found1) {
        connectivity.at(c.first).emplace_back(a.first);
      }
      if (!found2) {
        connectivity.at(c.first).emplace_back(b.first);
      }
    }
  }
  std::vector<glm::vec3> new_positions;
  std::vector<glm::vec2> new_uvs;
  for (int i = 0; i < vertices.size(); i++) {
    auto position = glm::vec3(0.0f);
    auto uv = glm::vec2(0.f);
    for (const auto& index : connectivity.at(i)) {
      const auto& vertex = vertices.at(index);
      position += vertex.position;
      uv += vertex.tex_coord;
    }
    new_positions.push_back(position / static_cast<float>(connectivity.at(i).size()));
    new_uvs.push_back(uv / static_cast<float>(connectivity.at(i).size()));
  }
  for (int i = 0; i < vertices.size(); i++) {
    if (vertices[i].position.y > 0.001f)
      vertices[i].position = new_positions[i];
    else {
      vertices[i].position.x = new_positions[i].x;
      vertices[i].position.z = new_positions[i].z;
    }
    vertices[i].tex_coord = new_uvs[i];
  }
}

void StrandModelMeshGenerator::CalculateNormal(std::vector<Vertex>& vertices, const std::vector<unsigned>& indices) {
  auto normal_lists = std::vector<std::vector<glm::vec3>>();
  const auto size = vertices.size();
  for (auto i = 0; i < size; i++) {
    normal_lists.emplace_back();
  }
  for (int i = 0; i < indices.size() / 3; i++) {
    const auto i1 = indices.at(i * 3);
    const auto i2 = indices.at(i * 3 + 1);
    const auto i3 = indices.at(i * 3 + 2);
    auto v1 = vertices[i1].position;
    auto v2 = vertices[i2].position;
    auto v3 = vertices[i3].position;
    auto normal = glm::normalize(glm::cross(v1 - v2, v1 - v3));
    normal_lists[i1].push_back(normal);
    normal_lists[i2].push_back(normal);
    normal_lists[i3].push_back(normal);
  }
  for (auto i = 0; i < size; i++) {
    auto normal = glm::vec3(0.0f);
    for (const auto j : normal_lists[i]) {
      normal += j;
    }
    vertices[i].normal = glm::normalize(normal);
  }
}

void StrandModelMeshGenerator::CalculateNormal(std::vector<Vertex>& vertices,
                                               const std::vector<std::pair<unsigned, unsigned>>& indices) {
  auto normal_lists = std::vector<std::vector<glm::vec3>>();
  const auto size = vertices.size();
  for (auto i = 0; i < size; i++) {
    normal_lists.emplace_back();
  }
  for (int i = 0; i < indices.size() / 3; i++) {
    const auto i1 = indices.at(i * 3);
    const auto i2 = indices.at(i * 3 + 1);
    const auto i3 = indices.at(i * 3 + 2);
    auto v1 = vertices[i1.first].position;
    auto v2 = vertices[i2.first].position;
    auto v3 = vertices[i3.first].position;
    auto normal = glm::normalize(glm::cross(v1 - v2, v1 - v3));
    normal_lists[i1.first].push_back(normal);
    normal_lists[i2.first].push_back(normal);
    normal_lists[i3.first].push_back(normal);
  }
  for (auto i = 0; i < size; i++) {
    auto normal = glm::vec3(0.0f);
    for (const auto j : normal_lists[i]) {
      normal += j;
    }
    vertices[i].normal = glm::normalize(normal);
  }
}

glm::vec3 ProjectVec3(const glm::vec3& a, const glm::vec3& dir) {
  return glm::normalize(dir) * glm::dot(a, dir) / glm::length(dir);
}

void StrandModelMeshGenerator::CalculateUv(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                           const StrandModelMeshGeneratorSettings& settings) {
  if (settings.fast_uv) {
    const auto& sorted_node_list = strand_model.strand_model_skeleton.PeekSortedNodeList();

    Jobs::RunParallelFor(vertices.size(), [&](unsigned vertex_index) {
      auto& vertex = vertices.at(vertex_index);

      float min_distance = FLT_MAX;
      SkeletonNodeHandle closest_node_handle = -1;

      for (const auto& node_handle : sorted_node_list) {
        const auto& node = strand_model.strand_model_skeleton.PeekNode(node_handle);
        const auto& profile = node.data.profile;
        if (profile.PeekParticles().size() < settings.min_cell_count_for_major_branches)
          continue;
        const auto node_start = node.info.global_position;
        const auto node_end = node.info.GetGlobalEndPosition();
        const auto closest_point = glm::closestPointOnLine(vertex.position, node_start, node_end);
        if (glm::dot(node_end - node_start, closest_point - node_start) <= 0.f ||
            glm::dot(node_start - node_end, closest_point - node_end) <= 0.f)
          continue;
        if (const auto current_distance = glm::distance(closest_point, vertex.position) / node.info.thickness; current_distance < min_distance) {
          min_distance = current_distance;
          closest_node_handle = node_handle;
        }
      }
      if (closest_node_handle != -1) {
        const auto closest_node = strand_model.strand_model_skeleton.PeekNode(closest_node_handle);
        const float end_point_root_distance = closest_node.info.root_distance;
        const float start_point_root_distance = closest_node.info.root_distance - closest_node.info.length;
        const auto closest_point = glm::closestPointOnLine(vertex.position, closest_node.info.global_position,
                                                          closest_node.info.GetGlobalEndPosition());
        const float distance_to_start = glm::distance(closest_point, closest_node.info.global_position);
        const float a = closest_node.info.length == 0 ? 1.f : distance_to_start / closest_node.info.length;
        const float root_distance = glm::mix(start_point_root_distance, end_point_root_distance, a);
        vertex.tex_coord.y = root_distance * settings.root_distance_multiplier;
        const auto v = glm::normalize(vertex.position - closest_point);
        const auto up = closest_node.info.regulated_global_rotation * glm::vec3(0, 1, 0);
        const auto left = closest_node.info.regulated_global_rotation * glm::vec3(1, 0, 0);
        const auto proj_up = ProjectVec3(v, up);
        const auto proj_left = ProjectVec3(v, left);
        const glm::vec2 position = glm::vec2(glm::length(proj_left) * (glm::dot(proj_left, left) > 0.f ? 1.f : -1.f),
                                             glm::length(proj_up) * (glm::dot(proj_up, up) > 0.f ? 1.f : -1.f));
        const float acos_val = glm::acos(position.x / glm::length(position));
        vertex.tex_coord.x = acos_val / glm::pi<float>();
      } else {
        vertex.tex_coord = glm::vec2(0.0f);
      }
    });
  } else {
    const auto& strand_group = strand_model.strand_model_skeleton.data.strand_group;
    auto min = glm::vec3(FLT_MAX);
    auto max = glm::vec3(FLT_MIN);
    for (const auto& segment : strand_group.PeekStrandSegments()) {
      if (segment.IsRecycled())
        continue;
      const auto& node = strand_model.strand_model_skeleton.PeekNode(segment.data.node_handle);
      const auto& profile = node.data.profile;
      if (profile.PeekParticles().size() < settings.min_cell_count_for_major_branches)
        continue;
      const auto segment_start = strand_group.GetStrandSegmentStart(segment.GetHandle());
      const auto segment_end = segment.info.global_position;
      min = glm::min(segment_start, min);
      min = glm::min(segment_start, min);
      max = glm::max(segment_end, max);
      max = glm::max(segment_end, max);
    }
    min -= glm::vec3(0.1f);
    max += glm::vec3(0.1f);
    VoxelGrid<std::vector<StrandSegmentHandle>> boundary_segments;
    boundary_segments.Initialize(0.01f, min, max, {});

    for (const auto& segment : strand_group.PeekStrandSegments()) {
      if (segment.IsRecycled())
        continue;
      const auto& node = strand_model.strand_model_skeleton.PeekNode(segment.data.node_handle);
      const auto& profile = node.data.profile;
      if (profile.PeekParticles().size() < settings.min_cell_count_for_major_branches)
        continue;

      const auto segment_start = strand_group.GetStrandSegmentStart(segment.GetHandle());
      const auto segment_end = segment.info.global_position;
      boundary_segments.Ref((segment_start + segment_end) * 0.5f).emplace_back(segment.GetHandle());
    }

    Jobs::RunParallelFor(vertices.size(), [&](unsigned vertex_index) {
      auto& vertex = vertices.at(vertex_index);
      float min_distance = FLT_MAX;
      StrandSegmentHandle closest_segment_handle = -1;
      boundary_segments.ForEach(vertex.position, 0.05f, [&](const std::vector<StrandSegmentHandle>& segment_handles) {
        for (const auto& segment_handle : segment_handles) {
          const auto& segment = strand_group.PeekStrandSegment(segment_handle);
          const auto segment_start = strand_group.GetStrandSegmentStart(segment_handle);
          const auto segment_end = segment.info.global_position;
          const auto closest_point = glm::closestPointOnLine(vertex.position, segment_start, segment_end);
          if (glm::dot(segment_end - segment_start, closest_point - segment_start) <= 0.f ||
              glm::dot(segment_start - segment_end, closest_point - segment_end) <= 0.f)
            continue;
          if (const auto current_distance = glm::distance(segment_end, vertex.position); current_distance < min_distance) {
            min_distance = current_distance;
            closest_segment_handle = segment.GetHandle();
          }
        }
      });
      SkeletonNodeHandle closest_node_handle = -1;
      if (closest_segment_handle != -1) {
        const auto segment = strand_group.PeekStrandSegment(closest_segment_handle);
        closest_node_handle = segment.data.node_handle;
      }
      if (closest_node_handle != -1) {
        const auto closest_node = strand_model.strand_model_skeleton.PeekNode(closest_node_handle);
        const float end_point_root_distance = closest_node.info.root_distance;
        const float start_point_root_distance = closest_node.info.root_distance - closest_node.info.length;
        const auto closest_point = glm::closestPointOnLine(vertex.position, closest_node.info.global_position,
                                                          closest_node.info.GetGlobalEndPosition());
        const float distance_to_start = glm::distance(closest_point, closest_node.info.global_position);
        const float a = closest_node.info.length == 0 ? 1.f : distance_to_start / closest_node.info.length;
        const float root_distance = glm::mix(start_point_root_distance, end_point_root_distance, a);
        vertex.tex_coord.y = root_distance * settings.root_distance_multiplier;
        const auto v = glm::normalize(vertex.position - closest_point);
        const auto up = closest_node.info.regulated_global_rotation * glm::vec3(0, 1, 0);
        const auto left = closest_node.info.regulated_global_rotation * glm::vec3(1, 0, 0);
        const auto proj_up = ProjectVec3(v, up);
        const auto proj_left = ProjectVec3(v, left);
        const glm::vec2 position = glm::vec2(glm::length(proj_left) * (glm::dot(proj_left, left) > 0.f ? 1.f : -1.f),
                                             glm::length(proj_up) * (glm::dot(proj_up, up) > 0.f ? 1.f : -1.f));
        const float acos_val = glm::acos(position.x / glm::length(position));
        vertex.tex_coord.x = acos_val / glm::pi<float>();
      } else {
        vertex.tex_coord = glm::vec2(0.0f);
      }
    });
  }
}
