#include "IterativeSlicingMeshGenerator.hpp"
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/io.hpp>
#include "MeshGenUtils.hpp"

#define DEBUG_OUTPUT false

using namespace evo_engine;
namespace eco_sys_lab_plugin {
typedef std::vector<std::pair<StrandHandle, glm::vec3>> Slice;
typedef std::vector<StrandHandle> PipeCluster;

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

SkeletonNodeHandle GetNodeHandle(const StrandModelStrandGroup& pipe_group, const StrandHandle& pipe_handle, float t) {
  const size_t lookup_index = glm::round(t) < pipe_group.PeekStrand(pipe_handle).PeekStrandSegmentHandles().size()
                                  ? glm::round(t)
                                  : (pipe_group.PeekStrand(pipe_handle).PeekStrandSegmentHandles().size() - 1);
  auto& strand_segment_handle = pipe_group.PeekStrand(pipe_handle).PeekStrandSegmentHandles()[lookup_index];
  auto& strand_segment_data = pipe_group.PeekStrandSegmentData(strand_segment_handle);

  return strand_segment_data.node_handle;
}

bool IsValidPipeParam(const StrandModel& strand_model, const StrandHandle& pipe_handle, float t) {
  const auto& pipe = strand_model.strand_model_skeleton.data.strand_group.PeekStrand(pipe_handle);
  return pipe.PeekStrandSegmentHandles().size() > glm::floor(t);
}

glm::vec3 GetPipeDir(const StrandModel& strand_model, const StrandHandle& pipe_handle, float t) {
  const auto& pipe = strand_model.strand_model_skeleton.data.strand_group.PeekStrand(pipe_handle);
  const auto seg_handle = pipe.PeekStrandSegmentHandles()[t];
  return GetSegDir(strand_model, seg_handle, fmod(t, 1.0));
}

void SetStrandColor(StrandModel& strand_model, const StrandHandle& pipe_handle, float t, glm::vec4 color) {
}

const Particle2D<CellParticlePhysicsData>& GetEndParticle(const StrandModel& strand_model,
                                                          const StrandHandle& pipe_handle, size_t index) {
  if (!IsValidPipeParam(strand_model, pipe_handle, index)) {
    EVOENGINE_ERROR("Error: Strand " << pipe_handle << " does not exist at " << index);
  }

  const auto& skeleton = strand_model.strand_model_skeleton;
  const auto& pipe = skeleton.data.strand_group.PeekStrand(pipe_handle);
  StrandSegmentHandle seg_handle = pipe.PeekStrandSegmentHandles()[index];
  auto& pipe_segment_data = skeleton.data.strand_group.PeekStrandSegmentData(seg_handle);

  const auto& node = skeleton.PeekNode(pipe_segment_data.node_handle);
  const auto& start_profile = node.data.profile;
  // To access the user's defined constraints (attractors, etc.)
  const auto& profile_constraints = node.data.profile_constraints;

  // To access the position of the start of the pipe segment within a boundary:
  const auto parent_handle = node.GetParentHandle();
  const auto& end_particle = start_profile.PeekParticle(pipe_segment_data.profile_particle_handle);

  return end_particle;
}

const Particle2D<CellParticlePhysicsData>& GetStartParticle(const StrandModel& strand_model,
                                                            const StrandHandle& pipe_handle, size_t index) {
  if (!IsValidPipeParam(strand_model, pipe_handle, index)) {
    EVOENGINE_ERROR("Strand " << pipe_handle << " does not exist at " << index);
  }

  const auto& skeleton = strand_model.strand_model_skeleton;
  const auto& pipe = skeleton.data.strand_group.PeekStrand(pipe_handle);
  const auto seg_handle = pipe.PeekStrandSegmentHandles()[index];
  auto& strand_segment_data = skeleton.data.strand_group.PeekStrandSegmentData(seg_handle);

  const auto& node = skeleton.PeekNode(strand_segment_data.node_handle);
  const auto& start_profile = node.data.profile;
  // To access the user's defined constraints (attractors, etc.)
  const auto& profile_constraints = node.data.profile_constraints;
  // To access the position of the start of the pipe segment within a boundary:
  const auto& start_particle = start_profile.PeekParticle(strand_segment_data.profile_particle_handle);
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

std::vector<size_t> CollectComponent(const Graph& g, size_t start_index) {
  // just do a dfs to collect everything
  return Dfs(g, start_index);
}

void ObtainProfiles(const StrandModelStrandGroup& pipes, std::vector<StrandSegmentHandle> seg_group) {
  std::vector visited(seg_group.size(), false);

  std::vector<std::vector<StrandSegmentHandle>> profiles;

  for (const auto& seg_handle : seg_group) {
    if (seg_handle == -1) {
      continue;
    }

    auto& seg = pipes.PeekStrandSegment(seg_handle);
    auto& segment_data = pipes.PeekStrandSegmentData(seg_handle);

    if (segment_data.is_boundary && !visited[seg.GetStrandHandle()]) {
      // traverse boundary
      std::vector<StrandSegmentHandle> profile;
      auto handle = seg_handle;
      do {
        profile.push_back(handle);
        visited[pipes.PeekStrandSegment(handle).GetStrandHandle()] = true;

      } while (handle != seg_handle);

      profiles.push_back(profile);
    }
  }
}

Slice ProfileToSlice(const StrandModel& strand_model, const std::vector<size_t>& profile,
                     const PipeCluster& pipe_cluster, const float t) {
  Slice slice;
  for (unsigned long long i : profile) {
    StrandHandle pipe_handle = pipe_cluster[i];
    slice.emplace_back(pipe_handle, GetPipePos(strand_model, pipe_handle, t));
  }

  return slice;
}

std::pair<Graph, std::vector<size_t>> ComputeCluster(const StrandModel& strand_model,
                                                     const PipeCluster& pipes_in_previous, size_t index,
                                                     std::vector<bool>& visited, float t, float max_dist,
                                                     size_t min_strand_count) {
  if (!IsValidPipeParam(strand_model, pipes_in_previous[index], t)) {
    return std::make_pair<>(Graph(), std::vector<size_t>());
  }
  // sweep over tree from root to leaves to reconstruct a skeleton with bark outlines
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

  glm::mat3x3 basis_trans = {{basis[0][0], basis[1][0], basis[2][0]},
                             {basis[0][1], basis[1][1], basis[2][1]},
                             {basis[0][2], basis[1][2], basis[2][2]}};

  // TODO: could we do this more efficiently? E.g. the Intel Embree library provides efficent ray casts and allows for
  // ray bundling Also, this is an experimental extension to glm and might not be stable

  // now project all of them onto plane
  Graph strand_graph;
  glm::vec3 min(std::numeric_limits<float>::infinity());
  glm::vec3 max(-std::numeric_limits<float>::infinity());

  for (auto& pipe_handle : pipes_in_previous) {
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
    //  store the intersection point in a graph
    size_t v_index = strand_graph.addVertex();
    glm::vec3 pos = basis_trans * (seg_pos + seg_dir * param);
    strand_graph[v_index] = pos;

    min = glm::min(min, pos);
    max = glm::max(max, pos);
  }

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
    EVOENGINE_WARNING("Cluster at t = " << t << " is too small, will be discarded. Minimum: " << min_strand_count
                                        << " Actual: " << candidates.size());
  } else if (candidates.size() == 3) {
    if (DEBUG_OUTPUT)
      EVOENGINE_LOG("Defaulting to triangle");
    for (size_t i = 0; i < candidates.size(); i++) {
      cluster.push_back(candidates[i]);
      strand_graph.addEdge(candidates[i], candidates[(i + 1) % candidates.size()]);
    }
  } else {
    if (DEBUG_OUTPUT)
      EVOENGINE_LOG("computing cluster...");
    Delaunay(strand_graph, max_dist, candidates);

    cluster = CollectComponent(strand_graph, index);
  }
  // write cluster

  for (size_t index_in_component : cluster) {
    if (visited[index_in_component]) {
      EVOENGINE_ERROR("index is already part of a different component.");
    }

    visited[index_in_component] = true;
  }

  return std::make_pair<>(strand_graph, cluster);
}

std::pair<Slice, PipeCluster> ComputeSlice(const StrandModel& strand_model, const PipeCluster& pipes_in_previous,
                                           Graph& strand_graph, std::vector<size_t>& cluster, float t, float max_dist) {
  // Find an extreme point in the cluster. It must lie on the boundary. From here we can start traversing the
  // boundary
  size_t leftmost_index = FindLeftmostIndex(strand_graph, cluster);
  auto boundary = TraceBoundary(strand_graph, leftmost_index);
  Slice slice = ProfileToSlice(strand_model, boundary, pipes_in_previous, t);

  PipeCluster pipes_in_component;

  for (const size_t index_in_component : cluster) {
    pipes_in_component.push_back(pipes_in_previous[index_in_component]);
  }
  return std::make_pair<>(slice, pipes_in_component);
}

std::pair<std::vector<Graph>, std::vector<std::vector<size_t>>> ComputeClusters(const StrandModel& strand_model,
                                                                                const PipeCluster& pipes_in_previous,
                                                                                float t, const float max_dist,
                                                                                size_t min_strand_count) {
  std::vector<bool> visited(pipes_in_previous.size(), false);
  std::vector<std::vector<size_t>> clusters;
  std::vector<Graph> graphs;

  std::vector<std::vector<size_t>> too_small_clusters;
  std::vector<Graph> too_small_graphs;

  for (std::size_t i = 0; i < pipes_in_previous.size(); i++) {
    if (visited[i]) {
      continue;
    }

    auto graph_and_cluster = ComputeCluster(strand_model, pipes_in_previous, i, visited, t, max_dist, min_strand_count);

    if (graph_and_cluster.second.size() >= min_strand_count) {
      graphs.push_back(graph_and_cluster.first);
      clusters.push_back(graph_and_cluster.second);
    } else if (graph_and_cluster.second.size() != 0) {
      too_small_graphs.push_back(graph_and_cluster.first);
      too_small_clusters.push_back(graph_and_cluster.second);
    }
  }

  return std::pair(graphs, clusters);
}

std::vector<std::tuple<Slice, PipeCluster, std::vector<size_t>>> ComputeSlices(
    const StrandModel& strand_model, const PipeCluster& pipes_in_previous, float t, float step_size, float max_dist,
    size_t min_strand_count, std::vector<size_t>& pipe_to_slice_index_map, size_t prev_slice_index) {
  const auto& skeleton = strand_model.strand_model_skeleton;
  const auto& pipe_group = skeleton.data.strand_group;

  // first check if there are any pipes that might be needed for merging
  PipeCluster all_pipes_with_same_node;
  // first check if there are any pipes that might be needed for merging
  if (t > 0.0) {
    // need to find all nodes that are asociated with the slice
    std::set<SkeletonNodeHandle> node_handles;

    for (auto& pipe_handle : pipes_in_previous) {
      node_handles.insert(GetNodeHandle(pipe_group, pipe_handle, glm::floor(t - step_size)));
    }

    for (auto nh : node_handles) {
      const auto* node = &skeleton.PeekNode(nh);

      // need to go further down in some cases
      size_t subtract = 1;
      while (node->data.profile.PeekParticles().size() < pipes_in_previous.size()) {
        nh = GetNodeHandle(pipe_group, pipes_in_previous.front(), glm::floor(t - step_size) - subtract);
        node = &skeleton.PeekNode(nh);
        subtract++;
      }

      for (auto& particle : node->data.profile.PeekParticles()) {
        all_pipes_with_same_node.push_back(particle.strand_handle);
      }
    }
  } else {
    all_pipes_with_same_node = pipes_in_previous;
  }

  // compute clusters
  std::pair<std::vector<Graph>, std::vector<std::vector<size_t>>> graphs_and_clusters =
      ComputeClusters(strand_model, pipes_in_previous, t, max_dist, min_strand_count);

  std::pair<std::vector<Graph>, std::vector<std::vector<size_t>>> filtered_graphs_and_clusters;

  std::vector<std::vector<size_t>> merge_list;

  // we need to filter now for clusters that are actually connected to pipes of the previous
  for (std::size_t i = 0; i < graphs_and_clusters.first.size(); i++) {
    auto& cluster = graphs_and_clusters.second[i];

    std::set<size_t> merge_indices;

    for (auto& index_in_component : cluster) {
      StrandHandle strand_handle = all_pipes_with_same_node[index_in_component];

      if (pipe_to_slice_index_map[strand_handle] == -1) {
        // this seems to be no issue after all
        EVOENGINE_WARNING("pipe_to_slice_index_map contains -1");
        continue;
      }

      merge_indices.insert(pipe_to_slice_index_map[strand_handle]);
    }

    if (*merge_indices.begin() < prev_slice_index)  // use minimum index to make mergers unique to avoid duplicates
    {
      // TODO: should anything be done here?
    } else if (merge_indices.find(prev_slice_index) != merge_indices.end()) {
      filtered_graphs_and_clusters.first.push_back(graphs_and_clusters.first[i]);
      filtered_graphs_and_clusters.second.push_back(graphs_and_clusters.second[i]);

      std::vector<size_t> merge_indices_vec;
      std::copy(merge_indices.begin(), merge_indices.end(), std::back_inserter(merge_indices_vec));

      if (merge_indices_vec.size() > 1) {
        std::cout << "Need to merge the following slices:\n";

        for (size_t index : merge_indices_vec) {
          std::cout << index << ", ";
        }
        std::cout << std::endl;
      }

      merge_list.push_back(merge_indices_vec);
    } else {
      // TODO: should anything be done here?
    }
  }

  // then loop over the clusters to compute slices
  std::vector<std::tuple<Slice, PipeCluster, std::vector<size_t>>> slices;

  for (std::size_t i = 0; i < filtered_graphs_and_clusters.first.size(); i++) {
    auto slice = ComputeSlice(strand_model, pipes_in_previous, graphs_and_clusters.first[i],
                              graphs_and_clusters.second[i], t, max_dist);
    slices.push_back(std::tuple<Slice, PipeCluster, std::vector<size_t>>(slice.first, slice.second, merge_list[i]));
  }

  return slices;
}

void Connect(std::vector<std::pair<StrandHandle, glm::vec3>>& slice0, size_t i0, size_t j0,
             std::pair<size_t, size_t> offset0, std::vector<std::pair<StrandHandle, glm::vec3>>& slice1, size_t i1,
             size_t j1, std::pair<size_t, size_t> offset1, std::vector<Vertex>& vertices,
             std::vector<glm::vec2>& tex_coords, std::vector<std::pair<unsigned, unsigned>>& indices,
             const StrandModelMeshGeneratorSettings& settings, bool invert_triangle) {
  if (DEBUG_OUTPUT)
    EVOENGINE_LOG("connecting " << i0 << ", " << j0 << " to " << i1 << ", " << j1);
  const size_t vert_between0 = (j0 + slice0.size() - i0) % slice0.size();
  const size_t vert_between1 = (j1 + slice1.size() - i1) % slice1.size();
  if (DEBUG_OUTPUT)
    EVOENGINE_LOG(vert_between0 << " and " << vert_between1 << " steps, respectively ");

  if (vert_between0 > slice0.size() / 2) {
    if (DEBUG_OUTPUT)
      EVOENGINE_LOG("Warning: too many steps for slice 0, should probably be swapped.");
  }

  if (vert_between1 > slice1.size() / 2) {
    if (DEBUG_OUTPUT)
      EVOENGINE_LOG("Warning: too many steps for slice 1, should probably be swapped.");
  }

  // merge the two
  if (DEBUG_OUTPUT)
    EVOENGINE_LOG("connecting slices with triangles");
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

      if (!invert_triangle) {
        indices.emplace_back(std::make_pair<>(offset0.first + (i0 + k0 + 1) % slice0.size(), tex_index0));
        indices.emplace_back(std::make_pair<>(offset0.first + (i0 + k0) % slice0.size(), tex_index1));
        indices.emplace_back(std::make_pair<>(offset1.first + (i1 + k1) % slice1.size(), tex_index2));
      } else {
        indices.emplace_back(std::make_pair<>(offset0.first + (i0 + k0 + 1) % slice0.size(), tex_index0));
        indices.emplace_back(std::make_pair<>(offset1.first + (i1 + k1) % slice1.size(), tex_index2));
        indices.emplace_back(std::make_pair<>(offset0.first + (i0 + k0) % slice0.size(), tex_index1));
      }

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

      if (!invert_triangle) {
        indices.emplace_back(std::make_pair<>(offset1.first + (i1 + k1) % slice1.size(), tex_index0));
        indices.emplace_back(std::make_pair<>(offset1.first + (i1 + k1 + 1) % slice1.size(),
                                              tex_index1  // use end texture coordinate
                                              ));

        indices.emplace_back(std::make_pair<>(offset0.first + (i0 + k0) % slice0.size(), tex_index2));
      } else {
        indices.emplace_back(std::make_pair<>(offset1.first + (i1 + k1) % slice1.size(), tex_index0));
        indices.emplace_back(std::make_pair<>(offset0.first + (i0 + k0) % slice0.size(), tex_index2));
        indices.emplace_back(std::make_pair<>(offset1.first + (i1 + k1 + 1) % slice1.size(),
                                              tex_index1  // use end texture coordinate
                                              ));
      }

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

bool ConnectSlices(const StrandModelStrandGroup& pipes, Slice& bottom_slice,
                   std::pair<unsigned, unsigned> bottom_offset, std::vector<Slice>& top_slices,
                   const std::vector<std::pair<unsigned, unsigned>>& top_offsets, std::vector<Vertex>& vertices,
                   std::vector<glm::vec2>& tex_coords, std::vector<std::pair<unsigned, unsigned>>& indices,
                   bool branch_connections, const StrandModelMeshGeneratorSettings& settings,
                   bool invert_triangles = false) {
  // we want to track whether we actually produced any geometry
  const size_t size_before = indices.size();

  // compute (incomplete) permutation that turns 0 into 1

  // map of pipe handle index to top slice and index in top slice
  std::vector<std::pair<size_t, size_t>> top_pipe_handle_index_map(pipes.PeekStrands().size(),
                                                                   std::make_pair<>(-1, -1));

  // map of pipe handle index to index in bottom slice
  std::vector<size_t> bottom_pipe_handle_index_map(pipes.PeekStrands().size(), -1);

  for (size_t s = 0; s < top_slices.size(); s++) {
    for (size_t i = 0; i < top_slices[s].size(); i++) {
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
    EVOENGINE_LOG("mapping back to permutation vector...");
  for (size_t i = 0; i < bottom_permutation.size(); i++) {
    bottom_permutation[i] = top_pipe_handle_index_map[bottom_slice[i].first];
  }

  // now we can more or less just connect them, except that we need to figure out the correct alignment and also get rid
  // of inversions
  // TODO: for now just assume that inversions do not happen
  size_t prev_i = -1;
  for (size_t i = bottom_permutation.size(); i > 0; i--) {
    if (bottom_permutation[i - 1].second != -1) {
      prev_i = i - 1;
      break;
    }
  }
  if (DEBUG_OUTPUT)
    EVOENGINE_LOG("Found first index " << prev_i);
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
    EVOENGINE_LOG("Found start index " << start_index);

  std::vector<size_t> indices_with_same_branch_correspondence;

  // shift this by one, otherwise the last section is not handled
  indices_with_same_branch_correspondence.push_back(start_index);
  prev_i = start_index;
  for (size_t counter = start_index + 1; counter != start_index + bottom_permutation.size() + 1; counter++) {
    size_t i = counter % bottom_permutation.size();

    if (bottom_permutation[i].second == -1) {
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
      size_t prev_mid =
          MidIndex(prev_index, bottom_permutation[i].second, top_slices[bottom_permutation[i].first].size());

      // TODO: we could do a very simple test if we selected the correct indices
      if (branch_connections) {
        Connect(bottom_slice, prev_i, bottom_mid, bottom_offset, top_slices[bottom_permutation[prev_i].first],
                bottom_permutation[prev_i].second, next_mid, top_offsets[bottom_permutation[prev_i].first], vertices,
                tex_coords, indices, settings, invert_triangles);
        if (DEBUG_OUTPUT)
          EVOENGINE_LOG("Connected bottom indices " << prev_i << " to " << bottom_mid << " with "
                                                    << bottom_permutation[prev_i].second << " to " << next_mid
                                                    << " of top boundary no. " << bottom_permutation[prev_i].first);

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
                settings, invert_triangles);

        if (DEBUG_OUTPUT)
          EVOENGINE_LOG("Connected bottom indices " << bottom_mid << " to " << i << " with " << prev_mid << " to "
                                                    << bottom_permutation[i].second << " of top boundary no. "
                                                    << bottom_permutation[i].first);
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

      // TODO: cocktail shaker sort might be better here
      do {
        attempts++;

        if (attempts >
            top_indices.size() + 10)  // add a constant because I'm not sure how small sizes of topIndices behave
        {
          EVOENGINE_ERROR("Untangling meshing errors failed. This will most likely result in artifacts.");
          break;
        }

        found_error = false;
        for (size_t j = 1; j < top_indices.size(); j++) {
          size_t steps =
              (top_indices[j] + top_slices[branch_index].size() - top_indices[j - 1]) % top_slices[branch_index].size();

          if (top_indices[j] >= top_slices[branch_index].size() ||
              top_indices[j - 1] >= top_slices[branch_index].size()) {
            EVOENGINE_ERROR("Looks like an incorrect index ended up in the index list. Ignoring this pair.");
          } else if (steps > (top_slices[branch_index].size() + 1) / 2) {
            found_error = true;
            if (DEBUG_OUTPUT)
              EVOENGINE_LOG("found error, correcting by swapping "
                            << top_indices[j - 1] << " and " << top_indices[j] << "; steps: " << steps
                            << "; element count: " << top_slices[branch_index].size());
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
                top_offsets[branch_index], vertices, tex_coords, indices, settings, invert_triangles);
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
  size_t index;
  float t;
  float accumulated_angle;
};

std::vector<SlicingData> Slicing(const StrandModel& strand_model, std::vector<SlicingData>& prev_slices,
                                 float step_size, float max_dist, std::vector<Vertex>& vertices,
                                 std::vector<glm::vec2>& tex_coords,
                                 std::vector<std::pair<unsigned, unsigned>>& indices,
                                 const StrandModelMeshGeneratorSettings& settings,
                                 std::vector<size_t>& pipe_to_slice_index_map, size_t prev_slice_index) {
  auto& prev_slice = prev_slices[prev_slice_index].slice;
  float t = prev_slices[prev_slice_index].t;
  std::pair<size_t, size_t> prev_offset(prev_slices[prev_slice_index].offset_vert,
                                        prev_slices[prev_slice_index].offset_tex);
  float accumulated_angle = prev_slices[prev_slice_index].accumulated_angle;
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
      ComputeSlices(strand_model, prev_slice.second, t, step_size, max_dist, settings.min_cell_count_for_major_branches,
                    pipe_to_slice_index_map, prev_slice_index);
  std::vector<Slice> top_slices;

  bool all_empty = true;

  for (auto& s : slices_and_clusters) {
    top_slices.push_back(std::get<0>(s));

    if (std::get<0>(s).size() != 0) {
      all_empty = false;
    }
  }

  if (all_empty) {
    if (DEBUG_OUTPUT)
      EVOENGINE_LOG("=== Ending branch at t = " << t << " ===");

    // createTwigTip(strandModel, prevSlice, prevOffset, t - stepSize, vertices, indices);

    return {};
  }

  std::vector<std::pair<unsigned, unsigned>> offsets;

  // create vertices
  for (auto& scm : slices_and_clusters) {
    Slice& s = std::get<0>(scm);
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

  bool connected = false;

  if (std::get<2>(slices_and_clusters.front()).size() > 1) {
    // need to merge here
    if (slices_and_clusters.size() > 1) {
      std::cout << "Warning: Multiple slices on both sides, not supported!" << std::endl;

      // let's just connect all of them. This will create intersecting meshes, but should work
      std::vector<Slice> bottom_slices;
      std::vector<std::pair<unsigned, unsigned>> bottom_offsets;

      for (size_t index : std::get<2>(slices_and_clusters.front())) {
        bottom_slices.push_back(prev_slices[index].slice.first);
        bottom_offsets.push_back(std::make_pair<>(prev_slices[index].offset_vert, prev_slices[index].offset_tex));
      }

      for (size_t i = 0; i < bottom_slices.size(); i++) {
        connected = ConnectSlices(pipe_group, bottom_slices[i], bottom_offsets[i], top_slices, offsets, vertices,
                                  tex_coords, indices, settings.branch_connections, settings);
      }

    } else {
      std::cout << "Merging branches at t = " << t << std::endl;
      std::vector<Slice> bottom_slices;
      std::vector<std::pair<unsigned, unsigned>> bottom_offsets;

      for (size_t index : std::get<2>(slices_and_clusters.front())) {
        bottom_slices.push_back(prev_slices[index].slice.first);
        bottom_offsets.push_back(std::make_pair<>(prev_slices[index].offset_vert, prev_slices[index].offset_tex));
      }

      connected = ConnectSlices(pipe_group, top_slices.front(), offsets.front(), bottom_slices, bottom_offsets,
                                vertices, tex_coords, indices, settings.branch_connections, settings, true);
    }
  } else {
    connected = ConnectSlices(pipe_group, prev_slice.first, prev_offset, top_slices, offsets, vertices, tex_coords,
                              indices, settings.branch_connections, settings);
  }

  if (!connected) {
    EVOENGINE_ERROR("Error: did not connect the slices at t = " << t << " ---");
  }

  if (DEBUG_OUTPUT)
    EVOENGINE_LOG("--- Done with slice at t = " << t << " ---");
  // accumulate next slices
  t += step_size;  // perhaps we should move this somewhere to the top
  std::vector<SlicingData> next_slices;

  for (size_t i = 0; i < slices_and_clusters.size(); i++) {
    if (std::get<0>(slices_and_clusters[i]).size() != 0) {
      if (DEBUG_OUTPUT)
        EVOENGINE_LOG("___ Slice at t = " << t << " ___");

      float new_accumulated_angle = accumulated_angle;

      if (std::floor(t - step_size) < std::floor(t)) {
        // need to compute new accumulated angle
        auto node_handle = GetNodeHandle(pipe_group, std::get<1>(slices_and_clusters[i])[0], t);
        const auto& node = skeleton.PeekNode(node_handle);
        new_accumulated_angle += node.data.twist_angle;
      }

      next_slices.push_back(
          SlicingData{std::make_pair<>(std::get<0>(slices_and_clusters[i]), std::get<1>(slices_and_clusters[i])),
                      offsets[i].first, offsets[i].second, 0, t, new_accumulated_angle});
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
  float t = 0.0;
  size_t index = 0;

  std::vector<size_t> pipe_to_slice_index_map(strand_model.strand_model_skeleton.data.strand_group.PeekStrands().size(),
                                              -1);
  std::vector<SlicingData> prev_slices;

  while (!queue.empty()) {
    SlicingData cur = queue.front();

    if (t != cur.t)  // re-compute pipe_to_slice_index_map
    {
      // reset to -1 to make this error-proof. It could happen that a strand is missing in the slices for some reason.
      pipe_to_slice_index_map =
          std::vector<size_t>(strand_model.strand_model_skeleton.data.strand_group.PeekStrands().size(), -1);
      prev_slices.clear();
      for (const SlicingData& s : queue._Get_container()) {
        for (auto& pipe_handle : s.slice.second) {
          pipe_to_slice_index_map[pipe_handle] = s.index;
        }

        prev_slices.push_back(s);
      }

      t = cur.t;
      index = 0;
    }

    queue.pop();

    if (DEBUG_OUTPUT)
      EVOENGINE_LOG("Took next slice with t = " << cur.t << " out of the queue");

    std::vector<SlicingData> slices = Slicing(strand_model, prev_slices, step_size, max_dist, vertices, tex_coords,
                                              indices, settings, pipe_to_slice_index_map, cur.index);

    for (SlicingData& s : slices) {
      s.index = index;
      index++;
      queue.push(s);
    }
  }
}

void IterativeSlicingMeshGenerator::Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                             std::vector<unsigned>& indices,
                                             const StrandModelMeshGeneratorSettings& settings) {
  // support mesh generation in framework
  std::vector<glm::vec2> dummy_tex_coords;
  std::vector<std::pair<unsigned, unsigned>> index_pairs;

  Generate(strand_model, vertices, dummy_tex_coords, index_pairs, settings);

  for (auto& pair : index_pairs) {
    indices.push_back(pair.first);
  }
}

void IterativeSlicingMeshGenerator::Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                             std::vector<glm::vec2>& tex_coords,
                                             std::vector<std::pair<unsigned, unsigned>>& index_pairs,
                                             const StrandModelMeshGeneratorSettings& settings) {
  const auto& skeleton = strand_model.strand_model_skeleton;
  const auto& pipe_group = skeleton.data.strand_group;

  if (pipe_group.PeekStrands().empty()) {
    return;
  }

  if (DEBUG_OUTPUT)
    EVOENGINE_LOG("getting first seg group");
  std::vector<StrandSegmentHandle> seg_group0 = GetSegGroup(pipe_group, 0);

  if (DEBUG_OUTPUT)
    EVOENGINE_LOG("determining max thickness");
  float max_thickness = 0.0f;
  ForEachSegment(pipe_group, seg_group0, [&](const StrandSegment& seg) {
    if (seg.end_thickness > max_thickness) {
      max_thickness = seg.end_thickness;
    }
  });

  float max_dist = 2 * max_thickness * sqrt(2) * 2.5f * settings.cluster_distance;
  // initial slice at root:
  std::vector<bool> visited(pipe_group.PeekStrands().size(), false);

  // prepare initial pipe cluster:
  PipeCluster pipe_cluster;

  for (StrandHandle strand_handle = 0; strand_handle < pipe_group.PeekStrands().size(); strand_handle++) {
    pipe_cluster.push_back(strand_handle);
  }

  float step_size = 1.0f / settings.steps_per_segment;
  // float max = settings.max_param;

  // all strands in the same slice initially
  std::vector<size_t> pipe_to_slice_index_map(strand_model.strand_model_skeleton.data.strand_group.PeekStrands().size(),
                                              0);
  auto first_slices =
      ComputeSlices(strand_model, pipe_cluster, 0, 0, max_dist, 3, pipe_to_slice_index_map, 0);  // TODO: magic number 3

  std::vector<SlicingData> start_slices;

  for (auto& slice : first_slices) {
    // create initial vertices
    size_t offset_vert = vertices.size();
    size_t offset_tex = tex_coords.size();

    bool is_first = true;

    for (auto& el : std::get<0>(slice)) {
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

    start_slices.push_back(SlicingData{std::make_pair<>(std::get<0>(slice), std::get<1>(slice)), offset_vert,
                                       offset_tex, 0, step_size, 0.0});
  }

  SliceIteratively(strand_model, start_slices, step_size, max_dist, vertices, tex_coords, index_pairs, settings);
}
};  // namespace eco_sys_lab_plugin