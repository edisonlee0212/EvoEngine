#include "DsKineticVoronoiMeshing.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_inverse.hpp>  // for inverse()
#include <glm/gtx/norm.hpp>            // for length2()
#include <queue>
#include <utility>
#include <vector>
#include "BufferExporter.hpp"
#include "ComputePipeline.hpp"
#include "DynamicStrands.hpp"
#include "KineticDelaunay.hpp"
#include "MeshIntersection.hpp"
#include "ObjExporter.hpp"
#include "Platform/Platform.hpp"
#include "ProgressBar.hpp"
#include "SegmentBuilder.hpp"
#include "Shader.hpp"
#include "VoronoiMesh.hpp"

using namespace eco_sys_lab_plugin;

// helper functions
std::vector<std::pair<size_t, double>> MergeSortedVectors(const std::vector<std::vector<double>>& inputs) {
  using Entry = std::pair<size_t, double>;  // (index of input vector, value)
  std::vector<std::pair<size_t, double>> result;

  struct HeapNode {
    size_t vec_idx;   // which input vector
    size_t elem_idx;  // index inside that vector
    double value;     // value itself

    bool operator>(const HeapNode& other) const {
      return value > other.value;  // for min-heap
    }
  };

  std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> min_heap;

  // Initialize heap with the first element of each vector
  for (size_t i = 0; i < inputs.size(); ++i) {
    if (!inputs[i].empty()) {
      min_heap.push({i, 0, inputs[i][0]});
    }
  }

  while (!min_heap.empty()) {
    auto node = min_heap.top();
    min_heap.pop();

    // record (vector index, value)
    result.emplace_back(node.vec_idx, node.value);

    // advance in that vector
    if (node.elem_idx + 1 < inputs[node.vec_idx].size()) {
      min_heap.push({node.vec_idx, node.elem_idx + 1, inputs[node.vec_idx][node.elem_idx + 1]});
    }
  }

  return result;
}

/**
 * @brief Compute a 3D affine transformation that maps three coplanar source
 *        points to three coplanar target points, assuming an affine
 *        transformation that preserves the normalized plane normal direction.
 *
 * Given three non-collinear source points (p0, p1, p2) and their corresponding
 * non-collinear target points (q0, q1, q2), this function constructs the unique
 * affine transform T that satisfies:
 *
 *     T * vec4(p0, 1) = vec4(q0, 1)
 *     T * vec4(p1, 1) = vec4(q1, 1)
 *     T * vec4(p2, 1) = vec4(q2, 1)
 *
 * as well as:
 *
 *     T * n  = n'
 *
 * where n and n' are the normalized plane normals of the source and target
 * triangles, respectively. The normal direction is enforced to avoid the
 * underdetermined case that arises when all points lie in a plane.
 *
 * @note The returned transform maps points **from the source frame to the
 *       target frame**, i.e.:
 *
 *           T * vec4(p, 1) = vec4(q, 1)
 *
 *       for any point p lying in the same plane as (p0,p1,p2).
 *
 * @param p0 First source point in 3D.
 * @param p1 Second source point in 3D.
 * @param p2 Third source point in 3D.
 * @param q0 Corresponding target point to p0.
 * @param q1 Corresponding target point to p1.
 * @param q2 Corresponding target point to p2.
 *
 * @return glm::mat4 The affine transformation matrix T such that T * p = q.
 *
 * @throws Undefined behavior if the three source or target points are collinear
 *         (i.e., they do not span a plane).
 */
glm::mat4 ComputeAffineFromCoplanarPoints(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2,
                                          const glm::vec3& q0, const glm::vec3& q1, const glm::vec3& q2) {
  // --- Source basis ---
  glm::vec3 u = p1 - p0;
  glm::vec3 v = p2 - p0;
  glm::vec3 n = glm::normalize(glm::cross(u, v));

  // --- Target basis ---
  glm::vec3 up = q1 - q0;
  glm::vec3 vp = q2 - q0;
  glm::vec3 np = glm::normalize(glm::cross(up, vp));

  // (Optional) ensure consistent orientation.
  // If dot(n, np) < 0, flip np.
  if (glm::dot(n, np) < 0.0f)
    np = -np;

  // Build basis matrices B and B'
  glm::mat3 B;
  B[0] = u;  // column 0
  B[1] = v;  // column 1
  B[2] = n;  // column 2

  glm::mat3 Bp;
  Bp[0] = up;
  Bp[1] = vp;
  Bp[2] = np;

  // Linear part: A = B' * inverse(B)
  glm::mat3 A = Bp * glm::inverse(B);

  // Translation: t = q0 - A * p0
  glm::vec3 t = q0 - A * p0;

  // Assemble full 4x4 affine transform
  glm::mat4 T(1.0f);
  T[0][0] = A[0][0];
  T[1][0] = A[1][0];
  T[2][0] = A[2][0];
  T[0][1] = A[0][1];
  T[1][1] = A[1][1];
  T[2][1] = A[2][1];
  T[0][2] = A[0][2];
  T[1][2] = A[1][2];
  T[2][2] = A[2][2];

  T[3] = glm::vec4(t, 1.0f);

  return T;
}

std::optional<std::array<size_t, 3>> FindNonCollinearTriple(std::function<glm::vec3(size_t)> get_point, size_t size,
                                                            float eps = 1e-6f) {
  if (size < 3)
    return std::optional<std::array<size_t, 3>>();

  // Step 1: choose p0
  size_t i0 = 0;
  size_t i1 = -1;
  size_t i2 = -1;
  // Step 2: choose p1 - must be distinct from p0

  for (int j = 1; j < size; ++j) {
    if (glm::length(get_point(j) - get_point(i0)) > eps) {
      i1 = j;
      break;
    }
  }
  if (i1 == -1)
    return std::optional<std::array<size_t, 3>>();  // all points identical

  // Step 3: find p2 that makes area > 0
  for (int k = i1 + 1; k < size; ++k) {
    glm::vec3 u = get_point(i1) - get_point(i0);
    glm::vec3 v = get_point(k) - get_point(i0);
    float area2 = glm::length(glm::cross(u, v));
    if (area2 > eps) {
      i2 = k;
      return std::optional<std::array<size_t, 3>>({i0, i1, i2});  // non-collinear triple found
    }
  }

  return std::optional<std::array<size_t, 3>>();  // all points collinear
}

std::optional<std::array<size_t, 2>> FindNonIdenticalPair(std::function<glm::vec3(size_t)> get_point, size_t size,
                                                          float eps = 1e-6f) {
  if (size < 2)
    return std::optional<std::array<size_t, 2>>();

  size_t i0 = 0;
  size_t i1 = -1;
  for (int j = 1; j < size; ++j) {
    if (glm::length(get_point(j) - get_point(i0)) > eps) {
      i1 = j;
      return std::optional<std::array<size_t, 2>>({i0, i1});
    }
  }
  return std::optional<std::array<size_t, 2>>();  // all points identical
}

glm::vec3 ProfileToModelCoordinates(const std::vector<std::vector<glm::mat4>>& profile_to_model_transforms,
                                    kinDS::VoronoiPoint<3> point, float t, const std::vector<size_t>& branch_indices,
                                    float w = 1.0f) {
  size_t lower_section_index = static_cast<size_t>(std::max(0.0f, glm::floor(t)));

  size_t upper_section_index = std::min(profile_to_model_transforms.size() - 1, static_cast<size_t>(glm::ceil(t)));

  // check range
  auto coord_str = std::to_string(t);
  if (lower_section_index >= profile_to_model_transforms.size()) {
    std::cout << ("ProfileToModelCoordinates: lower bound of point z-coordinate out of range: " + coord_str).c_str()
              << std::endl;
  }
  if (upper_section_index >= profile_to_model_transforms.size()) {
    std::cout << ("ProfileToModelCoordinates: upper bound of point z-coordinate out of range: " + coord_str).c_str()
              << std::endl;
  }

  // only set second coordinate to 0 for points, not for normal vectors
  // TODO: I actually wanted to get rid of this coordinate swap at some point
  glm::vec4 local_pos(point[0], (1.0f - w) * point[2], point[1], w);
  size_t lower_branch_index = branch_indices[lower_section_index];
  glm::vec4 global_pos = profile_to_model_transforms[lower_section_index][lower_branch_index] * local_pos;

  if (upper_section_index != lower_section_index) {
    size_t upper_branch_index = branch_indices[upper_section_index];
    glm::vec4 upper_global_pos = profile_to_model_transforms[upper_section_index][upper_branch_index] * local_pos;
    float frac = static_cast<float>(t - static_cast<double>(lower_section_index));
    global_pos = glm::mix(global_pos, upper_global_pos, frac);
  }

  if (w == 0.0f) {
    global_pos = glm::normalize(global_pos);
  }

  return glm::vec3(global_pos);
}

glm::vec3 ToVec3(const kinDS::VoronoiPoint<3>& a) {
  return glm::vec3(static_cast<float>(a[0]), static_cast<float>(a[1]), static_cast<float>(a[2]));
}

void DsKineticVoronoiMeshing::RunMeshingAlgorithm(
    const std::vector<std::vector<kinDS::VoronoiPoint<2>>>& support_points,
    std::vector<std::vector<double>>& subdivisions_by_strand,
    std::vector<std::vector<int>>& physics_strand_to_segment_indices,
    const std::vector<std::vector<glm::mat4>>& transforms_by_height_and_branch, const GlobalTransform& root_transform,
    const std::vector<std::vector<size_t>>& branch_indices,
    std::vector<std::vector<std::vector<size_t>>>& strands_by_branch_id) {
  bool recompute_segment_pairs = false;  // TODO: expose as option?

  std::vector<std::vector<glm::mat4>> normal_transforms_by_height_and_branch(transforms_by_height_and_branch.size());

  for (size_t i = 0; i < transforms_by_height_and_branch.size(); i++) {
    normal_transforms_by_height_and_branch[i].resize(transforms_by_height_and_branch[i].size());
    for (size_t j = 0; j < normal_transforms_by_height_and_branch[i].size(); j++) {
      normal_transforms_by_height_and_branch[i][j] =
          glm::transpose(glm::inverse(transforms_by_height_and_branch[i][j]));
      normal_transforms_by_height_and_branch[i][j][3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    }
  }

  // sort subdivisions into a single array
  std::vector<std::pair<size_t, double>> subdivisions = MergeSortedVectors(subdivisions_by_strand);

  ;

  EVOENGINE_LOG("Starting Kinetic Delaunay Voronoi Meshing...");
  kinDS::KineticDelaunay kinetic_delaunay(
      kinDS::BranchTrajectories(support_points, transforms_by_height_and_branch, branch_indices, strands_by_branch_id),
      render_settings.segment_meshlet_render_parameters.alpha_cutoff, false, branch_indices, strands_by_branch_id);

  bool transform_mesh_at_construction = false;

  kinetic_delaunay.init();
  kinDS::SegmentBuilder mesh_builder(kinetic_delaunay, subdivisions, transform_mesh_at_construction);
  mesh_builder.init();
  // auto points = kinetic_delaunay.getPointsAt(0.0);

  size_t section_count = kinetic_delaunay.getSectionCount();

  evo_engine::ProgressBar section_progress_bar(0, section_count, "Computing Kinetic Voronoi Sections",
                                               evo_engine::ProgressBar::Display::Absolute);
  for (size_t i = 0; i < section_count; ++i) {
    section_progress_bar.Update(i);
    if (i != 0)
      mesh_builder.betweenSections(i);
    kinetic_delaunay.advanceOneSection(mesh_builder);

    // points = kinetic_delaunay.getPointsAt(static_cast<double>(i + 1));
  }
  section_progress_bar.Finish();

  EVOENGINE_LOG("Finalizing Kinetic Delaunay Voronoi Meshing...");
  mesh_builder.finalize(section_count);

  auto [meshes, meshing_neighbor_indices] = mesh_builder.extractSegmentMeshlets();

  auto& boundary_mesh = mesh_builder.getBoundaryMesh();

  bool debug_export_meshes = true;

  if (debug_export_meshes) {
    kinDS::ObjExporter::writeMesh(boundary_mesh, "boundary_mesh.obj");
  }
  size_t max_meshlet_export = 50000000;
  // intersect all meshes with the boundary mesh and save the result
  // Build an AABB-tree of the boundary-mesh to prefilter
  kinDS::MeshIntersection boundary_intersector(boundary_mesh);

  ProgressBar intersection_progress_bar(0, meshes.size(), "Computing Mesh Intersections",
                                        ProgressBar::Display::Absolute, 50);

  std::atomic<int> progress_counter{0};
  Jobs::RunParallelFor(meshes.size(), [&](const size_t mesh_index) {
    progress_counter.fetch_add(1, std::memory_order_relaxed);
    intersection_progress_bar.Update(progress_counter);
    auto intersect_relation = boundary_intersector.ClassifyMeshRelation(meshes[mesh_index], true);

    switch (intersect_relation) {
      case kinDS::MeshIntersection::MeshRelation::INSIDE:
        // do nothing
        break;

      case kinDS::MeshIntersection::MeshRelation::INTERSECTING:
        if (debug_export_meshes && mesh_index < max_meshlet_export) {
          kinDS::ObjExporter::writeMesh(meshes[mesh_index], "meshlet" + std::to_string(mesh_index) + "_raw.obj");
        }
        std::tie(meshes[mesh_index], meshing_neighbor_indices[mesh_index]) =
            boundary_intersector.Intersect(meshes[mesh_index], meshing_neighbor_indices[mesh_index]);
        break;

      case kinDS::MeshIntersection::MeshRelation::OUTSIDE:
        // fully outside, result is empty mesh
        meshes[mesh_index] = kinDS::VoronoiMesh();
        meshing_neighbor_indices[mesh_index] = {};
        break;

      case kinDS::MeshIntersection::MeshRelation::UNDEFINED:
        EVOENGINE_ERROR("Mesh relation returned UNDEFINED");
        break;

      default:
        EVOENGINE_ERROR("Unknown return value of mesh relation");
        break;
    }
  });

  intersection_progress_bar.Finish();

  // Find empty meshes and try to fix them by expanding neighboring meshes
  std::vector<size_t> empty_mesh_indices;
  for (size_t mesh_index = 0; mesh_index < meshes.size(); mesh_index++) {
    if (meshes[mesh_index].getVertexCount() == 0) {
      empty_mesh_indices.push_back(mesh_index);
    }
  }

  // go through all triangles of all meshes and check their neighbor indices
  // if a neighbor index corresponds to an empty mesh, try to copy the triangle into the corresponding empty mesh
  for (size_t mesh_index = 0; mesh_index < meshes.size(); mesh_index++) {
    if (std::binary_search(empty_mesh_indices.begin(), empty_mesh_indices.end(), mesh_index)) {
      continue;
    }

    auto& mesh = meshes[mesh_index];
    auto& neighbor_indices = meshing_neighbor_indices[mesh_index];
    const auto& triangles = mesh.getTriangles();

    for (size_t triangle_index = 0; triangle_index < triangles.size(); triangle_index += 3) {
      int neighbor_mesh_index = neighbor_indices[triangle_index / 3];
      if (neighbor_mesh_index >= 0) {
        // indices are sorted, so we can use binary search
        if (std::binary_search(empty_mesh_indices.begin(), empty_mesh_indices.end(), neighbor_mesh_index)) {
          kinDS::VoronoiMesh& neighbor_mesh = meshes[neighbor_mesh_index];

          size_t v0_index = triangles[triangle_index];
          size_t v1_index = triangles[triangle_index + 1];
          size_t v2_index = triangles[triangle_index + 2];

          kinDS::VoronoiPoint<3> v0 = mesh.getVertices()[v0_index];
          kinDS::VoronoiPoint<3> v1 = mesh.getVertices()[v1_index];
          kinDS::VoronoiPoint<3> v2 = mesh.getVertices()[v2_index];

          kinDS::VoronoiPoint<3> t0 = mesh.getUV(triangle_index);
          kinDS::VoronoiPoint<3> t1 = mesh.getUV(triangle_index + 1);
          kinDS::VoronoiPoint<3> t2 = mesh.getUV(triangle_index + 2);

          // TODO: normals, uvs, other
          size_t new_v0_index = neighbor_mesh.addVertex(v0);
          size_t new_v1_index = neighbor_mesh.addVertex(v1);
          size_t new_v2_index = neighbor_mesh.addVertex(v2);

          size_t new_t0_index = neighbor_mesh.addUV(t0);
          size_t new_t1_index = neighbor_mesh.addUV(t1);
          size_t new_t2_index = neighbor_mesh.addUV(t2);

          // invert order to maintain consistent orientation
          neighbor_mesh.addTriangle(new_v1_index, new_v0_index, new_v2_index, new_t0_index, new_t1_index, new_t2_index);
          meshing_neighbor_indices[neighbor_mesh_index].push_back(mesh_index);
        }
      }
    }
  }

  // merge vertices and check how many could be (partially) fixed
  size_t fixed_mesh_count = 0;
  for (size_t mesh_index : empty_mesh_indices) {
    auto& mesh = meshes[mesh_index];

    if (mesh.getVertexCount() == 0) {
      continue;
    }

    mesh.mergeDuplicateVertices(1e-6);
    mesh.removeDegenerateTriangles();
    mesh.removeIsolatedVertices();
    mesh.computeNormals(kinDS::PerTriangleCorner);

    mesh.patchHoles(
        [&](size_t tri_index) {
          meshing_neighbor_indices[mesh_index].push_back(-2);
          // TODO: copy from neighbor mesh
        },
        [&](size_t v_index, size_t tri_index) {
          auto& v = mesh.getVertices()[v_index];
          // query point in boundary mesh
          auto match = boundary_intersector.MatchPointOnSurface(v);
          if (!match.hit) {
            // mark as not bark
            meshing_neighbor_indices[mesh_index][tri_index] = -1;
            mesh.addUV({0, 0, 0});
            mesh.addNormal({0, 0, 0});
          } else {
            size_t matched_triangle_index = match.triangle_index;

            // use barycentric coordinates to interpolate normal and uv from boundary mesh

            // first get boundary triangle vertices
            const auto& boundary_triangles = boundary_mesh.getTriangles();
            size_t bt0_index = boundary_triangles[matched_triangle_index * 3];
            size_t bt1_index = boundary_triangles[matched_triangle_index * 3 + 1];
            size_t bt2_index = boundary_triangles[matched_triangle_index * 3 + 2];

            // get normals
            auto n0 = boundary_mesh.getNormal(bt0_index);
            auto n1 = boundary_mesh.getNormal(bt1_index);
            auto n2 = boundary_mesh.getNormal(bt2_index);

            // get uvs
            auto uv0 = boundary_mesh.getUV(bt0_index);
            auto uv1 = boundary_mesh.getUV(bt1_index);
            auto uv2 = boundary_mesh.getUV(bt2_index);

            // interpolate
            auto n = n0 * match.u + n1 * match.v + n2 * match.w;
            n = n.normalized();

            auto uv = uv0 * match.u + uv1 * match.v + uv2 * match.w;
            mesh.addUV(uv);
            mesh.addNormal(n);
          }
        });

    fixed_mesh_count++;
  }

  EVOENGINE_LOG("Fixed " << fixed_mesh_count << " out of " << empty_mesh_indices.size() << " meshes.");

  const auto& meshing_strand_to_segment_indices = mesh_builder.getStrandToSegmentIndices();

  size_t max_meshing_id = 0;
  for (size_t strand_id = 0; strand_id < meshing_strand_to_segment_indices.size(); ++strand_id) {
    for (size_t segment_no = 0; segment_no < meshing_strand_to_segment_indices[strand_id].size(); ++segment_no) {
      size_t meshing_segment_id = meshing_strand_to_segment_indices[strand_id][segment_no];
      max_meshing_id = std::max(max_meshing_id, meshing_segment_id);
    }
  }

  std::vector<size_t> meshing_to_physics_segment_indices(max_meshing_id + 1, -1);
  for (size_t strand_id = 0; strand_id < physics_strand_to_segment_indices.size(); ++strand_id) {
    for (size_t segment_no = 0; segment_no < meshing_strand_to_segment_indices[strand_id].size(); ++segment_no) {
      size_t meshing_segment_id = meshing_strand_to_segment_indices[strand_id][segment_no];
      int physics_segment_id = physics_strand_to_segment_indices[strand_id][segment_no];
      meshing_to_physics_segment_indices[meshing_segment_id] = physics_segment_id;
    }
  }

  std::vector<DynamicStrands::GpuSegmentData>& segment_data_list = dynamic_strands->segment_data_list;
  std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs = dynamic_strands->segment_pairs;

  if (recompute_segment_pairs) {
    segment_pairs.clear();

    std::vector<size_t> pair_handle_offsets(segment_data_list.size(), 0);

    // clear existing pair handles in segment data
    for (auto& segment_data : segment_data_list) {
      for (auto& pair_handle : segment_data.pair_handles) {
        pair_handle = -1;
      }
    }

    for (size_t strand_id = 0; strand_id < physics_strand_to_segment_indices.size(); ++strand_id) {
      for (size_t segment_no = 0; segment_no < meshing_strand_to_segment_indices[strand_id].size(); ++segment_no) {
        size_t meshing_segment_id = meshing_strand_to_segment_indices[strand_id][segment_no];
        auto& mesh = meshes[meshing_segment_id];
        int physics_segment_id = physics_strand_to_segment_indices[strand_id][segment_no];
        const auto& triangles = mesh.getTriangles();

        std::set<int> neighbor_set;
        for (size_t triangle_vertex_index = 0; triangle_vertex_index < triangles.size(); triangle_vertex_index += 3) {
          int meshing_neighbor_segment_index = meshing_neighbor_indices[meshing_segment_id][triangle_vertex_index / 3];
          if (meshing_neighbor_segment_index >= 0) {
            int physics_neighbor_segment_index = meshing_to_physics_segment_indices[meshing_neighbor_segment_index];
            if (physics_neighbor_segment_index != -1) {
              neighbor_set.insert(physics_neighbor_segment_index);
            }
          }
        }

        // create a new segment pair for each neighbor if the neighbor index is greater to avoid duplicates from
        // symmetry
        for (auto& physics_neighbor_segment_index : neighbor_set) {
          if (physics_neighbor_segment_index > physics_segment_id) {
            DynamicStrands::GpuSegmentPair segment_pair;
            segment_pair.segment0_handle = physics_segment_id;
            segment_pair.segment1_handle = physics_neighbor_segment_index;

            // TODO: other properties

            int pair_handle = static_cast<int>(segment_pairs.size());
            segment_data_list[physics_segment_id].pair_handles[pair_handle_offsets[physics_segment_id]] = pair_handle;
            pair_handle_offsets[physics_segment_id]++;

            segment_data_list[physics_neighbor_segment_index]
                .pair_handles[pair_handle_offsets[physics_neighbor_segment_index]] = pair_handle;
            pair_handle_offsets[physics_neighbor_segment_index]++;

            segment_pairs.emplace_back(segment_pair);
          }
        }
      }
    }
  }

  for (size_t strand_id = 0; strand_id < physics_strand_to_segment_indices.size(); ++strand_id) {
    // Verify segment count:
    if (meshing_strand_to_segment_indices[strand_id].size() != physics_strand_to_segment_indices[strand_id].size()) {
      EVOENGINE_WARNING("Meshing algorithm resulted in "
                        << meshing_strand_to_segment_indices[strand_id].size() << " segments for strand " << strand_id
                        << ", but the physics simulation has " << physics_strand_to_segment_indices[strand_id].size()
                        << ". There are " << subdivisions_by_strand[strand_id].size()
                        << " subdivision parameters in range [" << subdivisions_by_strand[strand_id].front() << ", "
                        << subdivisions_by_strand[strand_id].back() << "].");
    }

    for (size_t segment_no = 0; segment_no < meshing_strand_to_segment_indices[strand_id].size(); ++segment_no) {
      // Get mesh using the segment id from the meshing algorithm.
      // Note that this is different from the original segment id from the strand model because it is assigned in the
      // order of creation of the segments.
      size_t meshing_segment_id = meshing_strand_to_segment_indices[strand_id][segment_no];
      auto& mesh = meshes[meshing_segment_id];
      int physics_segment_id = physics_strand_to_segment_indices[strand_id][segment_no];
      // get original segment id from strand model

      // store in the buffers
      size_t vertex_offset = segment_meshlet_vertices.size();
      for (const auto& v : mesh.getVertices()) {
        // v is a relative position in 2D, we need to convert it to 3D

        GpuSegmentMeshletVertex vertex;

        // Convert from profile to global 3D position

        if (!transform_mesh_at_construction) {
          vertex.x0 = root_transform.TransformPoint(
              ProfileToModelCoordinates(transforms_by_height_and_branch, v, v[2], branch_indices[strand_id]));
        } else {
          vertex.x0 = root_transform.TransformPoint(glm::vec3(v[0], v[1], v[2]));
        }
        vertex.x = vertex.x0;
        vertex.segment_index = physics_segment_id;
        segment_meshlet_vertices.push_back(vertex);
      }

      const auto& triangles = mesh.getTriangles();

      // for debugging, output all segment ids of the pair from both sources
      /*EVOENGINE_LOG("Physics simulation neighbors:")
      std::ostringstream oss;
      oss << '[';
      for (int pair_handle : segment_data_list[physics_segment_id].pair_handles) {
        int neighbor_segment_id = -1;
        if (pair_handle == -1) {
          continue;
        }

        if (segment_pairs[pair_handle].segment0_handle == physics_segment_id) {
          neighbor_segment_id = segment_pairs[pair_handle].segment1_handle;
        } else if (segment_pairs[pair_handle].segment1_handle == physics_segment_id) {
          neighbor_segment_id = segment_pairs[pair_handle].segment0_handle;
        }

        oss << neighbor_segment_id << ", ";
      }

      std::string s = oss.str();
      s.replace(s.size() - 2, 2, "]");
      std::cout << s << std::endl;

      EVOENGINE_LOG("Meshing neighbors:")
      std::set<int> neighbor_set;
      for (size_t triangle_vertex_index = 0; triangle_vertex_index < triangles.size(); triangle_vertex_index += 3) {
        int meshing_neighbor_segment_index = meshing_neighbor_indices[meshing_segment_id][triangle_vertex_index / 3];
        if (meshing_neighbor_segment_index >= 0) {
          int physics_neighbor_segment_index = meshing_to_physics_segment_indices[meshing_neighbor_segment_index];
          neighbor_set.insert(physics_neighbor_segment_index);
        }
      }

      std::cout << '[';
      for (auto& index : neighbor_set) {
        std::cout << index << ", ";
      }
      std::cout << ']' << std::endl;*/

      for (size_t triangle_vertex_index = 0; triangle_vertex_index < triangles.size(); triangle_vertex_index += 3) {
        GpuSegmentMeshletTriangle triangle;
        triangle.vertex_index0 = static_cast<unsigned int>(triangles[triangle_vertex_index] + vertex_offset);
        triangle.vertex_index1 = static_cast<unsigned int>(triangles[triangle_vertex_index + 1] + vertex_offset);
        triangle.vertex_index2 = static_cast<unsigned int>(triangles[triangle_vertex_index + 2] + vertex_offset);
        int meshing_neighbor_segment_index = meshing_neighbor_indices[meshing_segment_id][triangle_vertex_index / 3];
        if (meshing_neighbor_segment_index >= static_cast<long>(meshing_to_physics_segment_indices.size())) {
          EVOENGINE_ERROR("meshing_neighbor_segment_index out of bounds: " << meshing_neighbor_segment_index
                                                                           << "; upper bound is: "
                                                                           << meshing_to_physics_segment_indices.size())
        } else if (meshing_neighbor_segment_index >= 0) {
          triangle.neighbor_segment_index = meshing_to_physics_segment_indices[meshing_neighbor_segment_index];
        } else {
          triangle.neighbor_segment_index = meshing_neighbor_segment_index;
        }

        // Just assume the transformations (without translations) are orthogonal
        for (size_t j = 0; j < 3; j++) {
          auto source_tri_vertex_index = mesh.getTriangles()[triangle_vertex_index + j];
          auto normal = mesh.getNormal(triangle_vertex_index + j);
          if (!transform_mesh_at_construction) {
            triangle.normal0[j] = triangle.normal[j] =
                glm::vec4(root_transform.TransformVector(ProfileToModelCoordinates(
                              normal_transforms_by_height_and_branch, normal,
                              mesh.getVertices()[source_tri_vertex_index][2], branch_indices[strand_id], 0.0f)),
                          0.0f);
          } else {
            triangle.normal0[j] = triangle.normal[j] =
                glm::vec4(root_transform.TransformVector(glm::vec3(normal[0], normal[1], normal[2])), 0.0f);
          }

          if (mesh.hasValidUVIndex(triangle_vertex_index + j)) {
            triangle.uv[j] = glm::vec4(ToVec3(mesh.getUV(triangle_vertex_index + j)), 0.0);
          } else {
            triangle.uv[j] = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
          }
        }

        triangle.segment_pair_index = -1;

        if (triangle.neighbor_segment_index >= 0) {
          // find segment pair index (TODO: this is not very efficient, perhaps we can improve it in the future)
          for (int pair_handle : segment_data_list[physics_segment_id].pair_handles) {
            if (pair_handle == -1) {
              continue;
            }
            if (segment_pairs[pair_handle].segment0_handle != physics_segment_id &&
                segment_pairs[pair_handle].segment1_handle != physics_segment_id) {
              EVOENGINE_ERROR("Segment pair incorrectly referenced!");
            }
            if (segment_pairs[pair_handle].segment0_handle == triangle.neighbor_segment_index ||
                segment_pairs[pair_handle].segment1_handle == triangle.neighbor_segment_index) {
              triangle.segment_pair_index = static_cast<int>(pair_handle);
              break;
            }
          }

          if (triangle.segment_pair_index == -1) {
            /*EVOENGINE_WARNING("Could not find segment pair index for segment "
                              << physics_segment_id << " and neighbor segment " << triangle.neighbor_segment_index);*/
          }
        }

        segment_meshlet_triangles.push_back(triangle);
      }
    }
  }

  if (!debug_export_meshes) {
    EVOENGINE_LOG("Kinetic Delaunay Voronoi Meshing completed.");
    return;
  }
  EVOENGINE_LOG("Exporting Kinetic Delaunay Voronoi Meshes for Debugging...");
  kinDS::VoronoiMesh transformed_mesh(kinDS::PerTriangleCorner);  // also build transformed mesh for debugging
  for (size_t strand_id = 0; strand_id < physics_strand_to_segment_indices.size(); ++strand_id) {
    for (size_t segment_no = 0; segment_no < meshing_strand_to_segment_indices[strand_id].size(); ++segment_no) {
      // Get mesh using the segment id from the meshing algorithm.
      // Note that this is different from the original segment id from the strand model because it is assigned in the
      // order of creation of the segments.
      auto& mesh = meshes[meshing_strand_to_segment_indices[strand_id][segment_no]];

      // get original segment id from strand model
      size_t segment_id = meshing_strand_to_segment_indices[strand_id][segment_no];

      // store in the buffers
      size_t vertex_offset = transformed_mesh.getVertexCount();
      for (const auto& v : mesh.getVertices()) {
        // v is a relative position in 2D, we need to convert it to 3D
        if (branch_indices.size() <= strand_id) {
          EVOENGINE_ERROR("strand id out if bounds: " << strand_id << " vector length is: " << branch_indices.size());
        }
        glm::vec3 global_pos;
        if (!transform_mesh_at_construction) {
          global_pos = root_transform.TransformPoint(
              ProfileToModelCoordinates(transforms_by_height_and_branch, v, v[2], branch_indices[strand_id]));
        } else {
          global_pos = root_transform.TransformPoint(glm::vec3(v[0], v[1], v[2]));
        }
        transformed_mesh.addVertex(global_pos[0], global_pos[1], global_pos[2]);
      }

      const auto& triangles = mesh.getTriangles();
      for (size_t triangle_vertex_index = 0; triangle_vertex_index < triangles.size(); triangle_vertex_index += 3) {
        size_t dest_tri_vertex_index =
            3 * transformed_mesh.addTriangle(triangles[triangle_vertex_index] + vertex_offset,
                                             triangles[triangle_vertex_index + 1] + vertex_offset,
                                             triangles[triangle_vertex_index + 2] + vertex_offset);

        for (size_t j = 0; j < 3; j++) {
          auto source_tri_vertex_index = mesh.getTriangles()[triangle_vertex_index + j];
          auto untransformed_normal = mesh.getNormal(triangle_vertex_index + j);
          glm::vec3 normal;
          if (!transform_mesh_at_construction) {
            normal = glm::vec3(root_transform.TransformVector(ProfileToModelCoordinates(
                normal_transforms_by_height_and_branch, untransformed_normal,
                mesh.getVertices()[source_tri_vertex_index][2], branch_indices[strand_id], 0.0f)));
          } else {
            normal = glm::vec3(root_transform.TransformVector(
                glm::vec3(untransformed_normal[0], untransformed_normal[1], untransformed_normal[2])));
          }
          size_t normal_index = transformed_mesh.addNormal(normal.x, normal.y, normal.z);

          if (mesh.hasValidUVIndex(triangle_vertex_index + j)) {
            size_t uv_index = transformed_mesh.addUV(mesh.getUV(triangle_vertex_index + j));
            transformed_mesh.getUVIndices()[dest_tri_vertex_index + j] = uv_index;
          }
        }
      }
    }
  }

  // comment this out for now, strand id can not be obtained
  /*kinDS::VoronoiMesh transformed_boundary_mesh(kinDS::PerTriangleCorner);  // also build transformed mesh for
  debugging

  const auto& vertices = boundary_mesh.getVertices();
  for (size_t i = 0; i < vertices.size(); i++) {
    const auto& v = vertices[i];
    // v is a relative position in 2D, we need to convert it to 3D
    glm::vec3 global_pos = root_transform.TransformPoint(
        ProfileToModelCoordinates(transforms_by_height_and_branch, v, v[2], branch_indices[strand_id]));
    transformed_boundary_mesh.addVertex(global_pos[0], global_pos[1], global_pos[2]);
  }

  const auto& triangles = boundary_mesh.getTriangles();
  for (size_t triangle_vertex_index = 0; triangle_vertex_index < triangles.size(); triangle_vertex_index += 3) {
    size_t dest_tri_vertex_index = 3 * transformed_boundary_mesh.addTriangle(triangles[triangle_vertex_index],
                                                                             triangles[triangle_vertex_index + 1],
                                                                             triangles[triangle_vertex_index + 2]);

    for (size_t j = 0; j < 3; j++) {
      auto source_tri_vertex_index = boundary_mesh.getTriangles()[triangle_vertex_index + j];

      glm::vec3 normal = glm::vec3(root_transform.TransformVector(ProfileToModelCoordinates(
          normal_transforms_by_height_and_branch, boundary_mesh.getNormal(triangle_vertex_index + j),
          boundary_mesh.getVertices()[source_tri_vertex_index][2], branch_indices[strand_id], 0.0f)));
      size_t normal_index = transformed_boundary_mesh.addNormal(normal.x, normal.y, normal.z);

      if (boundary_mesh.hasValidUVIndex(triangle_vertex_index + j)) {
        size_t uv_index = transformed_boundary_mesh.addUV(boundary_mesh.getUV(triangle_vertex_index + j));
        transformed_boundary_mesh.getUVIndices()[dest_tri_vertex_index + j] = uv_index;
      }
    }
  }*/

  // for now, just combine all meshes into one
  kinDS::VoronoiMesh combined_mesh;
  for (const auto& mesh : meshes) {
    combined_mesh += mesh;
  }

  // also export some meshlets:
  for (size_t i = 0; i < std::min(max_meshlet_export, meshes.size()); i++) {
    kinDS::ObjExporter::writeMesh(meshes[i], "meshlet" + std::to_string(i) + ".obj");
  }

  // export combined mesh
  // combined_mesh.mergeDuplicateVertices(0.0001);
  // kinDS::ObjExporter::writeMesh(transformed_mesh, "transformed_mesh.obj");
  // kinDS::ObjExporter::writeMesh(transformed_boundary_mesh, "transformed_boundary_mesh.obj");
  // kinDS::ObjExporter::writeMesh(combined_mesh, "meshtest_subdivided.obj");
  EVOENGINE_LOG("Kinetic Delaunay Voronoi Meshes exported.");
}

// DsKineticVoronoiMeshing implementation
DsKineticVoronoiMeshing::RenderSettings DsKineticVoronoiMeshing::render_settings = {};

DsKineticVoronoiMeshing::DsKineticVoronoiMeshing() {
}

DsKineticVoronoiMeshing::~DsKineticVoronoiMeshing() {
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::InitBuffer(
    VkBufferCreateInfo& buffer_create_info, VmaAllocationCreateInfo& buffer_vma_allocation_create_info) {
  device_segment_meshlet_triangles_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segment_meshlet_vertices_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::InitData(
    const DynamicStrandsInitializeParameters& initialize_parameters, const StrandModelSkeleton& strand_model_skeleton,
    const StrandModelStrandGroup& strand_model_strand_group, DtsStrandGroup& randomly_subdivided_strand_group,
    DtsStrandGroup& uniformly_subdivided_strand_group) {
  const auto& randomly_subdivided_strands = randomly_subdivided_strand_group.PeekStrands();
  const auto& randomly_subdivided_strand_segments = randomly_subdivided_strand_group.PeekStrandSegments();
  if (randomly_subdivided_strands.empty()) {
    EVOENGINE_LOG("Strand Group is empty!");
    return;
  }
  strand_model_strand_group.UniformlySubdivide<DtsStrandGroupData, DtsStrandData, DtsStrandSegmentData>(
      uniformly_subdivided_strand_group, initialize_parameters.uniform_subdivision,
      [&](const StrandHandle src_handle, DtsStrandData& strand_data) {
      },
      [&](const float start_root_distance, const float end_root_distance, const StrandSegmentHandle src_handle,
          const uint32_t original_segment_index, const float segment_t, DtsStrandSegmentData& segment_data,
          const uint32_t sub_segment_index) {
        const auto& src_segment_data = strand_model_strand_group.PeekStrandSegmentData(src_handle);
        segment_data.node_handle = src_segment_data.node_handle;
        segment_data.original_segment_handle = src_handle;
        segment_data.original_segment_index = original_segment_index;
        segment_data.segment_index = sub_segment_index;
        segment_data.original_segment_t = segment_t;
        segment_data.start_root_distance = start_root_distance;
        segment_data.end_root_distance = end_root_distance;
        const auto& strand_segment = strand_model_strand_group.PeekStrandSegment(src_handle);
        const auto& strand = strand_model_strand_group.PeekStrand(strand_segment.GetStrandHandle());
        const auto& strand_segment_handles = strand.PeekStrandSegmentHandles();

        glm::vec2 p0, p1, p3;
        const glm::vec2 p2 = src_segment_data.profile_position;
        float d0, d1, d3;
        const float d2 = src_segment_data.initial_distance_to_boundary;
        if (src_handle == strand_segment_handles.front()) {
          d1 = d2;
          d0 = d1 * 2.0f - d2;

          p1 = p2;
          p0 = p1 * 2.0f - p2;
        } else if (strand_segment.GetPrevHandle() == strand_segment_handles.front()) {
          const auto& prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetPrevHandle());
          d0 = d2;
          d1 = prev_segment_data.initial_distance_to_boundary;

          p0 = p2;
          p1 = prev_segment_data.profile_position;
        } else {
          const auto& prev_segment = strand_model_strand_group.PeekStrandSegment(strand_segment.GetPrevHandle());
          const auto& prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetPrevHandle());
          const auto& prev_prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(prev_segment.GetPrevHandle());
          d0 = prev_prev_segment_data.initial_distance_to_boundary;
          d1 = prev_segment_data.initial_distance_to_boundary;

          p0 = prev_prev_segment_data.profile_position;
          p1 = prev_segment_data.profile_position;
        }
        if (src_handle == strand_segment_handles.back()) {
          d3 = d2 * 2.0f - d1;

          p3 = p2 * 2.0f - p1;
        } else {
          const auto& next_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetNextHandle());
          d3 = next_segment_data.initial_distance_to_boundary;

          p3 = next_segment_data.profile_position;
        }
        segment_data.initial_distance_to_boundary = Strands::CubicInterpolation(d0, d1, d2, d3, segment_t);
        segment_data.profile_position = Strands::CubicInterpolation(p0, p1, p2, p3, segment_t);

        const auto calculate_polar_coordinates = [](const glm::vec2& profile_position) {
          const auto r = glm::length(profile_position);
          if (r <= glm::epsilon<float>()) {
            return glm::vec2(0.0f);
          }
          if (profile_position.y >= 0)
            return glm::vec2(r, glm::acos(profile_position.x / r));
          return glm::vec2(r, -glm::acos(profile_position.x / r));
        };

        segment_data.profile_polar_coordinate = calculate_polar_coordinates(segment_data.profile_position);
      },
      (initialize_parameters.min_segment_length + initialize_parameters.max_segment_length) * .5f * .01f);

  struct GuidePoint {
    kinDS::VoronoiPoint<2> profile_position;
    SkeletonNodeHandle node_handle;
    StrandSegmentHandle segment_handle;
  };

  std::vector<std::vector<GuidePoint>> strand_guide_points(randomly_subdivided_strands.size());
  std::vector<int> uniform_particle_offsets(randomly_subdivided_strands.size());
  if (!uniform_particle_offsets.empty())
    uniform_particle_offsets[0] = 0;
  for (uint32_t strand_index = 1; strand_index < randomly_subdivided_strands.size(); strand_index++) {
    uniform_particle_offsets[strand_index] =
        uniform_particle_offsets[strand_index - 1] +
        uniformly_subdivided_strand_group.PeekStrand(strand_index - 1).PeekStrandSegmentHandles().size() + 1;
  }

  std::vector<std::vector<int>> randomly_subdivided_segment_handles(randomly_subdivided_strands.size());
  std::vector<std::vector<double>> random_subdivisions_by_strand(randomly_subdivided_strands.size());

  // for debugging
  std::vector<std::vector<double>> uniform_subdivisions_by_strand(randomly_subdivided_strands.size());

  int maxSegmentCount = std::numeric_limits<int>::min();
  std::mutex m;

  std::function<void(int&, int)> updateMax = [&](int& cur_max, int candidate) {
    std::lock_guard<std::mutex> lock(m);
    cur_max = std::max(cur_max, candidate);
  };

  Jobs::RunParallelFor(randomly_subdivided_strands.size(), [&](const size_t strand_index) {
    auto& random_subdivided_strand = randomly_subdivided_strands[strand_index];
    auto& uniformly_subdivided_strand = uniformly_subdivided_strand_group.PeekStrand(strand_index);

    const auto uniform_particle_offset = uniform_particle_offsets[strand_index];

    size_t segment_handle = uniformly_subdivided_strand.PeekStrandSegmentHandles()[0];
    auto& first_uniform_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(segment_handle);
    auto node_handle = first_uniform_segment_data.node_handle;

    // First 2 particles within same strand will always have same profile position/polar coordinate.
    kinDS::VoronoiPoint<2> profile_position{first_uniform_segment_data.profile_position.x,
                                            first_uniform_segment_data.profile_position.y};
    GuidePoint guide_point;
    guide_point.profile_position = profile_position;
    guide_point.node_handle = node_handle;
    guide_point.segment_handle = segment_handle;

    strand_guide_points[strand_index].push_back(guide_point);

    int last_index_with_new_node = 0;
    float previous_root_distance = 0.0f;
    updateMax(maxSegmentCount, uniformly_subdivided_strand.PeekStrandSegmentHandles().size());
    for (int uniform_segment_index = 0;
         uniform_segment_index < uniformly_subdivided_strand.PeekStrandSegmentHandles().size();
         uniform_segment_index++) {
      size_t segment_handle = uniformly_subdivided_strand.PeekStrandSegmentHandles()[uniform_segment_index];
      const auto& uniform_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(segment_handle);
      const auto& uniform_segment = uniformly_subdivided_strand_group.PeekStrandSegment(segment_handle);
      auto node_handle = uniform_segment_data.node_handle;
      kinDS::VoronoiPoint<2> profile_position{uniform_segment_data.profile_position.x,
                                              uniform_segment_data.profile_position.y};

      /* if (uniform_segment_data.segment_index != strand_guide_points[strand_index].size()) {
        EVOENGINE_WARNING(std::string("Deviation detected in guide point generation: guide point no. " +
                                      std::to_string(strand_guide_points[strand_index].size()) + " has distance " +
                                      std::to_string(uniform_segment_data.segment_index)));
      }*/
      GuidePoint guide_point;
      guide_point.profile_position = profile_position;
      guide_point.node_handle = node_handle;
      guide_point.segment_handle = segment_handle;

      strand_guide_points[strand_index].push_back(guide_point);
      uniform_subdivisions_by_strand[strand_index].push_back(uniform_segment.end_t);
    }

    // Iterate through randomly subdivided segments to obtain segment indices and subdivisions
    for (int random_segment_index = 0;
         random_segment_index < random_subdivided_strand.PeekStrandSegmentHandles().size(); random_segment_index++) {
      size_t segment_handle = random_subdivided_strand.PeekStrandSegmentHandles()[random_segment_index];
      const auto& segment = randomly_subdivided_strand_segments[segment_handle];
      const auto& random_segment_data = randomly_subdivided_strand_group.PeekStrandSegmentData(
          random_subdivided_strand.PeekStrandSegmentHandles()[random_segment_index]);

      randomly_subdivided_segment_handles[strand_index].push_back(segment_handle);

      // scale parameters to subdivision
      if (!isnan(segment.end_t)) {
        random_subdivisions_by_strand[strand_index].push_back(
            initialize_parameters.uniform_subdivision * (segment.end_t + random_segment_data.original_segment_index));
      }
    }
  });

  // Sort profile positions and global positions by t (TODO: should be by node index later). This needs to be sequential
  // because push_back is not thread-safe.
  size_t node_count = strand_model_skeleton.PeekRawNodes().size();
  std::vector<std::vector<size_t>> sorted_segments(maxSegmentCount);

  for (size_t strand_index = 0; strand_index < randomly_subdivided_strands.size(); ++strand_index) {
    auto& uniformly_subdivided_strand = uniformly_subdivided_strand_group.PeekStrand(strand_index);
    Jobs::RunParallelFor(
        uniformly_subdivided_strand.PeekStrandSegmentHandles().size(), [&](const size_t segment_index) {
          size_t segment_handle = uniformly_subdivided_strand.PeekStrandSegmentHandles()[segment_index];
          const auto& segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(segment_handle);
          const auto& segment = uniformly_subdivided_strand_group.PeekStrandSegment(segment_handle);
          // glm::vec3 global_position = segment.end_position;
          // sorted_positions[segment_index].push_back({glm::vec3(segment_data.profile_position, 0.0f),
          // global_position});
          sorted_segments[segment_index].push_back(segment_handle);
        });
  }

  std::vector<glm::mat4> profile_to_model_transforms(maxSegmentCount + 1);

  // create a file for debugging global vs profile positions
  /*std::ofstream debug_file("profile_to_global_debug.csv");
  // create header
  debug_file << "segment_index,p0_profile.x,p0_profile.y,p0_profile.z,p0_global.x,p0_global.y,p0_global.z,"
                "p1_profile.x,p1_profile.y,p1_profile.z,p1_global.x,p1_global.y,p1_global.z,"
                "p2_profile.x,p2_profile.y,p2_profile.z,p2_global.x,p2_global.y,p2_global.z\n";*/

  // we need to treat the first transform separately as it derives from the start points of the first segments
  const auto& strands = uniformly_subdivided_strand_group.PeekStrands();

  auto get_transforms = [&](int h) {
    if (h >= profile_to_model_transforms.size()) {
      EVOENGINE_ERROR("Segment index out of bounds!");
      return;
    }

    const auto& segments = sorted_segments[h == 0 ? 0 : (h - 1)];

    std::function<glm::vec3(size_t)> get_point = [&](size_t idx) {
      size_t segment_handle = segments[idx];
      const auto& segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(segment_handle);
      return glm::vec3(segment_data.profile_position.x, 0.0f, segment_data.profile_position.y);
    };

    std::optional<std::array<size_t, 3>> triple_opt = FindNonCollinearTriple(get_point, segments.size());

    glm::vec3 p0_profile, p1_profile, p2_profile;
    glm::vec3 p0_global, p1_global, p2_global;

    if (!triple_opt.has_value()) {
      // use normal to get a transformation
      // find non-identical pair
      std::optional<std::array<size_t, 2>> pair_opt = FindNonIdenticalPair(get_point, segments.size());

      // get normal from first segment
      StrandSegmentHandle first_segment_handle = segments[0];
      const auto& first_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(first_segment_handle);
      const auto& first_segment = uniformly_subdivided_strand_group.PeekStrandSegment(first_segment_handle);
      const auto& strand = uniformly_subdivided_strand_group.PeekStrand(first_segment.GetStrandHandle());
      StrandSegmentHandle next_segment_handle = first_segment.GetNextHandle();

      float normal_sign = 1.0f;
      glm::vec3 normal_global;

      if (h != 0) {
        // TODO: is this always safe? We could have a strand with only one segment.
        if (next_segment_handle == -1) {
          next_segment_handle = first_segment.GetPrevHandle();
          normal_sign = -1.0f;
        }

        const auto& next_segment = uniformly_subdivided_strand_group.PeekStrandSegment(next_segment_handle);

        normal_global = normal_sign * glm::normalize(next_segment.end_position - first_segment.end_position);
      } else {
        // use strand start position instead
        normal_global = normal_sign * glm::normalize(strand.start_position - first_segment.end_position);
      }

      glm::vec3 u_global;
      glm::vec3 v_global;

      glm::vec3 u_profile;
      glm::vec3 v_profile;

      if (!pair_opt.has_value()) {
        // all points identical, use normal and two arbitrary orthogonal vectors
        u_global = glm::normalize(glm::cross(normal_global, glm::vec3(1.0f, 0.0f, 0.0f)));
        if (glm::length(u_global) < glm::epsilon<float>()) {
          u_global = glm::normalize(glm::cross(normal_global, glm::vec3(0.0f, 0.0f, 1.0f)));
        }
        v_global = glm::normalize(glm::cross(normal_global, u_global));

        if (h != 0) {
          p0_global = first_segment.end_position;
        } else {
          p0_global = strand.start_position;
        }
        p1_global = p0_global + u_global;
        p0_profile = glm::vec3(first_segment_data.profile_position.x, 0.0f, first_segment_data.profile_position.y);
        p1_profile = p0_profile + glm::vec3(1.0f, 0.0f, 0.0f);
      } else {
        const auto& pair = pair_opt.value();
        // use the pair to define u direction
        size_t p0_idx = pair[0];
        size_t p1_idx = pair[1];

        const glm::vec2& p0_profile_2d =
            uniformly_subdivided_strand_group.PeekStrandSegmentData(segments[p0_idx]).profile_position;
        p0_profile = glm::vec3(p0_profile_2d.x, 0.0f, p0_profile_2d.y);

        const glm::vec2& p1_profile_2d =
            uniformly_subdivided_strand_group.PeekStrandSegmentData(segments[p1_idx]).profile_position;
        p1_profile = glm::vec3(p1_profile_2d.x, 0.0f, p1_profile_2d.y);

        if (h != 0) {
          p0_global = uniformly_subdivided_strand_group.PeekStrandSegment(segments[p0_idx]).end_position;
          p1_global = uniformly_subdivided_strand_group.PeekStrandSegment(segments[p1_idx]).end_position;
        } else {
          auto& p0_strand =
              strands[uniformly_subdivided_strand_group.PeekStrandSegment(segments[p0_idx]).GetStrandHandle()];
          auto& p1_strand =
              strands[uniformly_subdivided_strand_group.PeekStrandSegment(segments[p1_idx]).GetStrandHandle()];

          p0_global = p0_strand.start_position;
          p1_global = p1_strand.start_position;
        }

        u_profile = glm::normalize(p1_profile - p0_profile);
        u_global = glm::normalize(p1_global - p0_global);
        v_profile = glm::normalize(glm::cross(normal_global, u_profile));
        v_global = glm::normalize(glm::cross(normal_global, u_global));
      }

      p2_global = p0_global + v_global;
      p2_profile = p0_profile + glm::vec3(0.0f, 0.0f, 1.0f);  // TODO: is this correct?
    } else {
      const auto& triple = triple_opt.value();

      // now get the end positions and profile positions of the three segments
      const glm::vec2 p0_profile_2d =
          uniformly_subdivided_strand_group.PeekStrandSegmentData(segments[triple[0]]).profile_position;
      p0_profile = glm::vec3(p0_profile_2d.x, 0.0f, p0_profile_2d.y);

      const glm::vec2& p1_profile_2d =
          uniformly_subdivided_strand_group.PeekStrandSegmentData(segments[triple[1]]).profile_position;
      p1_profile = glm::vec3(p1_profile_2d.x, 0.0f, p1_profile_2d.y);

      const glm::vec2& p2_profile_2d =
          uniformly_subdivided_strand_group.PeekStrandSegmentData(segments[triple[2]]).profile_position;
      p2_profile = glm::vec3(p2_profile_2d.x, 0.0f, p2_profile_2d.y);

      if (h != 0) {
        p0_global = uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[0]]).end_position;
        p1_global = uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[1]]).end_position;
        p2_global = uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[2]]).end_position;
      } else {
        auto& p0_strand =
            strands[uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[0]]).GetStrandHandle()];
        auto& p1_strand =
            strands[uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[1]]).GetStrandHandle()];
        auto& p2_strand =
            strands[uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[2]]).GetStrandHandle()];
        p0_global = p0_strand.start_position;
        p1_global = p1_strand.start_position;
        p2_global = p2_strand.start_position;
      }
    }

    /*debug_file << segment_index << "," << p0_profile.x << "," << p0_profile.y << "," << p0_profile.z << ","
               << p0_global.x << "," << p0_global.y << "," << p0_global.z << "," << p1_profile.x << "," << p1_profile.y
               << "," << p1_profile.z << "," << p1_global.x << "," << p1_global.y << "," << p1_global.z << ","
               << p2_profile.x << "," << p2_profile.y << "," << p2_profile.z << "," << p2_global.x << "," << p2_global.y
               << "," << p2_global.z << "\n";*/

    profile_to_model_transforms[h] =
        ComputeAffineFromCoplanarPoints(p0_profile, p1_profile, p2_profile, p0_global, p1_global, p2_global);
  };

  Jobs::RunParallelFor(maxSegmentCount + 1, [&](const size_t segment_index) {
    get_transforms(segment_index);
  });

  // Create a branch index lookup using [strand_id][h]
  std::vector<std::vector<size_t>> branch_indices(strand_guide_points.size());
  size_t last_branch_index = 0;
  // Maintain the branches as [h][branch_id][strand_no]
  std::vector<std::vector<std::vector<size_t>>> strands_by_branch_id(maxSegmentCount + 1);

  std::map<SkeletonNodeHandle, size_t> node_to_branch_map;
  for (size_t strand_id = 0; strand_id < strand_guide_points.size(); strand_id++) {
    auto& guide_points = strand_guide_points[strand_id];
    auto node_handle = guide_points.front().node_handle;
    auto it = node_to_branch_map.find(node_handle);
    if (it != node_to_branch_map.end()) {
      size_t branch_index = it->second;
      branch_indices[strand_id].push_back(branch_index);
      strands_by_branch_id[0][branch_index].push_back(strand_id);
    } else {
      size_t branch_index = strands_by_branch_id[0].size();
      node_to_branch_map[node_handle] = branch_index;
      strands_by_branch_id[0].push_back({strand_id});
      branch_indices[strand_id].push_back(branch_index);
    }
  }

  for (size_t h = 1; h < maxSegmentCount + 1; h++) {
    // Now iterate over each branch and check if we need to split it
    strands_by_branch_id[h].resize(strands_by_branch_id[h - 1].size());

    // EVOENGINE_LOG("------------------ Height: " << h)

    for (size_t branch_id = 0; branch_id < strands_by_branch_id[h - 1].size(); branch_id++) {
      auto& branch_strands = strands_by_branch_id[h - 1][branch_id];
      // branch might end early:
      if (branch_strands.empty()) {
        // EVOENGINE_LOG("Branch with id " << branch_id << " ended at height " << h);
        continue;
      }

      SkeletonNodeHandle branch_node = strand_guide_points[branch_strands.front()][h].node_handle;
      std::map<SkeletonNodeHandle, size_t> node_to_branch_map;

      node_to_branch_map[branch_node] = branch_id;

      for (size_t& strand_id : branch_strands) {
        const auto& guide_points = strand_guide_points[strand_id];

        if (h >= guide_points.size()) {
          // EVOENGINE_LOG("Strand " << strand_id << " ended early at height " << h);
          continue;  // strand ends here
        }

        auto node_handle = guide_points[h].node_handle;

        auto it = node_to_branch_map.find(node_handle);
        if (it != node_to_branch_map.end()) {
          size_t branch_index = it->second;
          // EVOENGINE_LOG("strand " << strand_id << " belongs to already found branch " << branch_index);
          branch_indices[strand_id].push_back(branch_index);
          strands_by_branch_id[h][branch_index].push_back(strand_id);
        } else {
          size_t branch_index = strands_by_branch_id[h].size();
          // EVOENGINE_LOG("strand " << strand_id << " belongs to newly discovered branch " << branch_index);
          node_to_branch_map[node_handle] = branch_index;
          strands_by_branch_id[h].push_back({strand_id});
          branch_indices[strand_id].push_back(branch_index);
        }
      }
    }
  }

  // For debugging, output the node index for each height:
  /*for (size_t h = 0; h < maxSegmentCount + 1; h++) {
    std::cout << "Node handles at height " << h << ": ";

    // collect in a set to not list duplicates
    std::set<SkeletonNodeHandle> handles;
    for (size_t strand_id = 0; strand_id < strand_guide_points.size(); strand_id++) {
      auto& guide_points = strand_guide_points[strand_id];
      if (h < guide_points.size()) {
        handles.insert(guide_points[h].node_handle);
      }
    }

    for (auto& handle : handles) {
      std::cout << handle << ", ";
    }
    std::cout << std::endl;
  }

  for (size_t h = 0; h < maxSegmentCount + 1; h++) {
    std::cout << "Branch indices at height " << h << ": ";
    std::set<size_t> index_set;
    for (size_t strand_id = 0; strand_id < strand_guide_points.size(); strand_id++) {
      if (h < branch_indices[strand_id].size()) {
        index_set.insert(branch_indices[strand_id][h]);
      }
    }

    for (auto& index : index_set) {
      std::cout << index << ", ";
    }
    std::cout << std::endl;
  }*/

  // re-order transforms by height and branch
  std::vector<std::vector<glm::mat4>> transforms_by_height_and_branch(maxSegmentCount + 1);

  auto get_branch_transforms = [&](size_t branch_index, int h) {
    if (h >= profile_to_model_transforms.size()) {
      EVOENGINE_ERROR("Segment index out of bounds!");
      return;
    }

    const auto& strand_ids = strands_by_branch_id[h][branch_index];

    if (strand_ids.empty()) {
      return;
    }

    std::function<glm::vec3(size_t)> get_point = [&](size_t idx) {
      size_t strand_id = strand_ids[idx];
      size_t segment_handle = strand_guide_points[strand_id][h].segment_handle;
      const auto& segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(segment_handle);
      return glm::vec3(segment_data.profile_position.x, 0.0f, segment_data.profile_position.y);
    };

    std::optional<std::array<size_t, 3>> triple_opt = FindNonCollinearTriple(get_point, strand_ids.size());

    glm::vec3 p0_profile, p1_profile, p2_profile;
    glm::vec3 p0_global, p1_global, p2_global;

    if (!triple_opt.has_value()) {
      // use normal to get a transformation
      // find non-identical pair
      std::optional<std::array<size_t, 2>> pair_opt = FindNonIdenticalPair(get_point, strand_ids.size());

      // get normal from first segment
      StrandSegmentHandle first_segment_handle = strand_guide_points[strand_ids[0]][h].segment_handle;
      const auto& first_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(first_segment_handle);
      const auto& first_segment = uniformly_subdivided_strand_group.PeekStrandSegment(first_segment_handle);
      const auto& strand = uniformly_subdivided_strand_group.PeekStrand(first_segment.GetStrandHandle());
      StrandSegmentHandle next_segment_handle = first_segment.GetNextHandle();

      float normal_sign = 1.0f;
      glm::vec3 normal_global;

      if (h != 0) {
        // TODO: is this always safe? We could have a strand with only one segment.
        if (next_segment_handle == -1) {
          next_segment_handle = first_segment.GetPrevHandle();
          normal_sign = -1.0f;
        }

        const auto& next_segment = uniformly_subdivided_strand_group.PeekStrandSegment(next_segment_handle);

        normal_global = normal_sign * glm::normalize(next_segment.end_position - first_segment.end_position);
      } else {
        // use strand start position instead
        normal_global = normal_sign * glm::normalize(strand.start_position - first_segment.end_position);
      }

      glm::vec3 u_global;
      glm::vec3 v_global;

      glm::vec3 u_profile;
      glm::vec3 v_profile;

      if (!pair_opt.has_value()) {
        // all points identical, use normal and two arbitrary orthogonal vectors
        u_global = glm::normalize(glm::cross(normal_global, glm::vec3(1.0f, 0.0f, 0.0f)));
        if (glm::length(u_global) < glm::epsilon<float>()) {
          u_global = glm::normalize(glm::cross(normal_global, glm::vec3(0.0f, 0.0f, 1.0f)));
        }
        v_global = glm::normalize(glm::cross(normal_global, u_global));

        if (h != 0) {
          p0_global = first_segment.end_position;
        } else {
          p0_global = strand.start_position;
        }
        p1_global = p0_global + u_global;
        p0_profile = glm::vec3(first_segment_data.profile_position.x, 0.0f, first_segment_data.profile_position.y);
        p1_profile = p0_profile + glm::vec3(1.0f, 0.0f, 0.0f);
      } else {
        const auto& pair = pair_opt.value();
        // use the pair to define u direction
        size_t p0_idx = pair[0];
        size_t p1_idx = pair[1];

        const glm::vec2& p0_profile_2d =
            uniformly_subdivided_strand_group
                .PeekStrandSegmentData(strand_guide_points[strand_ids[p0_idx]][h].segment_handle)
                .profile_position;
        p0_profile = glm::vec3(p0_profile_2d.x, 0.0f, p0_profile_2d.y);

        const glm::vec2& p1_profile_2d =
            uniformly_subdivided_strand_group
                .PeekStrandSegmentData(strand_guide_points[strand_ids[p1_idx]][h].segment_handle)
                .profile_position;
        p1_profile = glm::vec3(p1_profile_2d.x, 0.0f, p1_profile_2d.y);

        if (h != 0) {
          p0_global = uniformly_subdivided_strand_group
                          .PeekStrandSegment(strand_guide_points[strand_ids[p0_idx]][h].segment_handle)
                          .end_position;
          p1_global = uniformly_subdivided_strand_group
                          .PeekStrandSegment(strand_guide_points[strand_ids[p1_idx]][h].segment_handle)
                          .end_position;
        } else {
          auto& p0_strand = strands[uniformly_subdivided_strand_group
                                        .PeekStrandSegment(strand_guide_points[strand_ids[p0_idx]][h].segment_handle)
                                        .GetStrandHandle()];
          auto& p1_strand = strands[uniformly_subdivided_strand_group
                                        .PeekStrandSegment(strand_guide_points[strand_ids[p1_idx]][h].segment_handle)
                                        .GetStrandHandle()];

          p0_global = p0_strand.start_position;
          p1_global = p1_strand.start_position;
        }

        u_profile = glm::normalize(p1_profile - p0_profile);
        u_global = glm::normalize(p1_global - p0_global);
        v_profile = glm::normalize(glm::cross(normal_global, u_profile));
        v_global = glm::normalize(glm::cross(normal_global, u_global));
      }

      p2_global = p0_global + v_global;
      p2_profile = p0_profile + glm::vec3(0.0f, 0.0f, 1.0f);  // TODO: is this correct?
    } else {
      const auto& triple = triple_opt.value();

      // now get the end positions and profile positions of the three segments
      const glm::vec2 p0_profile_2d =
          uniformly_subdivided_strand_group
              .PeekStrandSegmentData(strand_guide_points[strand_ids[triple[0]]][h].segment_handle)
              .profile_position;
      p0_profile = glm::vec3(p0_profile_2d.x, 0.0f, p0_profile_2d.y);

      const glm::vec2& p1_profile_2d =
          uniformly_subdivided_strand_group
              .PeekStrandSegmentData(strand_guide_points[strand_ids[triple[1]]][h].segment_handle)
              .profile_position;
      p1_profile = glm::vec3(p1_profile_2d.x, 0.0f, p1_profile_2d.y);

      const glm::vec2& p2_profile_2d =
          uniformly_subdivided_strand_group
              .PeekStrandSegmentData(strand_guide_points[strand_ids[triple[2]]][h].segment_handle)
              .profile_position;
      p2_profile = glm::vec3(p2_profile_2d.x, 0.0f, p2_profile_2d.y);

      if (h != 0) {
        p0_global = uniformly_subdivided_strand_group
                        .PeekStrandSegment(strand_guide_points[strand_ids[triple[0]]][h].segment_handle)
                        .end_position;
        p1_global = uniformly_subdivided_strand_group
                        .PeekStrandSegment(strand_guide_points[strand_ids[triple[1]]][h].segment_handle)
                        .end_position;
        p2_global = uniformly_subdivided_strand_group
                        .PeekStrandSegment(strand_guide_points[strand_ids[triple[2]]][h].segment_handle)
                        .end_position;
      } else {
        auto& p0_strand = strands[uniformly_subdivided_strand_group
                                      .PeekStrandSegment(strand_guide_points[strand_ids[triple[0]]][h].segment_handle)
                                      .GetStrandHandle()];
        auto& p1_strand = strands[uniformly_subdivided_strand_group
                                      .PeekStrandSegment(strand_guide_points[strand_ids[triple[1]]][h].segment_handle)
                                      .GetStrandHandle()];
        auto& p2_strand = strands[uniformly_subdivided_strand_group
                                      .PeekStrandSegment(strand_guide_points[strand_ids[triple[2]]][h].segment_handle)
                                      .GetStrandHandle()];
        p0_global = p0_strand.start_position;
        p1_global = p1_strand.start_position;
        p2_global = p2_strand.start_position;
      }
    }

    /*debug_file << segment_index << "," << p0_profile.x << "," << p0_profile.y << "," << p0_profile.z << ","
               << p0_global.x << "," << p0_global.y << "," << p0_global.z << "," << p1_profile.x << "," << p1_profile.y
               << "," << p1_profile.z << "," << p1_global.x << "," << p1_global.y << "," << p1_global.z << ","
               << p2_profile.x << "," << p2_profile.y << "," << p2_profile.z << "," << p2_global.x << "," << p2_global.y
               << "," << p2_global.z << "\n";*/

    transforms_by_height_and_branch[h][branch_index] =
        ComputeAffineFromCoplanarPoints(p0_profile, p1_profile, p2_profile, p0_global, p1_global, p2_global);
  };

  Jobs::RunParallelFor(maxSegmentCount + 1, [&](const size_t h) {
    transforms_by_height_and_branch[h].resize(strands_by_branch_id[h].size());
    for (size_t branch_index = 0; branch_index < transforms_by_height_and_branch[h].size(); branch_index++)
      get_branch_transforms(branch_index, h);
  });

  // Proof of concept, just assume we have one trunk with no branches and all strands have the same length
  // construct cubic hermite spline for each strand
  std::vector<std::vector<kinDS::VoronoiPoint<2>>> strand_splines;
  for (const auto& guide_points : strand_guide_points) {
    // extract support points
    std::vector<kinDS::VoronoiPoint<2>> support_points;
    support_points.reserve(guide_points.size());
    for (auto& gp : guide_points) {
      support_points.emplace_back(gp.profile_position);
    }
    strand_splines.push_back(support_points);
  }

  RunMeshingAlgorithm(strand_splines, random_subdivisions_by_strand, randomly_subdivided_segment_handles,
                      transforms_by_height_and_branch, initialize_parameters.root_transform, branch_indices,
                      strands_by_branch_id);
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::InitializationGraphicsPipeline(
    const DynamicStrandsInitializeParameters& initialize_parameters) {
  // Don't need this for now
}

struct VertexPredictionPushConstant {
  uint32_t vertex_count = 0;
  int padding0;
  int padding1;
  int padding2;
};

struct TrianglePredictionPushConstant {
  uint32_t triangle_count = 0;
  int padding0;
  int padding1;
  int padding2;
};

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::BuildRenderComputePipelines() {
  static std::shared_ptr<Shader> shader{};
  shader = std::make_shared<Shader>();
  shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                     std::filesystem::path("./EcoSysLabResources") /
                         "Shaders/Compute/DynamicStrands/Prediction/KineticVoronoiMeshing/Vertex.comp");

  branches_vertex_update_pipeline = std::make_shared<ComputePipeline>();
  branches_vertex_update_pipeline->compute_shader = shader;
  branches_vertex_update_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

  auto& push_constant_range = branches_vertex_update_pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(VertexPredictionPushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  branches_vertex_update_pipeline->Initialize();

  // Triangles
  branches_triangle_update_pipeline = std::make_shared<ComputePipeline>();
  branches_triangle_update_pipeline->compute_shader =
      Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Compute/DynamicStrands/Prediction/KineticVoronoiMeshing/Triangle.comp");
  branches_triangle_update_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

  auto& triangle_prediction_push_constant_range =
      branches_triangle_update_pipeline->push_constant_ranges.emplace_back();
  triangle_prediction_push_constant_range.size = sizeof(TrianglePredictionPushConstant);
  triangle_prediction_push_constant_range.offset = 0;
  triangle_prediction_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  branches_triangle_update_pipeline->Initialize();
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::RenderCompute() const {
  if (dynamic_strands->segments.empty())
    return;
  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    // Vertices
    VertexPredictionPushConstant vertex_push_constant;
    vertex_push_constant.vertex_count = segment_meshlet_vertices.size();
    branches_vertex_update_pipeline->Bind(vk_command_buffer);
    branches_vertex_update_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    branches_vertex_update_pipeline->PushConstant(vk_command_buffer, 0, vertex_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(vertex_push_constant.vertex_count, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    // Triangles
    TrianglePredictionPushConstant triangle_push_constant;
    triangle_push_constant.triangle_count = segment_meshlet_triangles.size();
    branches_triangle_update_pipeline->Bind(vk_command_buffer);
    branches_triangle_update_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    branches_triangle_update_pipeline->PushConstant(vk_command_buffer, 0, triangle_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(triangle_push_constant.triangle_count, work_group_invocations), 1,
                  1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::BuildRenderingPipelines() {
  BuildSegmentMeshletsRenderingPipelines();
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::Download() {
  if (!segment_meshlet_vertices.empty()) {
    device_segment_meshlet_vertices_buffer->DownloadVector(segment_meshlet_vertices, segment_meshlet_vertices.size());
  }
  if (!segment_meshlet_triangles.empty()) {
    device_segment_meshlet_triangles_buffer->DownloadVector(segment_meshlet_triangles,
                                                            segment_meshlet_triangles.size());
  }
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::Upload() {
  device_segment_meshlet_vertices_buffer->UploadVector(segment_meshlet_vertices);
  device_segment_meshlet_vertices_buffer->SetDebugName("Segment Meshlet Vertices Buffer");
  device_segment_meshlet_triangles_buffer->UploadVector(segment_meshlet_triangles);
  device_segment_meshlet_triangles_buffer->SetDebugName("Segment Meshlet Triangles Buffer");
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::Clear() {
  segment_meshlet_vertices.clear();
  segment_meshlet_triangles.clear();
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::UpdateBindings() const {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  dynamic_strands->strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
      8, device_segment_meshlet_vertices_buffer, 0);
  dynamic_strands->strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
      9, device_segment_meshlet_triangles_buffer, 0);
}

bool eco_sys_lab_plugin::DsKineticVoronoiMeshing::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  FileUtils::SaveFile(
      "Download and export PLY", "PLY", {".ply"},
      [&](const std::filesystem::path& path) {
        dynamic_strands->Download();
        EVOENGINE_LOG("Downloaded data from GPU");
        PlyExporter::ExportAscii(path, segment_meshlet_vertices, segment_meshlet_triangles,
                                 render_settings.segment_meshlet_render_parameters.uv_height_factor,
                                 render_settings.segment_meshlet_render_parameters.uv_circum_factor);
      },
      false);
  ImGui::SameLine();
  FileUtils::SaveFile(
      "Export PLY", "PLY", {".ply"},
      [&](const std::filesystem::path& path) {
        PlyExporter::ExportAscii(path, segment_meshlet_vertices, segment_meshlet_triangles,
                                 render_settings.segment_meshlet_render_parameters.uv_height_factor,
                                 render_settings.segment_meshlet_render_parameters.uv_circum_factor);
      },
      false);

  FileUtils::SaveFile(
      "Download and export OBJ", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        dynamic_strands->Download();
        EVOENGINE_LOG("Downloaded data from GPU");
        ObjExporter::ExportObj(path, segment_meshlet_vertices, segment_meshlet_triangles, dynamic_strands->segments,
                               render_settings.segment_meshlet_render_parameters.uv_height_factor,
                               render_settings.segment_meshlet_render_parameters.uv_circum_factor,
                               render_settings.segment_meshlet_render_parameters.fracture_distance);
      },
      false);
  ImGui::SameLine();
  FileUtils::SaveFile(
      "Export OBJ", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        ObjExporter::ExportObj(path, segment_meshlet_vertices, segment_meshlet_triangles, dynamic_strands->segments,
                               render_settings.segment_meshlet_render_parameters.uv_height_factor,
                               render_settings.segment_meshlet_render_parameters.uv_circum_factor,
                               render_settings.segment_meshlet_render_parameters.fracture_distance);
      },
      false);

  return false;
}

void DsKineticVoronoiMeshing::Stats(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Text((std::string("Segment Meshlets Vertices: ") + std::to_string(segment_meshlet_vertices.size())).c_str());
  ImGui::Text((std::string("Segment Meshlets Triangles: ") + std::to_string(segment_meshlet_triangles.size())).c_str());
}

void DsKineticVoronoiMeshing::OnInspectRenderSettings(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Checkbox("Render Segment Meshlets", &render_settings.segment_meshlet_render_parameters.enabled);
  if (render_settings.segment_meshlet_render_parameters.enabled) {
    if (ImGui::Button("Rebuild segment meshlet pipelines")) {
      BuildSegmentMeshletsRenderingPipelines();
    }

    ImGui::Combo("Color mode", {"Standard", "Normals", "UVs", "Pair"},
                 render_settings.segment_meshlet_render_parameters.color_mode);

    // uv factors
    ImGui::DragFloat("UV height factor", &render_settings.segment_meshlet_render_parameters.uv_height_factor, 0.001f,
                     0.001f, 1.0f);
    ImGui::DragFloat("UV circum factor", &render_settings.segment_meshlet_render_parameters.uv_circum_factor, 1.0f,
                     1.0f, 50.0f, "%.0f");

    ImGui::DragFloat("Fracture distance", &render_settings.segment_meshlet_render_parameters.fracture_distance, 0.0001f,
                     0.0f, 2.0f, "%.4f");
  }
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::RegisterRenderInstances(Handle& rendering_instance_handle,
                                                                          std::shared_ptr<Scene> scene, Entity& owner) {
  // TODO: add settings
  RegisterSegmentMeshletsRenderInstance(rendering_instance_handle, scene, owner);
}

void DsKineticVoronoiMeshing::RegisterSegmentMeshletsRenderInstance(Handle& rendering_instance_handle,
                                                                    std::shared_ptr<Scene> scene, Entity& owner) {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  const auto inner_wood_material = dynamic_strands->materials.inner_wood_material_ref.Get<Material>();
  const auto snow_material = dynamic_strands->materials.snow_material_ref.Get<Material>();
  if (const auto bark_material = dynamic_strands->materials.bark_material_ref.Get<Material>();
      bark_material && inner_wood_material && snow_material) {
    if (!dynamic_strands->segments.empty()) {
      if (segment_meshlet_point_light_render_pipeline && segment_meshlet_point_light_render_pipeline->Initialized()) {
        render_layer->RenderToPointLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSegmentMeshletsToPointLightShadowMap(render_settings.segment_meshlet_render_parameters,
                                                            vk_command_buffer, view);
        });
      }
      if (segment_meshlet_spot_light_render_pipeline && segment_meshlet_spot_light_render_pipeline->Initialized()) {
        render_layer->RenderToSpotLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSegmentMeshletsToSpotLightShadowMap(render_settings.segment_meshlet_render_parameters,
                                                           vk_command_buffer, view);
        });
      }
      if (segment_meshlet_directional_light_render_pipeline &&
          segment_meshlet_directional_light_render_pipeline->Initialized()) {
        render_layer->RenderToDirectionalLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSegmentMeshletsToDirectionalLightShadowMap(render_settings.segment_meshlet_render_parameters,
                                                                  vk_command_buffer, view);
        });
      }
      if (segment_meshlet_render_pipeline && segment_meshlet_render_pipeline->Initialized()) {
        const auto current_render_storage = Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
        const auto renderer_handle = rendering_instance_handle;
        int bark_material_index = -1;
        current_render_storage->RegisterRenderInstance(scene, owner, renderer_handle, bark_material,
                                                       &bark_material_index);
        const auto inner_material_index = current_render_storage->RegisterMaterial(inner_wood_material);
        const auto snow_material_index = current_render_storage->RegisterMaterial(snow_material);
        render_layer->DeferredRenderingAllCameras(
            [=](const VkCommandBuffer vk_command_buffer,
                const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                const RenderLayer::DeferredRenderingView& view) {
              return RenderSegmentMeshletsToCameraDeferred(
                  renderer_handle, bark_material_index, inner_material_index, snow_material_index,
                  render_settings.segment_meshlet_render_parameters, vk_command_buffer,
                  geometry_pass_color_attachment_infos, view, VK_POLYGON_MODE_FILL);
            });
      }
    }
  }
}

void eco_sys_lab_plugin::DsKineticVoronoiMeshing::Visualize(
    const std::shared_ptr<Camera>& target_camera, const DynamicStrandsInitializeParameters& initialize_parameters,
    const DynamicStrandsVisualizationParameters& visualization_parameters) {
  // TODO
}

void DsKineticVoronoiMeshing::BuildSegmentMeshletsRenderingPipelines() {
  segment_meshlet_point_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  segment_meshlet_point_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Task/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet.task");
  segment_meshlet_point_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/"
                                  "PointLightShadowMap.mesh");
  segment_meshlet_point_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  segment_meshlet_point_light_render_pipeline->geometry_type = GeometryType::Mesh;
  segment_meshlet_point_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  segment_meshlet_point_light_render_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  segment_meshlet_point_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  segment_meshlet_point_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& point_light_push_constant_range =
      segment_meshlet_point_light_render_pipeline->push_constant_ranges.emplace_back();
  point_light_push_constant_range.size = sizeof(SegmentMeshletPushConstant);
  point_light_push_constant_range.offset = 0;
  point_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  segment_meshlet_point_light_render_pipeline->Initialize();
  // Descriptor set layout
  segment_meshlet_spot_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  segment_meshlet_spot_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Task/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet.task");

  segment_meshlet_spot_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/"
                                  "SpotLightShadowMap.mesh");
  segment_meshlet_spot_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  segment_meshlet_spot_light_render_pipeline->geometry_type = GeometryType::Mesh;
  segment_meshlet_spot_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  segment_meshlet_spot_light_render_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  segment_meshlet_spot_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  segment_meshlet_spot_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& spot_light_push_constant_range =
      segment_meshlet_spot_light_render_pipeline->push_constant_ranges.emplace_back();
  spot_light_push_constant_range.size = sizeof(SegmentMeshletPushConstant);
  spot_light_push_constant_range.offset = 0;
  spot_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  segment_meshlet_spot_light_render_pipeline->Initialize();
  // Descriptor set layout
  segment_meshlet_directional_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  segment_meshlet_directional_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Task/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet.task");

  // TODO: fix path
  segment_meshlet_directional_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/"
                                  "DirectionalLightShadowMap.mesh");
  segment_meshlet_directional_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  segment_meshlet_directional_light_render_pipeline->geometry_type = GeometryType::Mesh;
  segment_meshlet_directional_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  segment_meshlet_directional_light_render_pipeline->descriptor_set_layouts.emplace_back(
      DynamicStrands::strands_layout);
  segment_meshlet_directional_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  segment_meshlet_directional_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& directional_light_push_constant_range =
      segment_meshlet_directional_light_render_pipeline->push_constant_ranges.emplace_back();
  directional_light_push_constant_range.size = sizeof(SegmentMeshletPushConstant);
  directional_light_push_constant_range.offset = 0;
  directional_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  segment_meshlet_directional_light_render_pipeline->Initialize();
  // Descriptor set layout
  segment_meshlet_render_pipeline = std::make_shared<GraphicsPipeline>();
  segment_meshlet_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Task/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet.task");
  segment_meshlet_render_pipeline->mesh_shader = Shader::CreateTemporary(
      ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/Rendering.mesh");
  segment_meshlet_render_pipeline->fragment_shader = Shader::CreateTemporary(
      ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Fragment/DynamicStrands/Rendering/KineticVoronoiMeshing/Branches.frag");
  segment_meshlet_render_pipeline->geometry_type = GeometryType::Mesh;
  segment_meshlet_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  segment_meshlet_render_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  segment_meshlet_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::lighting_layout);
  segment_meshlet_render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
  segment_meshlet_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  segment_meshlet_render_pipeline->color_attachment_formats = {2, Platform::Constants::g_buffer_color};
  auto& push_constant_range = segment_meshlet_render_pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(SegmentMeshletPushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  segment_meshlet_render_pipeline->Initialize();
}

uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletsToPointLightShadowMap(
    const SegmentMeshletsRenderParameters& render_parameters, const VkCommandBuffer vk_command_buffer,
    const RenderLayer::PointLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  SegmentMeshletPushConstant push_constant;
  push_constant.index1.sub_light_index = view.face_index;
  push_constant.index2.light_index = view.light_index;
  push_constant.vertex_count = segment_meshlet_vertices.size();
  push_constant.triangle_count = segment_meshlet_triangles.size();
  push_constant.color_mode = render_settings.segment_meshlet_render_parameters.color_mode;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  segment_meshlet_point_light_render_pipeline->Bind(vk_command_buffer);
  segment_meshlet_point_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  segment_meshlet_point_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_point_light_render_pipeline->states.ResetAllStates(0);
  segment_meshlet_point_light_render_pipeline->states.SetViewportScissor(view.viewport);
  segment_meshlet_point_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  segment_meshlet_point_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(segment_meshlet_triangles.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return segment_meshlet_triangles.size();
}

uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletsToSpotLightShadowMap(
    const SegmentMeshletsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const RenderLayer::SpotLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  SegmentMeshletPushConstant push_constant;
  push_constant.index1.sub_light_index = 0;
  push_constant.index2.light_index = view.light_index;
  push_constant.vertex_count = segment_meshlet_vertices.size();
  push_constant.triangle_count = segment_meshlet_triangles.size();
  push_constant.color_mode = render_settings.segment_meshlet_render_parameters.color_mode;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  segment_meshlet_spot_light_render_pipeline->Bind(vk_command_buffer);
  segment_meshlet_spot_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  segment_meshlet_spot_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_spot_light_render_pipeline->states.ResetAllStates(0);
  segment_meshlet_spot_light_render_pipeline->states.SetViewportScissor(view.viewport);
  segment_meshlet_spot_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  segment_meshlet_spot_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(segment_meshlet_triangles.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return segment_meshlet_triangles.size();
}

uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletsToDirectionalLightShadowMap(
    const SegmentMeshletsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const RenderLayer::DirectionalLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  SegmentMeshletPushConstant push_constant;
  push_constant.index1.sub_light_index = view.split_index;
  push_constant.index2.light_index = view.light_index;
  push_constant.vertex_count = segment_meshlet_vertices.size();
  push_constant.triangle_count = segment_meshlet_triangles.size();
  push_constant.color_mode = render_settings.segment_meshlet_render_parameters.color_mode;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  segment_meshlet_directional_light_render_pipeline->Bind(vk_command_buffer);
  segment_meshlet_directional_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  segment_meshlet_directional_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_directional_light_render_pipeline->states.ResetAllStates(0);
  segment_meshlet_directional_light_render_pipeline->states.SetViewportScissor(view.viewport);
  segment_meshlet_directional_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  segment_meshlet_directional_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(segment_meshlet_triangles.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return segment_meshlet_triangles.size();
}

uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletsToCameraDeferred(
    const Handle& renderer_handle, int bark_material_index, int inner_wood_material_index, int snow_material_index,
    const SegmentMeshletsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
    const RenderLayer::DeferredRenderingView& view, VkPolygonMode polygon_mode) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  if (!Platform::Constants::support_mesh_shader) {
    EVOENGINE_LOG("Failed to render! Mesh shader unsupported!")
    return 0;
  }

  // TODO: If we add any compute shaders, also check them here
  if (!segment_meshlet_render_pipeline || !segment_meshlet_render_pipeline->Initialized()) {
    return 0;
  }
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  SegmentMeshletPushConstant render_push_constant;
  render_push_constant.index1.instance_index =
      Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage()->GetRenderInstanceIndex(renderer_handle);
  render_push_constant.index2.camera_index = view.camera_index;
  render_push_constant.vertex_count = segment_meshlet_vertices.size();
  render_push_constant.triangle_count = segment_meshlet_triangles.size();
  render_push_constant.color_mode = render_settings.segment_meshlet_render_parameters.color_mode;
  render_push_constant.inner_wood_material_index = inner_wood_material_index;
  render_push_constant.bark_material_index = bark_material_index;
  render_push_constant.uv_height_factor = render_settings.segment_meshlet_render_parameters.uv_height_factor;
  render_push_constant.uv_circum_factor = render_settings.segment_meshlet_render_parameters.uv_circum_factor;
  render_push_constant.fracture_distance = render_settings.segment_meshlet_render_parameters.fracture_distance;

  segment_meshlet_render_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
  segment_meshlet_render_pipeline->states.SetViewportScissor(view.viewport);
  segment_meshlet_render_pipeline->states.polygon_mode = polygon_mode;
  segment_meshlet_render_pipeline->states.line_width = 2.0f;
  segment_meshlet_render_pipeline->states.ApplyAllStates(vk_command_buffer);

#ifdef USE_RENDERDOC
  if (rdoc_api) {
    rdoc_api->StartFrameCapture(NULL, NULL);
    EVOENGINE_LOG("RDOC API detected!");
  }
#endif  //  USERENDERDOC

  segment_meshlet_render_pipeline->Bind(vk_command_buffer);
  segment_meshlet_render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                     RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  segment_meshlet_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_render_pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                                     RenderLayer::GetLightingDescriptorSet()->GetVkDescriptorSet());

  segment_meshlet_render_pipeline->PushConstant(vk_command_buffer, 0, render_push_constant);

  const uint32_t count = Platform::DivUp(segment_meshlet_triangles.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
#ifdef USE_RENDERDOC
  if (rdoc_api)
    rdoc_api->EndFrameCapture(NULL, NULL);
#endif
  return dynamic_strands->segments.size();
}

/* uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletVisualizationToCameraDeferred(
    const Handle& renderer_handle, const DynamicStrandsInitializeParameters& initialize_parameters,
    const SmallSegmentsVisualizationRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
    const RenderLayer::DeferredRenderingView& view) const {
}*/