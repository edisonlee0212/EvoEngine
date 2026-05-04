#include "MeshIntersection.hpp"

using namespace kinDS;

#include <array>
#include <iostream>
#include <string>
#include <vector>

#ifdef USE_CGAL
#  include <CGAL/Intersection_traits_3.h>
#  include <CGAL/Side_of_triangle_mesh.h>
#  include <CGAL/boost/graph/split_graph_into_polylines.h>
#  include <CGAL/intersections.h>  // triangle�triangle intersection
#  include <CGAL/version.h>

using SideTest = CGAL::Side_of_triangle_mesh<MeshCGAL_internal, Kernel>;

void debug_print_face_index_map(const MeshCGAL<size_t>& mesh, const std::string& name) {
  using FaceDescriptor = MeshCGAL_internal::Face_index;

  std::cout << "=== Face index map dump: " << name << " ===\n";

  std::size_t count = 0;
  for (FaceDescriptor f : faces(mesh.mesh)) {
    std::cout << "  Face " << count << " --> fmap[f] = " << mesh.fidx[f] << "\n";
    ++count;
  }

  std::cout << "Total faces: " << count << "\n";
  std::cout << "=== End dump ===\n";
}

void debug_print_face_origin_map(const MeshCGAL<Origin>& mesh, const std::string& name) {
  using FaceDescriptor = MeshCGAL_internal::Face_index;

  std::cout << "=== Face origin map dump: " << name << " ===\n";

  std::size_t count = 0;
  for (FaceDescriptor f : faces(mesh.mesh)) {
    auto val = mesh.fidx[f];
    // val is std::pair<int, size_t>

    std::cout << "  Face " << count << " --> origin = { mesh_id = " << val.mesh_index << ", face_idx = " << val.face_id
              << " }\n";

    ++count;
  }

  std::cout << "Total faces: " << count << "\n";
  std::cout << "=== End dump ===\n";
}

// Convert from std::vector-based mesh data to CGAL Surface_mesh
static MeshCGAL_internal vectorToCgalMesh(const std::vector<std::array<double, 3>>& vertices,
                                          const std::vector<std::array<size_t, 3>>& triangles) {
  MeshCGAL_internal mesh;
  std::vector<MeshCGAL_internal::Vertex_index> vmap(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    vmap[i] = mesh.add_vertex(Point_3(vertices[i][0], vertices[i][1], vertices[i][2]));
  }

  for (const auto& t : triangles)
    mesh.add_face(vmap[t[0]], vmap[t[1]], vmap[t[2]]);

  return mesh;
}

static MeshCGAL<Origin> voronoiMeshToCgalMesh(const VoronoiMesh& input_mesh, const std::vector<int>& neighbor_segments,
                                              int mesh_id = -1) {
  MeshCGAL<Origin> output_mesh("f:origin", Origin{-1, 0});

  auto& vertices = input_mesh.getVertices();
  std::vector<MeshCGAL_internal::Vertex_index> vmap(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    vmap[i] = output_mesh.mesh.add_vertex(Point_3(vertices[i][0], vertices[i][1], vertices[i][2]));
  }

  auto& triangles = input_mesh.getTriangles();
  for (size_t i = 0; i < triangles.size(); i += 3) {
    auto face_index = output_mesh.mesh.add_face(vmap[triangles[i]], vmap[triangles[i + 1]], vmap[triangles[i + 2]]);

    if (output_mesh.mesh.is_valid(face_index)) {
      if (!neighbor_segments.empty()) {
        output_mesh.fidx[face_index] = {mesh_id, i / 3};
      }
    } else {
      // usually not relevant, but can be commented in if any issues arise
      // EVOENGINE_WARNING("Adding triangle no. " << (i / 3) << " failed, will be ignored. It is probably degenerate.");
    }
  }

  return output_mesh;
}

static double eps = 1e-12;

kinDS::MeshIntersection::MeshIntersection(const VoronoiMesh& static_mesh) : boundary_mesh_voronoi(static_mesh) {
  // All neighbor segments are -1
  std::vector<int> neighbor_segments(static_mesh.getTriangleCount(), -1);

  boundary_mesh = voronoiMeshToCgalMesh(static_mesh, neighbor_segments, 0);

  // assume that this is already the case and omit this call
  // PMP::orient_to_bound_a_volume(boundary_mesh);
  tree = TreeCGAL(faces(boundary_mesh.mesh).first, faces(boundary_mesh.mesh).second, boundary_mesh.mesh);
  tree.build();
  tree.accelerate_distance_queries();
}
#else
kinDS::MeshIntersection::MeshIntersection(const VoronoiMesh& static_mesh) {
  EVOENGINE_ERROR("CGAL was not found, acceleration data structure could not be constructed!");
}
#endif

void interpolateProperties(const VoronoiMesh& original_mesh, VoronoiMesh& new_mesh, size_t original_face_id,
                           std::array<size_t, 3> new_tri) {
  std::array<size_t, 3> old_tri;
  std::array<Vector<3>, 3> old_normals;

  bool interpolate_uv = true;
  for (size_t i = 0; i < 3; i++) {
    old_tri[i] = original_mesh.getTriangles()[3 * original_face_id + i];
    old_normals[i] = original_mesh.getNormal(3 * original_face_id + i);
    // EVOENGINE_LOG("Old tri id " << i << ": " << old_tri[i]);
    if (original_mesh.getUVIndices()[old_tri[i]] >= original_mesh.getUVs().size()) {
      interpolate_uv = false;
    }
  }

  // EVOENGINE_LOG("interpolate_uv: " << interpolate_uv);
  if (interpolate_uv) {
    new_mesh.getUVIndices().resize(new_mesh.getTriangleCount() * 3, -1);
  }

  // compute the barycentric coordinates of the new face with regard to the new one so we can interpolate properties
  for (size_t i = 0; i < 3; i++) {
    auto barycentric_coords =
        original_mesh.computeBarycentricCoordinates(original_face_id, new_mesh.getVertices()[new_tri[i]]);

    // compute new normals and UVs by interpolating from old mesh
    Vector<3> interpolated_normal = barycentric_coords[0] * old_normals[0] + barycentric_coords[1] * old_normals[1] +
                                    barycentric_coords[2] * old_normals[2];
    size_t normal_index = new_mesh.addNormal(interpolated_normal);

    if (interpolate_uv) {
      auto interpolated_uv =
          barycentric_coords[0] * original_mesh.getUVs()[original_mesh.getUVIndices()[3 * original_face_id]] +
          barycentric_coords[1] * original_mesh.getUVs()[original_mesh.getUVIndices()[3 * original_face_id + 1]] +
          barycentric_coords[2] * original_mesh.getUVs()[original_mesh.getUVIndices()[3 * original_face_id + 2]];

      size_t index = new_mesh.addUV(interpolated_uv);
      new_mesh.getUVIndices()[new_mesh.getTriangleCount() * 3 - 3 + i] = index;
    }
  }
}

std::pair<VoronoiMesh, std::vector<int>> MeshIntersection::Intersect(const VoronoiMesh& mesh,
                                                                     const std::vector<int>& neighbor_segments) {
  std::pair<VoronoiMesh, std::vector<int>> ret_val;
  auto& [intersection_mesh, out_neighbor_segments] = ret_val;

  if (mesh.getTriangles().empty()) {
    EVOENGINE_WARNING("The input is empty. Returning empty intersection mesh.");
    return ret_val;  // empty mesh
  }

#ifdef USE_CGAL

  MeshCGAL<Origin> input_mesh = voronoiMeshToCgalMesh(mesh, neighbor_segments, 1);
  // we need a copy because the corefinement is destructive
  MeshCGAL<Origin> boundary_mesh_copy = boundary_mesh;
  MeshCGAL<Origin> output_mesh("f:origin", {-1, 0});
  //  assume that this is already the case and omit this call
  //  PMP::orient_to_bound_a_volume(input_mesh);

  RecordingVisitor visitor(boundary_mesh_copy, input_mesh, output_mesh);

  bool success = PMP::corefine_and_compute_intersection(boundary_mesh_copy.mesh, input_mesh.mesh, output_mesh.mesh,
                                                        PMP::parameters::visitor(visitor));

  if (!success) {
    EVOENGINE_ERROR("Intersection failed - make sure both meshes are closed.");
    return ret_val;  // empty mesh
  }

  for (const auto& v : output_mesh.mesh.vertices()) {
    const Point_3& p = output_mesh.mesh.point(v);
    intersection_mesh.addVertex(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
  }

  for (const auto& f : output_mesh.mesh.faces()) {
    std::array<size_t, 3> tri;
    size_t idx = 0;
    for (const auto& v : CGAL::vertices_around_face(output_mesh.mesh.halfedge(f), output_mesh.mesh)) {
      tri[idx++] = static_cast<size_t>(v);
    }
    size_t new_triangle_index = intersection_mesh.addTriangle(tri[0], tri[1], tri[2]);

    // Get neighbor segment from face property map
    // -2 is default and corresponds to the bark boundary. We need this because -1 corresponds to open segments, which
    // can be thought of as either cut off or connecting to the soil.
    int neighbor_segment = -2;

    auto origin = output_mesh.fidx[f];
    if (origin.mesh_index == 1) {
      if (origin.face_id < neighbor_segments.size()) {
        neighbor_segment = neighbor_segments[origin.face_id];
      } else {
        EVOENGINE_WARNING("Invalid origin triangle index: " << origin.face_id)
      }

      interpolateProperties(mesh, intersection_mesh, origin.face_id, tri);
    } else if (origin.mesh_index == 0) {
      // we keep the default value for the neighbor as the boundary mesh has no neighbors
      interpolateProperties(boundary_mesh_voronoi, intersection_mesh, origin.face_id, tri);
    } else {
      EVOENGINE_WARNING("Invalid origin mesh index: " << origin.mesh_index);
    }

    out_neighbor_segments.push_back(neighbor_segment);
  }
#else
  EVOENGINE_ERROR(
      "CGAL is required for the intersection computation but was not found. Returning empty intersection mesh.");
#endif

  return ret_val;
}

kinDS::MeshIntersection::MeshRelation kinDS::MeshIntersection::ClassifyMeshRelation(const VoronoiMesh& mesh,
                                                                                    bool assume_inside) {
#ifdef USE_CGAL
  // --- Side-of-mesh using the same tree (no rebuild) ---
  SideTest side(tree);

  bool any_inside = false;
  bool any_outside = false;

  // ---------- 1. Triangle-M0 intersection using the tree ----------

  for (size_t i = 0; i < mesh.getTriangles().size(); i += 3) {
    const Point<3>& p0 = mesh.getVertices()[mesh.getTriangles()[i]];
    Kernel::Point_3 p0_cgal(p0[0], p0[1], p0[2]);
    const Point<3>& p1 = mesh.getVertices()[mesh.getTriangles()[i + 1]];
    Kernel::Point_3 p1_cgal(p1[0], p1[1], p1[2]);
    const Point<3>& p2 = mesh.getVertices()[mesh.getTriangles()[i + 2]];
    Kernel::Point_3 p2_cgal(p2[0], p2[1], p2[2]);

    CGAL::Triangle_3<Kernel> tri(p0_cgal, p1_cgal, p2_cgal);
    // This performs triangle vs. all boundary_mesh triangles intersection test via the tree
    if (tree.do_intersect(tri)) {
      return MeshRelation::INTERSECTING;
    }
  }

  if (assume_inside) {
    return MeshRelation::INSIDE;
  }

  // ---------- 2. No intersections --> classify inside/outside ----------
  for (size_t i = 0; i < mesh.getTriangles().size(); i += 3) {
    const Point<3>& p = mesh.getVertices()[mesh.getTriangles()[i]];
    Kernel::Point_3 p_cgal(p[0], p[1], p[2]);

    CGAL::Bounded_side bs = side(p_cgal);

    if (bs == CGAL::ON_BOUNDARY)
      return MeshRelation::INTERSECTING;  // touching boundary = intersecting

    if (bs == CGAL::ON_BOUNDED_SIDE)
      any_inside = true;
    else
      any_outside = true;

    // If both occur, surface must cross the boundary (even without intersection)
    if (any_inside && any_outside)
      return MeshRelation::INTERSECTING;
  }

  if (any_inside)
    return MeshRelation::INSIDE;
  return MeshRelation::OUTSIDE;
#else
  EVOENGINE_ERROR("CGAL was not found, cannot determine mesh relation!");
  return MeshRelation::UNDEFINED;
#endif
}