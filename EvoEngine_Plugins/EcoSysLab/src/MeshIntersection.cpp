#include "MeshIntersection.hpp"

using namespace kinDS;

#ifdef USE_CGAL
#  include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#  include <CGAL/Polygon_mesh_processing/corefinement.h>  // for corefine_and_compute_intersection()
#  include <CGAL/Polygon_mesh_processing/measure.h>       // optional area/volume utils
#  include <CGAL/Polygon_mesh_processing/orientation.h>   // for orient_to_bound_a_volume()
#  include <CGAL/Polygon_mesh_processing/repair.h>        // for merge_duplicate_vertices(), remove_isolated_vertices()
#  include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>  // for remove_degenerate_faces()
#  include <CGAL/Polygon_mesh_processing/stitch_borders.h>       // for stitch_borders()
#  include <CGAL/Polygon_mesh_processing/triangulate_faces.h>    // (optional) triangulate_face if needed
#  include <CGAL/Surface_mesh.h>
#  include <array>
#  include <iostream>
#  include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using PointCGAL = Kernel::Point_3;
using MeshCGAL = CGAL::Surface_mesh<PointCGAL>;

// Convert from std::vector-based mesh data to CGAL Surface_mesh
static void vectorToCgalMesh(const std::vector<std::array<double, 3>>& vertices,
                             const std::vector<std::array<size_t, 3>>& triangles, MeshCGAL& mesh) {
  std::vector<MeshCGAL::Vertex_index> vmap(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    vmap[i] = mesh.add_vertex(PointCGAL(vertices[i][0], vertices[i][1], vertices[i][2]));
  }

  for (const auto& t : triangles)
    mesh.add_face(vmap[t[0]], vmap[t[1]], vmap[t[2]]);
}

static void voronoiMeshToCgalMesh(const Mesh& input_mesh, MeshCGAL& output_mesh) {
  auto& vertices = input_mesh.getVertices();
  std::vector<MeshCGAL::Vertex_index> vmap(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    vmap[i] = output_mesh.add_vertex(PointCGAL(vertices[i][0], vertices[i][1], vertices[i][2]));
  }

  auto& vertex_indices = input_mesh.getVertexIndices();
  for (size_t i = 0; i < vertex_indices.size(); i += 3) {
    output_mesh.add_face(vmap[vertex_indices[i]], vmap[vertex_indices[i + 1]], vmap[vertex_indices[i + 2]]);
  }
}
#endif

Mesh MeshIntersection::intersect(const Mesh& meshA, const Mesh& meshB) {
  Mesh meshIntersection;

  if (meshA.getVertexIndices().empty() || meshB.getVertexIndices().empty()) {
    EVOENGINE_WARNING("One of the input meshes is empty. Returning empty intersection mesh.");
    return meshIntersection;  // empty mesh
  }

#ifdef USE_CGAL
  MeshCGAL cgalMeshA, cgalMeshB, cgalMeshIntersection;
  voronoiMeshToCgalMesh(meshA, cgalMeshA);
  voronoiMeshToCgalMesh(meshB, cgalMeshB);

  PMP::orient_to_bound_a_volume(cgalMeshA);
  PMP::orient_to_bound_a_volume(cgalMeshB);

  bool success = PMP::corefine_and_compute_intersection(cgalMeshA, cgalMeshB, cgalMeshIntersection);

  if (!success) {
    std::cerr << "Intersection failed - make sure both meshes are closed.\n";
    return meshIntersection;  // empty mesh
  }
  // Convert back to VoronoiMesh
  std::vector<std::array<double, 3>> vertices;
  std::vector<std::array<size_t, 3>> triangles;

  for (const auto& v : cgalMeshIntersection.vertices()) {
    const PointCGAL& p = cgalMeshIntersection.point(v);
    meshIntersection.addVertex(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
  }

  for (const auto& f : cgalMeshIntersection.faces()) {
    std::array<size_t, 3> tri;
    size_t idx = 0;
    for (const auto& v : CGAL::vertices_around_face(cgalMeshIntersection.halfedge(f), cgalMeshIntersection)) {
      tri[idx++] = static_cast<size_t>(v);
    }
    meshIntersection.addTriangle(tri[0], tri[1], tri[2]);
  }
#endif

  return meshIntersection;
}
