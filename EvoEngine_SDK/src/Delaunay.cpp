#include "Delaunay.hpp"
#include "Mesh.hpp"
#include "Jobs.hpp"
#include "ProjectManager.hpp"
#include "geogram/basic/psm.h"
#include "geogram/delaunay/parallel_delaunay_3d.h"
using namespace evo_engine;

GEO::Delaunay_var delaunay{};

void ProcessDelauney3D(const bool keeps_infinite, const std::vector<glm::vec3>& points) {
  std::vector<double> converted_points(points.size() * 3);
  Jobs::RunParallelFor(points.size(), [&](const auto i) {
    converted_points[i * 3] = points[i].x;
    converted_points[i * 3 + 1] = points[i].y;
    converted_points[i * 3 + 2] = points[i].z;
  });
  if (!delaunay.get())
    delaunay = GEO::Delaunay::create(3, "PDEL");

  // If we want the convex hull, we keep the infinite facets,
  // because the convex hull can be retrieved as the finite facets
  // of the infinite cells (note: it would be also possible to
  // throw away the infinite cells and get the convex hull as
  // the facets adjacent to no cell).
  delaunay->set_keeps_infinite(keeps_infinite);
  delaunay->set_vertices(static_cast<GEO::index_t>(points.size()), converted_points.data());
}
std::shared_ptr<Mesh> GenerateMesh(std::vector<glm::uvec3>& triangles, const std::vector<glm::vec3>& points) {
  std::unordered_map<GEO::index_t, unsigned> vertices_map;
  std::vector<Vertex> vertices;
  Vertex archetype{};
  for (auto& triangle : triangles) {
    for (uint32_t i = 0; i < 3; i++) {
      auto& v = triangle[i];
      if (const auto search = vertices_map.find(v); search == vertices_map.end()) {
        const unsigned real_v = static_cast<unsigned>(vertices.size());
        vertices_map[v] = real_v;
        archetype.position = points[v];
        vertices.emplace_back(archetype);
        v = real_v;
      } else {
        v = search->second;
      }
    }
  }
  const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
  VertexAttributes attributes{};
  mesh->SetVertices(attributes, vertices, triangles);
  return mesh;
}

std::vector<glm::uvec3> Delaunay3D::GenerateConvexHullTriangles(const std::vector<glm::vec3>& points) {
  ProcessDelauney3D(true, points);

  const auto triangle_count = delaunay->nb_cells() - delaunay->nb_finite_cells();
  std::vector<glm::uvec3> triangles(triangle_count);
  Jobs::RunParallelFor(triangle_count, [&](const auto i) {
    for (GEO::index_t lv = 0; lv < 4; ++lv) {
      uint32_t v_i = 0;
      if (const GEO::signed_index_t v = delaunay->cell_vertex(static_cast<GEO::index_t>(i) + delaunay->nb_cells(), lv);
          v != -1) {
        triangles[i][v_i] = static_cast<GEO::index_t>(v);
        v_i++;
      }
    }
  });
  return triangles;
}

std::vector<glm::uvec3> Delaunay3D::GenerateConcaveHullTriangles(const std::vector<glm::vec3>& points, const float max_edge_length) {
  ProcessDelauney3D(false, points);

  std::vector<glm::uvec3> triangles;
  for (GEO::index_t t = 0; t < delaunay->nb_cells(); ++t) {
    const auto v0 = static_cast<GEO::index_t>(delaunay->cell_vertex(t, 0));
    const auto v1 = static_cast<GEO::index_t>(delaunay->cell_vertex(t, 1));
    const auto v2 = static_cast<GEO::index_t>(delaunay->cell_vertex(t, 2));
    const auto v3 = static_cast<GEO::index_t>(delaunay->cell_vertex(t, 3));
    if (glm::distance(points[v0], points[v1]) > max_edge_length)
      continue;
    if (glm::distance(points[v0], points[v2]) > max_edge_length)
      continue;
    if (glm::distance(points[v0], points[v3]) > max_edge_length)
      continue;
    if (glm::distance(points[v1], points[v2]) > max_edge_length)
      continue;
    if (glm::distance(points[v1], points[v3]) > max_edge_length)
      continue;
    if (glm::distance(points[v2], points[v3]) > max_edge_length)
      continue;
    triangles.emplace_back(v0, v1, v2);
    triangles.emplace_back(v1, v2, v3);
    triangles.emplace_back(v2, v3, v0);
    triangles.emplace_back(v3, v0, v1);
  }
  return triangles;
}

std::vector<glm::uvec4> Delaunay3D::GenerateConcaveHullTetrahedrons(const std::vector<glm::vec3>& points,
                                                                  const float max_edge_length) {
  std::vector<glm::uvec4> tetrahedrons;
  for (GEO::index_t t = 0; t < delaunay->nb_cells(); ++t) {
    const auto v0 = static_cast<GEO::index_t>(delaunay->cell_vertex(t, 0));
    const auto v1 = static_cast<GEO::index_t>(delaunay->cell_vertex(t, 1));
    const auto v2 = static_cast<GEO::index_t>(delaunay->cell_vertex(t, 2));
    const auto v3 = static_cast<GEO::index_t>(delaunay->cell_vertex(t, 3));
    if (glm::distance(points[v0], points[v1]) > max_edge_length)
      continue;
    if (glm::distance(points[v0], points[v2]) > max_edge_length)
      continue;
    if (glm::distance(points[v0], points[v3]) > max_edge_length)
      continue;
    if (glm::distance(points[v1], points[v2]) > max_edge_length)
      continue;
    if (glm::distance(points[v1], points[v3]) > max_edge_length)
      continue;
    if (glm::distance(points[v2], points[v3]) > max_edge_length)
      continue;
    tetrahedrons.emplace_back(v0, v1, v2, v3);
  }
  return tetrahedrons;
}

std::shared_ptr<Mesh> Delaunay3D::GenerateConvexHullMesh(const std::vector<glm::vec3>& points) {
  auto triangles = GenerateConvexHullTriangles(points);
  return GenerateMesh(triangles, points);
}

std::shared_ptr<Mesh> Delaunay3D::GenerateConcaveHullMesh(const std::vector<glm::vec3>& points, float max_edge_length) {
  auto triangles = GenerateConcaveHullTriangles(points, max_edge_length);
  return GenerateMesh(triangles, points);
}