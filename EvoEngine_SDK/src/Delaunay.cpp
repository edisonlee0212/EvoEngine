#include "Delaunay.hpp"
#include "Jobs.hpp"
#include "Mesh.hpp"
#include "ProjectManager.hpp"
#define USE_GEOGRAM false
#if USE_GEOGRAM
#include "geogram/basic/psm.h"
#include "geogram/delaunay/parallel_delaunay_3d.h"
#else
#include "tetgen.h"

#endif
using namespace evo_engine;

#if USE_GEOGRAM
GEO::Delaunay_var GeogramProcessDelaunay3D(const bool keeps_infinite, const std::vector<glm::vec3>& points) {
  std::vector<double> converted_points(points.size() * 3);
  Jobs::RunParallelFor(points.size(), [&](const auto i) {
    converted_points[i * 3] = points[i].x;
    converted_points[i * 3 + 1] = points[i].y;
    converted_points[i * 3 + 2] = points[i].z;
  });
  GEO::Delaunay_var delaunay = GEO::Delaunay::create(3, "BDEL");

  // If we want the convex hull, we keep the infinite facets,
  // because the convex hull can be retrieved as the finite facets
  // of the infinite cells (note: it would be also possible to
  // throw away the infinite cells and get the convex hull as
  // the facets adjacent to no cell).
  delaunay->set_keeps_infinite(keeps_infinite);
  delaunay->set_vertices(static_cast<GEO::index_t>(points.size()), converted_points.data());
  return delaunay;
}
#else
tetgenio TetGenProcessDelaunay3D(tetgenbehavior behavior, const std::vector<glm::vec3>& points) {
  tetgenio in, out;
  in.numberofpoints = points.size();
  in.pointlist = new double[in.numberofpoints * 3]; 

  Jobs::RunParallelFor(points.size(), [&](const auto i) {
    in.pointlist[i * 3] = points[i].x;
    in.pointlist[i * 3 + 1] = points[i].y;
    in.pointlist[i * 3 + 2] = points[i].z;
  });

  try {
    tetrahedralize(&behavior, &in, &out);  // Perform the mesh generation
  } catch (const int err) {
    EVOENGINE_ERROR("Error during tetrahedralization: " + std::to_string(err));
  }
  return out;
}

#endif

std::shared_ptr<Mesh> GenerateMesh(std::vector<glm::uvec3>& triangles, const std::vector<glm::vec3>& points) {
  std::unordered_map<unsigned, unsigned> vertices_map;
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


std::vector<Delaunay3D::Tetrahedron> Delaunay3D::GenerateTetrahedrons(const std::vector<glm::vec3>& points) {
  std::vector<Tetrahedron> tetrahedrons{};
#if USE_GEOGRAM
  const auto delaunay = GeogramProcessDelaunay3D(false, points);
  tetrahedrons.resize(delaunay->nb_cells());
  Jobs::RunParallelFor(delaunay->nb_cells(), [&](const auto t) {
    auto& tetrahedron = tetrahedrons[t];
    tetrahedron.v[0] = static_cast<GEO::index_t>(delaunay->cell_vertex(t, 0));
    tetrahedron.v[1] = static_cast<GEO::index_t>(delaunay->cell_vertex(t, 1));
    tetrahedron.v[2] = static_cast<GEO::index_t>(delaunay->cell_vertex(t, 2));
    tetrahedron.v[3] = static_cast<GEO::index_t>(delaunay->cell_vertex(t, 3));
    tetrahedron.volume = CalculateTetrahedronVolume(points[tetrahedron.v[0]], points[tetrahedron.v[1]],
                                                    points[tetrahedron.v[2]], points[tetrahedron.v[3]]);
    tetrahedron.circumradius = CalculateTetrahedronCircumradius(points[tetrahedron.v[0]], points[tetrahedron.v[1]],
                                                                points[tetrahedron.v[2]], points[tetrahedron.v[3]]);
  });
#else
  tetgenbehavior behavior{};  // Default behavior (Delaunay tetrahedralization)
  behavior.zeroindex = 1;
  const auto out = TetGenProcessDelaunay3D(behavior, points);

  tetrahedrons.resize(out.numberoftetrahedra);
  Jobs::RunParallelFor(out.numberoftetrahedra, [&](const auto i) {
    auto& tetrahedron = tetrahedrons[i];
    tetrahedron.v[0] = out.tetrahedronlist[i * 4];
    tetrahedron.v[1] = out.tetrahedronlist[i * 4 + 1];
    tetrahedron.v[2] = out.tetrahedronlist[i * 4 + 2];
    tetrahedron.v[3] = out.tetrahedronlist[i * 4 + 3];
    tetrahedron.volume = CalculateTetrahedronVolume(points[tetrahedron.v[0]], points[tetrahedron.v[1]],
                                                    points[tetrahedron.v[2]], points[tetrahedron.v[3]]);
    tetrahedron.circumradius = CalculateTetrahedronCircumradius(points[tetrahedron.v[0]], points[tetrahedron.v[1]],
                                                                points[tetrahedron.v[2]], points[tetrahedron.v[3]]);
  });
#endif
  return tetrahedrons;
}


std::vector<glm::uvec3> Delaunay3D::GenerateConvexHullTriangles(const std::vector<glm::vec3>& points) {
  std::vector<glm::uvec3> triangles{};
#if USE_GEOGRAM
  const auto delaunay = GeogramProcessDelaunay3D(true, points);
  const auto triangle_count = delaunay->nb_cells() - delaunay->nb_finite_cells();
  triangles.resize(triangle_count);
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
#else
  tetgenbehavior behavior{};  // Default behavior (Delaunay tetrahedralization)
  behavior.quality = 1;
  behavior.quiet = 1;
  behavior.convex = 1;        // Convex hull generation
  behavior.nobisect = 1;      // Do not add Steiner points
  behavior.facesout = 1;      // Output boundary faces (convex hull)
  behavior.zeroindex = 1;
  const auto out = TetGenProcessDelaunay3D(behavior, points);

  triangles.resize(out.numberoftrifaces);
  Jobs::RunParallelFor(out.numberoftrifaces, [&](const auto i) {
    triangles[i] = glm::uvec3(out.trifacelist[i * 3], out.trifacelist[i * 3 + 1], out.trifacelist[i * 3 + 2]
    );
  });

#endif
  return triangles;
}

std::vector<glm::uvec3> Delaunay3D::FindOuterShell(const std::vector<Tetrahedron>& tetrahedrons,
                                                   const std::function<bool(const Tetrahedron& tetrahedron)>& filter) {
  std::unordered_map<glm::uvec3, uint32_t> face_map{};
  const auto sort_triangle_indices = [](const glm::uvec3& src) {
    if (src.x <= src.y) {
      if (src.z <= src.x) {
        return glm::uvec3(src.z, src.x, src.y);
      }
      if (src.z <= src.y) {
        return glm::uvec3(src.x, src.z, src.y);
      }
      return src;
    }
    if (src.z <= src.y) {
      return glm::uvec3(src.z, src.y, src.x);
    }
    if (src.z <= src.x) {
      return glm::uvec3(src.y, src.z, src.x);
    }
    return glm::uvec3(src.y, src.x, src.z);
  };
  const auto register_map = [&](const glm::uvec3& triangle) {
    if (const auto search = face_map.find(triangle); search == face_map.end()) {
      face_map.insert({triangle, 1});
    }else {
      search->second++;
    }
  };
  for (const auto& i : tetrahedrons) {
    if (!filter(i))
      continue;
    register_map(sort_triangle_indices(glm::uvec3(i.v[1], i.v[2], i.v[3])));
    register_map(sort_triangle_indices(glm::uvec3(i.v[0], i.v[2], i.v[3])));
    register_map(sort_triangle_indices(glm::uvec3(i.v[0], i.v[1], i.v[3])));
    register_map(sort_triangle_indices(glm::uvec3(i.v[0], i.v[1], i.v[2])));
  }
  std::vector<glm::uvec3> triangles;
  for (const auto& i : face_map) {
    if (i.second == 1) {
      triangles.emplace_back(i.first);
    }
  }
  return triangles;
}

std::vector<glm::uvec3> Delaunay3D::GenerateAlphaShapeTriangles(const std::vector<glm::vec3>& points,
                                                                const float max_circumradius) {
  const auto tetrahedrons = GenerateTetrahedrons(points);
  return FindOuterShell(tetrahedrons, [&](const Tetrahedron& tetrahedron) {
    return tetrahedron.circumradius <= max_circumradius;
  });
}
std::vector<glm::uvec3> Delaunay3D::GenerateConcaveHullTriangles(const std::vector<glm::vec3>& points,
                                                                 const float max_edge_length) {
  const auto tetrahedrons = GenerateTetrahedrons(points);
  return FindOuterShell(tetrahedrons, [&](const Tetrahedron& tetrahedron) {
    if (glm::distance(points[tetrahedron.v[0]], points[tetrahedron.v[1]]) > max_edge_length)
      return false;
    if (glm::distance(points[tetrahedron.v[0]], points[tetrahedron.v[2]]) > max_edge_length)
      return false;
    if (glm::distance(points[tetrahedron.v[0]], points[tetrahedron.v[3]]) > max_edge_length)
      return false;
    if (glm::distance(points[tetrahedron.v[1]], points[tetrahedron.v[2]]) > max_edge_length)
      return false;
    if (glm::distance(points[tetrahedron.v[1]], points[tetrahedron.v[3]]) > max_edge_length)
      return false;
    if (glm::distance(points[tetrahedron.v[2]], points[tetrahedron.v[3]]) > max_edge_length)
      return false;
  });
}

float Delaunay3D::CalculateTetrahedronCircumradius(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2,
                                                   const glm::vec3& p3) {
  const float a = glm::distance(p1, p2);
  const float b = glm::distance(p0, p2);
  const float c = glm::distance(p0, p1);
  const float x = glm::distance(p3, p2);
  const float y = glm::distance(p3, p1);
  const float z = glm::distance(p3, p0);

  const float v = CalculateTetrahedronVolume(p0, p1, p2, p3);
  return glm::sqrt(a * a * b * b * c * c + a * a * x * x + b * b * y * y + c * c * z * z - a * a * b * b * z * z -
              a * a * c * c * y * y - b * b * c * c * x * x) /
         (6.f * v);
}

float Delaunay3D::CalculateTetrahedronVolume(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2,
                                             const glm::vec3& p3) {
  return 0.1666666f *
         glm::abs((p0[0] * (p1[1] * p2[2] + p3[1] * p2[2] + p1[2] * p3[1] - p1[2] * p2[1] - p3[2] * p2[1] - p1[1] * p3[2])) -
                  p1[0] * (p0[1] * p2[2] + p3[1] * p2[2] + p0[2] * p3[1] - p0[2] * p2[1] - p3[2] * p2[1] - p0[1] * p3[2]) +
                  p2[0] * (p0[1] * p1[2] + p3[1] * p1[2] + p0[2] * p3[1] - p0[2] * p1[1] - p3[2] * p1[1] - p0[1] * p3[2]) -
                  p3[0] * (p0[1] * p1[2] + p2[1] * p1[2] + p0[2] * p2[1] - p0[2] * p1[1] - p2[2] * p1[1] - p0[1] * p2[2]));
}




std::shared_ptr<Mesh> Delaunay3D::GenerateConvexHullMesh(const std::vector<glm::vec3>& points) {
  auto triangles = GenerateConvexHullTriangles(points);
  return GenerateMesh(triangles, points);
}

std::shared_ptr<Mesh> Delaunay3D::GenerateAlphaShapeMesh(const std::vector<glm::vec3>& points, const float max_circumradius) {
  auto triangles = GenerateAlphaShapeTriangles(points, max_circumradius);
  return GenerateMesh(triangles, points);
}

std::shared_ptr<Mesh> Delaunay3D::GenerateConcaveHullMesh(const std::vector<glm::vec3>& points, float max_edge_length) {
  auto triangles = GenerateConcaveHullTriangles(points, max_edge_length);
  return GenerateMesh(triangles, points);
}
