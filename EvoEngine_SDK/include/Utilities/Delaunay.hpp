#pragma once
#include "Mesh.hpp"
#include "geogram/basic/psm.h"
#include "geogram/delaunay/parallel_delaunay_3d.h"

namespace evo_engine {
class Delaunay3D {
 public:
  static GEO::Delaunay_var ProcessDelaunay3D(bool keeps_infinite, const std::vector<glm::vec3>& points);

  struct Tetrahedron {
    uint32_t v[4]{};
    float circumradius = 0.f;
    float volume = 0.f;
  };

  static float CalculateTetrahedronCircumradius(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2,
                                     const glm::vec3& p3);
  static float CalculateTetrahedronVolume(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2,
                                     const glm::vec3& p3);

  static std::vector<Tetrahedron> GenerateTetrahedrons(const std::vector<glm::vec3>& points);

  static std::vector<glm::uvec3> FindOuterShell(const std::vector<Tetrahedron>& tetrahedrons,
                                                const std::function<bool(const Tetrahedron& tetrahedron)>& filter);

  static std::vector<glm::uvec3> GenerateConvexHullTriangles(const std::vector<glm::vec3>& points);
  static std::vector<glm::uvec3> GenerateAlphaShapeTriangles(const std::vector<glm::vec3>& points,
                                                              float max_circumradius);
  static std::vector<glm::uvec3> GenerateConcaveHullTriangles(const std::vector<glm::vec3>& points,
                                                             float max_edge_length);

  static std::shared_ptr<Mesh> GenerateConvexHullMesh(const std::vector<glm::vec3>& points);
  static std::shared_ptr<Mesh> GenerateAlphaShapeMesh(const std::vector<glm::vec3>& points, float max_circumradius);
  static std::shared_ptr<Mesh> GenerateConcaveHullMesh(const std::vector<glm::vec3>& points, float max_edge_length);

};
}  // namespace evo_engine