#pragma once
#include "Mesh.hpp"

namespace evo_engine {
class Delaunay3D {
 public:
  static std::vector<glm::uvec4> GenerateConcaveHullTetrahedrons(const std::vector<glm::vec3>& points,
                                                               float max_edge_length);

  static std::vector<glm::uvec3> GenerateConvexHullTriangles(const std::vector<glm::vec3>& points);
  static std::vector<glm::uvec3> GenerateConcaveHullTriangles(const std::vector<glm::vec3>& points,
                                                              float max_edge_length);

  static std::shared_ptr<Mesh> GenerateConvexHullMesh(const std::vector<glm::vec3>& points);
  static std::shared_ptr<Mesh> GenerateConcaveHullMesh(const std::vector<glm::vec3>& points, float max_edge_length);
};
}  // namespace evo_engine