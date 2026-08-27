
#pragma once
#include "Mesh.hpp"

namespace evo_engine {

/**
 * @brief A class for performing 3D Delaunay tetrahedralization and related geometric operations.
 */
class EVOENGINE_API Delaunay3D {
 public:
  /**
   * @brief A structure representing a single tetrahedron in 3D space.
   */
  struct Tetrahedron {
    int v[4]{};                    /**< Indices of the vertices that form the tetrahedron. */
    int neighbor_tet_indices[4]{}; /**< Indices of the neighboring tetrahedrons. */
    float circumradius = 0.f;      /**< Circumradius of the tetrahedron. */
    float volume = 0.f;            /**< Volume of the tetrahedron. */
  };

  /**
   * @brief Calculates the circumradius of a tetrahedron given its vertices.
   *
   * @param p0 The first vertex of the tetrahedron.
   * @param p1 The second vertex of the tetrahedron.
   * @param p2 The third vertex of the tetrahedron.
   * @param p3 The fourth vertex of the tetrahedron.
   * @return The circumradius of the tetrahedron.
   */
  static float CalculateTetrahedronCircumradius(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2,
                                                const glm::vec3& p3);

  /**
   * @brief Calculates the volume of a tetrahedron given its vertices.
   *
   * @param p0 The first vertex of the tetrahedron.
   * @param p1 The second vertex of the tetrahedron.
   * @param p2 The third vertex of the tetrahedron.
   * @param p3 The fourth vertex of the tetrahedron.
   * @return The volume of the tetrahedron.
   */
  static float CalculateTetrahedronVolume(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2,
                                          const glm::vec3& p3);

  /**
   * @brief Generates a list of tetrahedrons by performing Delaunay triangulation on a set of points.
   *
   * @param points A vector containing the set of points in 3D space.
   * @return A vector containing the generated tetrahedrons.
   */
  static std::vector<Tetrahedron> GenerateTetrahedrons(const std::vector<glm::vec3>& points);

  /**
   * @brief Finds the outer shell triangles of a set of tetrahedrons.
   *
   * @param tetrahedrons A vector containing the tetrahedrons.
   * @param filter A function to filter tetrahedrons to consider.
   * @return A vector of triangle indices representing the outer shell.
   */
  static std::vector<glm::uvec3> FindOuterShell(const std::vector<Tetrahedron>& tetrahedrons,
                                                const std::function<bool(const Tetrahedron& tetrahedron)>& filter);

  /**
   * @brief Generates the convex hull as a set of triangular faces from a set of 3D points.
   *
   * @param points A vector containing the set of points in 3D space.
   * @return A vector of triangle indices representing the convex hull.
   */
  static std::vector<glm::uvec3> GenerateConvexHullTriangles(const std::vector<glm::vec3>& points);

  /**
   * @brief Generates the alpha shape as a set of triangular faces from a set of 3D points.
   *
   * @param points A vector containing the set of points in 3D space.
   * @param max_circumradius The maximum circumradius to filter tetrahedrons for the alpha shape.
   * @return A vector of triangle indices representing the alpha shape.
   */
  static std::vector<glm::uvec3> GenerateAlphaShapeTriangles(const std::vector<glm::vec3>& points,
                                                             float max_circumradius);

  /**
   * @brief Generates the concave hull as a set of triangular faces from a set of 3D points.
   *
   * @param points A vector containing the set of points in 3D space.
   * @param max_edge_length The maximum edge length to filter edges for the concave hull.
   * @return A vector of triangle indices representing the concave hull.
   */
  static std::vector<glm::uvec3> GenerateConcaveHullTriangles(const std::vector<glm::vec3>& points,
                                                              float max_edge_length);

  /**
   * @brief Generates a Mesh object representing the convex hull from a set of 3D points.
   *
   * @param points A vector containing the set of points in 3D space.
   * @return A shared pointer to the generated Mesh object.
   */
  static std::shared_ptr<Mesh> GenerateConvexHullMesh(const std::vector<glm::vec3>& points);

  /**
   * @brief Generates a Mesh object representing the alpha shape from a set of 3D points.
   *
   * @param points A vector containing the set of points in 3D space.
   * @param max_circumradius The maximum circumradius to filter tetrahedrons for the alpha shape.
   * @return A shared pointer to the generated Mesh object.
   */
  static std::shared_ptr<Mesh> GenerateAlphaShapeMesh(const std::vector<glm::vec3>& points, float max_circumradius);

  /**
   * @brief Generates a Mesh object representing the concave hull from a set of 3D points.
   *
   * @param points A vector containing the set of points in 3D space.
   * @param max_edge_length The maximum edge length to filter edges for the concave hull.
   * @return A shared pointer to the generated Mesh object.
   */
  static std::shared_ptr<Mesh> GenerateConcaveHullMesh(const std::vector<glm::vec3>& points, float max_edge_length);
};

}  // namespace evo_engine
