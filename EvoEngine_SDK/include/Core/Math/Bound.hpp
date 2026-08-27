
#pragma once
#include "IDataComponent.hpp"

namespace evo_engine {
struct Vertex;

/**
 * @brief Represents an axis-aligned bounding box.
 */
struct EVOENGINE_API Bound {
  glm::vec3 min = glm::vec3(FLT_MAX);  /**< The minimum point of the bounding box. */
  glm::vec3 max = glm::vec3(-FLT_MAX); /**< The maximum point of the bounding box. */

  /**
   * @brief Computes the size of the bounding box.
   * @return Size as a `glm::vec3`.
   */
  [[nodiscard]] glm::vec3 Size() const;

  /**
   * @brief Computes the center point of the bounding box.
   * @return Center as a `glm::vec3`.
   */
  [[nodiscard]] glm::vec3 Center() const;

  /**
   * @brief Checks if a point is inside the bounding box.
   * @param position The point to check as a `glm::vec3`.
   * @return `true` if the point is inside the bounding box, otherwise `false`.
   */
  [[nodiscard]] bool InBound(const glm::vec3& position) const;

  /**
   * @brief Applies a transformation matrix to the bounding box.
   * @param transform The transformation matrix.
   */
  void ApplyTransform(const glm::mat4& transform);

  /**
   * @brief Populates a list of corner points of the bounding box.
   * @param corners A vector to store the corners of the bounding box.
   */
  void PopulateCorners(std::vector<glm::vec3>& corners) const;
};

/**
 * @brief Represents a 3D ray.
 */
struct EVOENGINE_API Ray : IDataComponent {
  glm::vec3 start;     /**< The starting point of the ray. */
  glm::vec3 direction; /**< The direction vector of the ray. */
  float length;        /**< The length of the ray. */

  /**
   * @brief Default constructor for the Ray.
   */
  Ray() = default;

  /**
   * @brief Constructs a ray between two points.
   * @param start The starting point of the ray.
   * @param end The endpoint of the ray.
   */
  Ray(const glm::vec3& start, const glm::vec3& end);

  /**
   * @brief Constructs a ray given a start, direction, and length.
   * @param start The starting point of the ray.
   * @param direction The direction vector of the ray.
   * @param length The length of the ray.
   */
  Ray(const glm::vec3& start, const glm::vec3& direction, float length);

  /**
   * @brief Checks if the ray intersects with a sphere.
   * @param position The sphere center.
   * @param radius The sphere radius.
   * @return `true` if the ray intersects the sphere, otherwise `false`.
   */
  [[nodiscard]] bool Intersect(const glm::vec3& position, float radius) const;

  /**
   * @brief Checks if the ray intersects with a transformed bounding box.
   * @param transform The transformation matrix applied to the bounding box.
   * @param bound The bounding box.
   * @return `true` if the ray intersects, otherwise `false`.
   */
  [[nodiscard]] bool Intersect(const glm::mat4& transform, const Bound& bound) const;

  /**
   * @brief Computes the endpoint of the ray.
   * @return The endpoint as `glm::vec3`.
   */
  [[nodiscard]] glm::vec3 GetEnd() const;

  /**
   * @brief Finds the closest point on a line segment to a given point.
   * @param point The query point.
   * @param a The start of the line segment.
   * @param b The end of the line segment.
   * @return The closest point on the line segment as `glm::vec3`.
   */
  [[nodiscard]] static glm::vec3 ClosestPointOnLine(const glm::vec3& point, const glm::vec3& a, const glm::vec3& b);
};

/**
 * @brief Represents a plane in 3D space.
 */
struct EVOENGINE_API Plane {
  /**
   * @brief Constructs a plane from a parameter vector.
   * @param param A `glm::vec4` containing the plane parameters.
   */
  explicit Plane(const glm::vec4& param);

  /**
   * @brief Constructs a plane from its normal and the distance from the origin.
   * @param normal The normal vector of the plane.
   * @param distance The distance of the plane from the origin.
   */
  Plane(const glm::vec3& normal, float distance);

  /**
   * @brief Constructs a plane from its normal and a point on the plane.
   * @param normal The normal vector of the plane.
   * @param point A point on the plane.
   */
  Plane(const glm::vec3& normal, const glm::vec3& point);

  float a, b, c, d; /**< The plane equation coefficients. */

  /**
   * @brief Default constructor for the Plane.
   */
  Plane();

  /**
   * @brief Normalizes the coefficients of the plane.
   */
  void Normalize();

  /**
   * @brief Calculates the distance of a triangle from the plane.
   * @param vertices The list of vertices.
   * @param triangle The indices of the triangle vertices.
   * @return The average distance of the triangle from the plane.
   */
  [[nodiscard]] float CalculateTriangleDistance(const std::vector<Vertex>& vertices, const glm::uvec3& triangle) const;

  /**
   * @brief Calculates the maximum distance of a triangle from the plane.
   * @param vertices The list of vertices.
   * @param triangle The indices of the triangle vertices.
   * @return The maximum distance of the triangle from the plane.
   */
  [[nodiscard]] float CalculateTriangleMaxDistance(const std::vector<Vertex>& vertices,
                                                   const glm::uvec3& triangle) const;

  /**
   * @brief Calculates the minimum distance of a triangle from the plane.
   * @param vertices The list of vertices.
   * @param triangle The indices of the triangle vertices.
   * @return The minimum distance of the triangle from the plane.
   */
  [[nodiscard]] float CalculateTriangleMinDistance(const std::vector<Vertex>& vertices,
                                                   const glm::uvec3& triangle) const;

  /**
   * @brief Computes the normal vector of the plane.
   * @return The normal vector as `glm::vec3`.
   */
  [[nodiscard]] glm::vec3 GetNormal() const;

  /**
   * @brief Retrieves the distance of the plane from the origin.
   * @return The distance as a `float`.
   */
  [[nodiscard]] float GetDistance() const;

  /**
   * @brief Calculates the signed distance of a point from the plane.
   * @param point The point to calculate the distance for.
   * @return The signed distance as a `float`.
   */
  [[nodiscard]] float CalculatePointDistance(const glm::vec3& point) const;

  /**
   * @brief Projects a point onto the plane.
   * @param point The point to project.
   * @return The projected point as `glm::vec3`.
   */
  [[nodiscard]] glm::vec3 Project(const glm::vec3& point) const;

  /**
   * @brief Projects a 3D point onto a plane defined by an origin and two directional vectors.
   * @param point The point to project.
   * @param plane_origin The origin of the plane.
   * @param plane_dir_x A vector defining one direction of the plane.
   * @param plane_dir_y A vector defining another direction of the plane.
   * @return The projected point as `glm::vec2`.
   */
  static glm::vec2 ProjectPointToPlane(const glm::vec3& point, const glm::vec3& plane_origin,
                                       const glm::vec3& plane_dir_x, const glm::vec3& plane_dir_y);
};
}  // namespace evo_engine
