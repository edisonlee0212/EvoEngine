#pragma once
#include <Curve.hpp>

namespace digital_agriculture_plugin {
using namespace evo_engine;

/**
 * @class MaizeSplineSegment
 * @brief Represents a segment of the Maize spline.
 *
 * This class stores the position, orientation, and radius of a segment of the Maize plant.
 */
class MaizeSplineSegment {
 public:
  glm::vec3 position;                ///< The position of the segment.
  glm::vec3 front;                   ///< The forward direction vector.
  glm::vec3 up;                      ///< The up direction vector.
  float radius;                      ///< The radius of the segment.
  float theta;                       ///< The angular rotation of the segment.
  float left_height_offset = 1.0f;   ///< Height offset for the left side.
  float right_height_offset = 1.0f;  ///< Height offset for the right side.

  /**
   * @brief Default constructor.
   */
  MaizeSplineSegment() = default;

  /**
   * @brief Parameterized constructor to initialize a Maize spline segment.
   * @param position The position of the segment.
   * @param up The up direction vector.
   * @param front The forward direction vector.
   * @param radius The radius of the segment.
   * @param theta The angular rotation of the segment.
   * @param left_height_offset The left side height offset.
   * @param right_height_offset The right side height offset.
   */
  MaizeSplineSegment(const glm::vec3& position, const glm::vec3& up, const glm::vec3& front, float radius,
                       float theta, float left_height_offset = 0.0f, float right_height_offset = 0.0f);

  /**
   * @brief Computes the leaf point for the given angle.
   * @param angle The angle in radians.
   * @return The computed leaf point position.
   */
  [[nodiscard]] glm::vec3 GetLeafPoint(float angle) const;

  /**
   * @brief Computes the stem point for the given angle.
   * @param angle The angle in radians.
   * @return The computed stem point position.
   */
  [[nodiscard]] glm::vec3 GetStemPoint(float angle) const;

  /**
   * @brief Computes the normal vector at the given angle.
   * @param angle The angle in radians.
   * @return The computed normal vector.
   */
  [[nodiscard]] glm::vec3 GetNormal(float angle) const;
};

/**
 * @class MaizeSpline
 * @brief Represents a spline composed of Maize segments.
 *
 * This class contains multiple MaizeSplineSegment instances and provides methods
 * for their procedural generation and processing.
 */
class MaizeSpline {
 public:
  std::vector<MaizeSplineSegment> segments;  ///< List of segments forming the spline.

  /**
   * @brief Subdivides the spline into smaller segments based on distance.
   * @param subdivision_distance The maximum distance between subdivisions.
   * @param subdivided_segments The resulting subdivided segments.
   */
  void SubdivideByDistance(float subdivision_distance, std::vector<MaizeSplineSegment>& subdivided_segments) const;

  /**
   * @brief Serializes the spline data into a YAML emitter.
   * @param name The name used for serialization.
   * @param out The YAML emitter to write data into.
   */
  void Serialize(const std::string& name, YAML::Emitter& out) const;

  /**
   * @brief Deserializes the spline data from a YAML node.
   * @param name The name used for deserialization.
   * @param in The YAML node containing serialized data.
   */
  void Deserialize(const std::string& name, const YAML::Node& in);

  /**
   * @brief Retrieves control points for a given segment.
   * @param segment_index The index of the segment.
   * @param p0 The first control point.
   * @param p1 The second control point.
   * @param p2 The third control point.
   * @param p3 The fourth control point.
   */
  void GetPositionControlPoints(uint32_t segment_index, glm::vec3& p0, glm::vec3& p1, glm::vec3& p2,
                                glm::vec3& p3) const;

  /**
   * @brief Computes the arc length of a specific segment.
   * @param segment_index The index of the segment.
   * @param t_start The start parameter of the segment.
   * @param t_end The end parameter of the segment.
   * @param tolerance The tolerance for numerical integration.
   * @return The computed arc length.
   */
  [[nodiscard]] float GetSegmentArcLength(uint32_t segment_index, float t_start = 0.f, float t_end = 1.f,
                                          float tolerance = 0.0001f) const;

  /**
   * @brief Computes the total arc length of the spline.
   * @param tolerance The tolerance for numerical integration.
   * @return The computed total arc length.
   */
  [[nodiscard]] float GetArcLength(float tolerance = 0.0001f) const;

  /**
   * @brief Interpolates a segment at a given index with a parameter t.
   * @param segment_index The index of the segment.
   * @param t The interpolation parameter.
   * @return The interpolated segment.
   */
  [[nodiscard]] MaizeSplineSegment InterpolateSegment(uint32_t segment_index, float t) const;

  /**
   * @brief Rebuilds the spline with a fixed number of segments.
   * @param segment_count The desired number of segments.
   * @param tolerance The tolerance for error control.
   * @return A vector containing the rebuilt segments.
   */
  [[nodiscard]] std::vector<MaizeSplineSegment> RebuildFixedSizeSegments(size_t segment_count,
                                                                           float tolerance = 0.0001f) const;

  /**
   * @brief Rebuilds the spline with a fixed segment length.
   * @param segment_length The desired length of segments.
   * @param tolerance The tolerance for error control.
   * @return A vector containing the rebuilt segments.
   */
  [[nodiscard]] std::vector<MaizeSplineSegment> RebuildFixedLengthSegments(float segment_length,
                                                                             float tolerance = 0.0001f) const;

  /**
   * @brief Extracts and returns only the stem part of the spline.
   * @return A vector containing the stem segments.
   */
  [[nodiscard]] std::vector<MaizeSplineSegment> GetStemPart() const;

  /**
   * @brief Extracts and returns only the leaf part of the spline.
   * @return A vector containing the leaf segments.
   */
  [[nodiscard]] std::vector<MaizeSplineSegment> GetLeafPart() const;
};

}  // namespace digital_agriculture_plugin
