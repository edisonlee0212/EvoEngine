/*****************************************************************//**
 * \file   SorghumDescriptorReconstruction.hpp
 * \brief  Provides interfaces for the bezier splines and sorghum reconstruction
 * 
 * \author Demoy
 * \date   February 2025
 *********************************************************************/

// @edisonlee0212: here I define methods.

#pragma once
#include "SorghumDescriptor.hpp"
using namespace evo_engine;
namespace digital_agriculture_plugin {

/**
 * @struct CubicBezierPoint
 * @brief Represents the shared control points in a cubic Bezier spline.
 */
struct CubicBezierPoint {
  glm::vec3 position;      ///< The position of the control point.
  glm::vec3 left_handle;   ///< The 3rd control point for the preceding Bezier curve.
  glm::vec3 right_handle;  ///< The 2nd control point for the next Bezier curve.

  /**
   * @brief Checks if the control point maintains C1 continuity.
   * @return True if C1 continuity is maintained, false otherwise.
   */
  [[nodiscard]] inline bool IsC1Continuity() const {
    return right_handle - position == position - left_handle;
  }
};

/**
 * @struct CubicSplineSample
 * @brief Represents a sample point on a cubic Bezier spline.
 */
struct CubicSplineSample {
  int segmentIndex;    ///< The index of the segment containing the sample.
  float t;             ///< The parameter t along the segment.
  glm::vec3 position;  ///< The 3D position of the sample.
};

/**
 * @class CubicBezierSpline
 * @brief Represents a cubic Bezier spline and provides interpolation and sampling methods.
 */
class CubicBezierSpline {
 private:
  std::vector<float> segmentLengths;  ///< Lengths of each segment.

  /**
   * @brief Returns uniformly distributed sample points along the spline.
   * @param num The number of sample points.
   * @return A vector of uniformly distributed sample points.
   */
  [[nodiscard]] std::vector<CubicSplineSample> getUniformSamples(int num);

 public:
  std::vector<CubicBezierPoint> joints;  ///< The shared control points defining the spline.

  /**
   * @brief Performs cubic Bezier interpolation.
   * @param v0 The first control point.
   * @param v1 The second control point.
   * @param v2 The third control point.
   * @param v3 The fourth control point.
   * @param t The interpolation parameter (0 <= t <= 1).
   * @return The interpolated point.
   */
  [[nodiscard]] static glm::vec3 interpolation(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                                               const glm::vec3& v3,
                                 const float t);

  /**
   * @brief Computes the tangent vector at a given parameter t on the spline segment.
   * @param v0 The first control point.
   * @param v1 The second control point.
   * @param v2 The third control point.
   * @param v3 The fourth control point.
   * @param t The interpolation parameter (0 <= t <= 1).
   * @return The tangent vector.
   */
  [[nodiscard]] static glm::vec3 getTangent(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                                            const glm::vec3& v3,
                              const float t);

  /**
   * @brief Calculates the length of a segment using adaptive sampling.
   * @param v0 The first control point.
   * @param v1 The second control point.
   * @param v2 The third control point.
   * @param v3 The fourth control point.
   * @param t_start The starting parameter.
   * @param t_end The ending parameter.
   * @param tolerance The tolerance for adaptive sampling.
   * @return The estimated length of the segment.
   */
  [[nodiscard]] static float calculateLengthAdaptive(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                                       const glm::vec3& v3, float t_start = 0, float t_end = 1,
                                       const float tolerance = 0.001f);

  /**
   * @brief Finds the parameter t corresponding to a target length using adaptive sampling.
   * @param v0 The first control point.
   * @param v1 The second control point.
   * @param v2 The third control point.
   * @param v3 The fourth control point.
   * @param t_start The starting parameter.
   * @param target_length The desired length.
   * @param tolerance The tolerance for adaptive sampling.
   * @return The parameter t that corresponds to the target length.
   */
  [[nodiscard]] static float findTAdaptive(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                                           const glm::vec3& v3,
                             const float t_start, const float target_length, const float tolerance);

  /**
   * @brief Computes the total length of the spline and updates segment lengths.
   * @return The total length of the spline.
   */
  float getLength();

  /**
   * @brief Samples points along the spline based on a given distance.
   * @param distance The distance to sample.
   * @return A vector of sampled points along the spline.
   */
  [[nodiscard]] std::vector<CubicSplineSample> getSamplesByLength(float distance);



  /**
   * @brief Finds intersection points between the spline and a given plane.
   * @param planePoint A point on the plane.
   * @param normal The normal vector of the plane.
   * @return A vector of intersection points.
   */
  [[nodiscard]] std::vector<glm::vec3> getSurfaceIntersection(const glm::vec3& planePoint,
                                                              const glm::vec3& normal) const;

  /**
   * @brief Computes the signed distance from a point to a plane.
   * @param point The point to test.
   * @param normal The normal vector of the plane.
   * @param planePoint A point on the plane.
   * @return The signed distance from the point to the plane.
   */
  inline static float planeEquation(const glm::vec3& point, const glm::vec3& normal, const glm::vec3& planePoint) {
    return glm::dot(normal, point - planePoint);
  }

  /**
   * @brief Uses the bisection method to find the intersection parameter t.
   * @param t_min The minimum t value.
   * @param t_max The maximum t value.
   * @param v0 The first control point.
   * @param v1 The second control point.
   * @param v2 The third control point.
   * @param v3 The fourth control point.
   * @param planePoint A point on the plane.
   * @param normal The normal vector of the plane.
   * @param epsilon The tolerance for convergence.
   * @return The intersection parameter t.
   */
  [[nodiscard]] static float bisectionMethod(float t_min, float t_max, const glm::vec3& v0, const glm::vec3& v1,
                                             const glm::vec3& v2,
                               const glm::vec3& v3, const glm::vec3& planePoint, const glm::vec3& normal,
                               float epsilon = 0.0001f);

  /**
   * @brief Uses Newton's method to find the intersection parameter t.
   * @param t The initial guess for the parameter.
   * @param v0 The first control point.
   * @param v1 The second control point.
   * @param v2 The third control point.
   * @param v3 The fourth control point.
   * @param planePoint A point on the plane.
   * @param normal The normal vector of the plane.
   * @param epsilon The tolerance for convergence.
   * @param maxIter The maximum number of iterations.
   * @return The intersection parameter t.
   */
  [[nodiscard]] static float newtonMethod(float t, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                                          const glm::vec3& v3,
                            const glm::vec3& planePoint, const glm::vec3& normal, float epsilon = 0.0001f,
                            int maxIter = 100);

  /**
   * @brief Computes the tangent at a specific sample point.
   * @param sample The spline sample.
   * @return The tangent vector at the sample point.
   */
  [[nodiscard]] glm::vec3 getTangent(const CubicSplineSample& sample) const;

  /**
   * @brief Performs interpolation within a specific segment of the spline.
   * @param segmentIndex The index of the segment.
   * @param t The parameter t within the segment.
   * @return The interpolated point.
   */
  [[nodiscard]] glm::vec3 segmentInterpolation(int segmentIndex, float t) const;

  /**
   * @brief Returns sampled points along the spline for visualization.
   * @param numPerCurve The number of samples per curve.
   * @return A vector of sampled points.
   */
  [[nodiscard]] std::vector<glm::vec3> getLineSamples(int numPerCurve = 4) const;
};

/**
 * @class SorghumDescriptorReconstruction
 * @brief Provides methods for reconstructing sorghum structures using cubic Bezier splines.
 */
class SorghumDescriptorReconstruction {
 private:
  float theta = 50.0f;                    ///< An angle parameter used during reconstruction.
  float scale = 1.0f;                     ///< A scale parameter used during reconstruction.
  glm::vec3 center = glm::vec3(0, 0, 0);  ///< The center of the plant.
  float stemRaius = 0.01f;                ///< The radius of the stem.
  int stemSegmentsCount = 54;  ///< The total number of the segments of the stem, defining the height of the stem.

 public:
  SorghumDescriptorReconstruction(float theta = 50.0f, float scale = 1.0f, glm::vec3 center = glm::vec3(0, 0, 0),
                                  float stemRaius = 0.01f, int stemSegmentsCount = 54)
      : theta(theta), scale(scale), center(center), stemRaius(stemRaius), stemSegmentsCount(stemSegmentsCount) {
  }


  /**
   * @brief Reconstructs cubic Bezier splines from YAML content.
   * @param yaml_content The YAML content containing leaf lines.
   * @return A vector of reconstructed Bezier splines.
   */
  static std::vector<std::unordered_map<std::string, CubicBezierSpline>> ReconstructBezierSplineFromYAML(
      std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>>& yaml_content);

  /**
   * @brief Samples points from the Bezier splines for visualization.
   * @param bezierSplines The Bezier splines to sample.
   * @param numPerCurve The number of samples per curve.
   * @return A vector of sampled points.
   */
  [[nodiscard]] static std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>>
  GetLineSamplesFromBezierSplines(std::vector<std::unordered_map<std::string, CubicBezierSpline>> bezierSplines,
                                  int numPerCurve = 4);

  /**
   * @brief Extends a leaf structure to connect with the stem.
   * @param leaf The leaf descriptor to extend.
   */
  void ExtendLeafToStem(SorghumLeafDescriptor& leaf) const;

  /**
   * @brief Reconstructs the sorghum stem using sampled points.
   * @param sorghum_descriptor The descriptor containing sorghum information.
   * @param samples The number of samples along the stem.
   * @return A vector of points representing the reconstructed stem.
   */
  std::vector<glm::vec3> ReconstructSorghumStem(
  SorghumDescriptor& sorghum_descriptor) const;

  /**
   * @brief Reconstructs sorghum geometry using Bezier splines.
   * @param sorghum_descriptor The sorghum descriptor.
   * @param bezierSplines The Bezier splines representing different parts.
   * @param theta The rotation angle.
   * @param scale The scaling factor.
   * @return A vector of reconstructed sorghum structures.
   */
  std::vector<std::vector<glm::vec3>> ReconstructSorghumFromBezierSplines(
      SorghumDescriptor& sorghum_descriptor,
      const std::vector<std::unordered_map<std::string, CubicBezierSpline>>& bezierSplines) const;

  /// @brief Fills particle information using YAML points for gizmo visualization.
  /// @param leafIndex Index of the leaf to process. If -1, process all leaves.
  /// @param scale Scaling factor applied to the particle positions.
  /// @param yamlContent YAML content containing center, left, and right points.
  /// @param particleInfos Output vector to store the particle information.
  /// @param leafCount Number of leaves to process.
  /// @param lineCount Number of particle lines per leaf.
  /// @param PointsCount Number of points per line.
  static void FillYAMLPointsParticle(int leafIndex, float scale,
                                     std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>>& yamlContent,
                                     std::vector<ParticleInfo>& particleInfos, int leafCount = 1, int lineCount = 3,
                                     int PointsCount = 32);

  /// @brief Fills particle information using Bezier spline points for gizmo visualization.
  /// @param leafIndex Index of the leaf to process. If -1, process all leaves.
  /// @param scale Scaling factor applied to the particle positions.
  /// @param bezierSplinePoints Bezier spline points for particle positioning.
  /// @param particleInfos Output vector to store the particle information.
  /// @param leafCount Number of leaves to process.
  /// @param lineCount Number of particle lines per leaf.
  /// @param PointsCount Number of points per line.
  /// @param uniformSegmentCount Whether the segments have uniform counts.
  static void FillBezierSplinePointsParticle(int leafIndex, float scale,
                                             std::vector<std::vector<glm::vec3>>& bezierSplinePoints,
                                             std::vector<ParticleInfo>& particleInfos, int leafCount = 1,
                                             int lineCount = 3, int PointsCount = 32, bool uniformSegmentCount = false);

  /// @brief Fills particle information using leaf segment frames for gizmo visualization.
  /// @param leafIndex Index of the leaf to process. If -1, process all leaves.
  /// @param scale Scaling factor applied to the particle positions.
  /// @param sorghum_descriptor Descriptor containing leaf and spline segment information.
  /// @param particleInfos Output vector to store the particle information.
  /// @param leafCount Number of leaves to process.
  /// @param uniformSegmentCount Whether the segments have uniform counts.
  static void FillLeafSegmentFrameParticle(int leafIndex, float scale, SorghumDescriptor& sorghum_descriptor,
                                           std::vector<ParticleInfo>& particleInfos, int leafCount,
                                           bool uniformSegmentCount = true);
};
}  // namespace digital_agriculture_plugin
