
#pragma once

namespace evo_engine {

/**
 * @brief Represents a single cubic Bezier curve.
 */
class EVOENGINE_API BezierCurve {
 public:
  /**
   * @brief Default constructor for the BezierCurve class.
   */
  BezierCurve();

  /**
   * @brief Generates a uniform set of points along the Bezier curve.
   *
   * @param point_amount The number of points to generate.
   * @param points A reference to a vector where the generated points will be stored.
   */
  void GetUniformCurve(size_t point_amount, std::vector<glm::vec3>& points) const;

  /**
   * @brief Constructor to define a Bezier curve with specific control points.
   *
   * @param cp0 The first control point.
   * @param cp1 The second control point.
   * @param cp2 The third control point.
   * @param cp3 The fourth control point.
   */
  BezierCurve(glm::vec3 cp0, glm::vec3 cp1, glm::vec3 cp2, glm::vec3 cp3);

  /**
   * @brief Computes the point on the Bezier curve corresponding to parameter t.
   *
   * @param t A parameter between 0 and 1 representing the position on the curve.
   * @return The computed point on the curve.
   */
  [[nodiscard]] glm::vec3 GetPoint(float t) const;

  /**
   * @brief Computes the axis (derivative) of the Bezier curve at parameter t.
   *
   * @param t A parameter between 0 and 1 for which the axis is computed.
   * @return The computed axis vector.
   */
  [[nodiscard]] glm::vec3 GetAxis(float t) const;

  /**
   * @brief Gets the axis at the start of the Bezier curve.
   *
   * @return The axis vector at the start of the curve.
   */
  [[nodiscard]] glm::vec3 GetStartAxis() const;

  /**
   * @brief Gets the axis at the end of the Bezier curve.
   *
   * @return The axis vector at the end of the curve.
   */
  [[nodiscard]] glm::vec3 GetEndAxis() const;

  /**
   * @brief Computes the total length of the Bezier curve.
   *
   * @return The length of the curve.
   */
  [[nodiscard]] float GetLength() const;

  glm::vec3 p0, p1, p2, p3;  ///< The control points of the Bezier curve.
};

/**
 * @brief Represents a spline composed of multiple connected Bezier curves.
 */
class EVOENGINE_API BezierSpline {
 public:
  std::vector<BezierCurve> curves;  ///< A collection of Bezier curves forming the spline.

  /**
   * @brief Imports Bezier spline data from an input stream.
   *
   * @param stream The input file stream containing serialized spline data.
   */
  void Import(std::ifstream& stream);

  /**
   * @brief Evaluates the axis of the spline at a given point.
   *
   * @param point A float value representing the position on the spline, where 0 is the start and 1 is the end.
   * @return The axis vector at the given point.
   */
  [[nodiscard]] glm::vec3 EvaluateAxisFromCurves(float point) const;

  /**
   * @brief Evaluates the position of the spline at a given point.
   *
   * @param point A float value representing the position on the spline, where 0 is the start and 1 is the end.
   * @return The position vector at the given point.
   */
  [[nodiscard]] glm::vec3 EvaluatePointFromCurves(float point) const;

  /**
   * @brief Draws editor controls for the spline.
   */
  void Draw();

  /**
   * @brief Serializes the Bezier spline to an output YAML emitter.
   *
   * @param out The YAML emitter to store the serialized data.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes the Bezier spline from a YAML node.
   *
   * @param in The YAML node containing serialized data of the spline.
   */
  void Deserialize(const YAML::Node& in);
};
}  // namespace evo_engine
