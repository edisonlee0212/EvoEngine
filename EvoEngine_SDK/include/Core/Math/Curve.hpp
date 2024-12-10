#pragma once

namespace evo_engine {

class BezierCurve {
 public:
  BezierCurve();
  void GetUniformCurve(size_t point_amount, std::vector<glm::vec3>& points) const;
  BezierCurve(glm::vec3 cp0, glm::vec3 cp1, glm::vec3 cp2, glm::vec3 cp3);

  [[nodiscard]] glm::vec3 GetPoint(float t) const;

  [[nodiscard]] glm::vec3 GetAxis(float t) const;

  [[nodiscard]] glm::vec3 GetStartAxis() const;

  [[nodiscard]] glm::vec3 GetEndAxis() const;

  [[nodiscard]] float GetLength() const;

  glm::vec3 p0, p1, p2, p3;
};

class BezierSpline {
 public:
  std::vector<BezierCurve> curves;
  void Import(std::ifstream& stream);
  [[nodiscard]] glm::vec3 EvaluateAxisFromCurves(float point) const;
  [[nodiscard]] glm::vec3 EvaluatePointFromCurves(float point) const;
  void OnInspect();
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
};
}  // namespace evo_engine
