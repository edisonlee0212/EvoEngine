#pragma once
using namespace evo_engine;
namespace eco_sys_lab {
struct LogWoodIntersectionBoundaryPoint {
  float m_centerDistance = 0.0f;
  float m_defectStatus = 0.0f;
  glm::vec4 m_color = glm::vec4(1.0f);
};
class LogWoodIntersection {
 public:
  glm::vec2 m_center = glm::vec2(0.0f);
  std::vector<LogWoodIntersectionBoundaryPoint> m_boundary{};
  [[nodiscard]] float GetCenterDistance(float angle) const;
  [[nodiscard]] glm::vec2 GetBoundaryPoint(float angle) const;
  [[nodiscard]] float GetDefectStatus(float angle) const;
  [[nodiscard]] glm::vec4 GetColor(float angle) const;
  [[nodiscard]] float GetAverageDistance() const;
  [[nodiscard]] float GetMaxDistance() const;
  [[nodiscard]] float GetMinDistance() const;
};

struct LogGradeFaceCutting {
  float m_startInMeters = 0;
  float m_endInMeters = 0;
};

struct LogGradingFace {
  int m_faceIndex = 0;
  int m_startAngle = 0;
  int m_endAngle = 0;
  int m_faceGrade = 0;
  std::vector<LogGradeFaceCutting> m_clearCuttings{};
  float m_clearCuttingMinLengthInMeters = 0;
  float m_clearCuttingMinProportion = 0;
};

struct LogGrading {
  float m_doyleRuleScale = 0.0f;
  float m_scribnerRuleScale = 0.0f;
  float m_internationalRuleScale = 0.0f;

  float m_crookDeduction = 0.0f;
  float m_sweepDeduction = 0.0f;
  int m_angleOffset = 0;
  float m_lengthWithoutTrimInMeters = 0.0f;
  float m_scalingDiameterInMeters = 0;
  int m_gradeDetermineFaceIndex = 0;
  int m_grade = 0;
  int m_startIntersectionIndex = 0;
  float m_startHeightInMeters = 0.0f;
  LogGradingFace m_faces[4]{};
};

class LogWood {
 public:
  static float InchesToMeters(float inches);
  static float FeetToMeter(float feet);
  static float MetersToInches(float meters);
  static float MetersToFeet(float meters);
  bool m_butt = true;
  float m_length = 0.0f;
  float m_sweepInInches = 0.0f;
  float m_crookCInInches = 0.0f;
  float m_crookCLInFeet = 0.0f;
  bool m_soundDefect = false;
  std::vector<LogWoodIntersection> m_intersections;
  [[nodiscard]] glm::vec2 GetSurfacePoint(float height, float angle) const;
  [[nodiscard]] float GetCenterDistance(float height, float angle) const;
  [[nodiscard]] float GetDefectStatus(float height, float angle) const;
  [[nodiscard]] glm::vec4 GetColor(float height, float angle) const;

  [[nodiscard]] LogWoodIntersectionBoundaryPoint& GetBoundaryPoint(float height, float angle);
  void Rotate(int degrees);
  [[nodiscard]] float GetAverageDistance(float height) const;
  [[nodiscard]] float GetAverageDistance() const;
  [[nodiscard]] float GetMaxAverageDistance() const;
  [[nodiscard]] float GetMinAverageDistance() const;
  [[nodiscard]] float GetMaxDistance() const;
  [[nodiscard]] float GetMinDistance() const;
  void MarkDefectRegion(float height, float angle, float heightRange, float angleRange);
  void EraseDefectRegion(float height, float angle, float heightRange, float angleRange);
  void ClearDefects();
  [[nodiscard]] bool RayCastSelection(const glm::mat4& transform, float pointDistanceThreshold, const Ray& ray,
                                      float& height, float& angle) const;

  void CalculateGradingData(std::vector<LogGrading>& logGrading) const;
  void ColorBasedOnGrading(const LogGrading& logGradingData);
};
}  // namespace eco_sys_lab