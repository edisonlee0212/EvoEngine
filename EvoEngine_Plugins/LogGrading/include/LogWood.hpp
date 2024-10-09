#pragma once
using namespace evo_engine;
namespace log_grading_plugin {
struct Vec4 {
  float v0 = 0;
  float v1 = 0;
  float v2 = 0;
  float v3 = 0;
  Vec4 Multiply(float other) const;

  Vec4 Add(const Vec4& other) const;
};
struct Vec3 {
  float v0 = 0;
  float v1 = 0;
  float v2 = 0;

  Vec3 Add(const Vec3& other) const;

  Vec3 Subtract(const Vec3& other) const;

  Vec3 Div(float other) const;

  Vec3 Multiply(float other) const;

  float Distance(const Vec3& other) const;

  float Dot(const Vec3& other) const;
};
struct Vec2 {
  float v0 = 0;
  float v1 = 0;

  Vec2 Multiply(float rhs) const;

  Vec2 Add(const Vec2& rhs) const;
};

struct LogWoodIntersectionBoundaryPoint {
  float m_center_distance = 0.0f;
  float m_defect_status = 0.0f;
  Vec4 m_color = Vec4{1.0f, 1.0f, 1.0f, 1.0f};
};
class LogWoodIntersection {
 public:
  Vec2 m_center = Vec2{0.0f, 0.0f};
  std::vector<LogWoodIntersectionBoundaryPoint> m_boundary{};
  float GetCenterDistance(float angle) const;
  Vec2 GetBoundaryPoint(float angle) const;
  float GetDefectStatus(float angle) const;
  Vec4 GetColor(float angle) const;
  float GetAverageDistance() const;
  float GetMaxDistance() const;
  float GetMinDistance() const;
};

struct LogGradeFaceCutting {
  float m_start_in_meters = 0;
  float m_end_in_meters = 0;
};

struct LogGradingFace {
  int m_face_index = 0;
  int m_start_angle = 0;
  int m_end_angle = 0;
  int m_face_grade = 0;
  std::vector<LogGradeFaceCutting> m_clear_cuttings{};
  float m_clear_cutting_min_length_in_meters = 0;
  float m_clear_cutting_min_proportion = 0;
};

struct LogGrading {
  float m_doyle_rule_scale = 0.0f;
  float m_scribner_rule_scale = 0.0f;
  float m_international_rule_scale = 0.0f;

  float m_crook_deduction = 0.0f;
  float m_sweep_deduction = 0.0f;
  int m_angle_offset = 0;
  float m_length_without_trim_in_meters = 0.0f;
  float m_scaling_diameter_in_meters = 0;
  int m_grade_determine_face_index = 0;
  int m_grade = 0;
  int m_start_intersection_index = 0;
  float m_start_height_in_meters = 0.0f;
  LogGradingFace m_faces[4]{};
};

class LogWood {
 public:
  static float InchesToMeters(float inches);
  static float FeetToMeter(float feet);
  static float MetersToInches(float meters);
  static float MetersToFeet(float meters);
  static Vec3 ClosestPointOnLine(const Vec3& point, const Vec3& a, const Vec3& b) {
    const float line_length = a.Distance(b);
    const Vec3 vector = point.Subtract(a);
    const Vec3 line_direction = b.Subtract(a).Div(line_length);

    // Project Vector to LineDirection to get the distance of point from a
    const float distance = vector.Dot(line_direction);
    return a.Add(line_direction.Multiply(distance));
  }

  bool m_butt = true;
  float m_length = 0.0f;
  float m_sweep_in_inches = 0.0f;
  float m_crook_cl_in_inches = 0.0f;
  float m_crook_cl_in_feet = 0.0f;
  bool m_sound_defect = false;
  std::vector<LogWoodIntersection> m_intersections;
  Vec2 GetSurfacePoint(float height, float angle) const;
  float GetCenterDistance(float height, float angle) const;
  float GetDefectStatus(float height, float angle) const;
  Vec4 GetColor(float height, float angle) const;

  LogWoodIntersectionBoundaryPoint& GetBoundaryPoint(float height, float angle);
  void Rotate(int degrees);
  float GetAverageDistance(float height) const;
  float GetAverageDistance() const;
  float GetMaxAverageDistance() const;
  float GetMinAverageDistance() const;
  float GetMaxDistance() const;
  float GetMinDistance() const;
  void MarkDefectRegion(float height, float angle, float height_range, float angle_range);
  void EraseDefectRegion(float height, float angle, float height_range, float angle_range);
  void ClearDefects();
  bool RayCastSelection(const glm::mat4& transform, float point_distance_threshold, const Ray& ray, float& height,
                        float& angle) const;

  void CalculateGradingData(std::vector<LogGrading>& log_grading) const;
  void ColorBasedOnGrading(const LogGrading& log_grading_data);
};
}  // namespace log_grading_plugin