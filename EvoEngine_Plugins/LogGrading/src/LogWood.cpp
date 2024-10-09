#include "LogWood.hpp"

using namespace log_grading_plugin;

float radians(const float degrees) {
  return degrees * static_cast<float>(0.01745329251994329576923690768489);
}

Vec4 Vec4::Multiply(const float other) const {
  return {v0 * other, v1 * other, v2 * other, v3 * other};
}

Vec4 Vec4::Add(const Vec4& other) const {
  return {v0 + other.v0, v1 + other.v1, v2 + other.v2, v3 + other.v3};
}

Vec3 Vec3::Add(const Vec3& other) const {
  return {v0 + other.v0, v1 + other.v1, v2 + other.v2};
}

Vec3 Vec3::Subtract(const Vec3& other) const {
  return {v0 - other.v0, v1 - other.v1, v2 - other.v2};
}

Vec3 Vec3::Div(const float other) const {
  return {v0 / other, v1 / other, v2 / other};
}

Vec3 Vec3::Multiply(const float other) const {
  return {v0 * other, v1 * other, v2 * other};
}

float Vec3::Distance(const Vec3& other) const {
  return std::sqrt(std::abs(v0 * v0 - other.v0 * other.v0) + std::abs(v1 * v1 - other.v1 * other.v1) +
                   std::abs(v2 * v2 - other.v2 * other.v2));
}

float Vec3::Dot(const Vec3& other) const {
  return v0 * other.v0 + v1 * other.v1 + v2 * other.v2;
}

Vec2 Vec2::Multiply(const float rhs) const {
  return {v0 * rhs, v1 * rhs};
}

Vec2 Vec2::Add(const Vec2& rhs) const {
  return {v0 + rhs.v0, v1 + rhs.v1};
}

float LogWoodIntersection::GetCenterDistance(const float angle) const {
  assert(m_boundary.size() == 360);
  const int start_index = static_cast<int>(angle) % 360;
  float angle_int_part;
  const float a = std::modf(angle, &angle_int_part);
  const int end_index = (start_index + 1) % 360;
  return m_boundary[start_index].m_center_distance * a + m_boundary[end_index].m_center_distance * (1.f - a);
}

Vec2 LogWoodIntersection::GetBoundaryPoint(const float angle) const {
  return m_center.Add(Vec2{std::cos(radians(angle)), std::sin(radians(angle))}.Multiply(GetCenterDistance(angle)));
}

float LogWoodIntersection::GetDefectStatus(const float angle) const {
  assert(m_boundary.size() == 360);
  const int start_index = static_cast<int>(angle) % 360;
  float angle_int_part;
  const float a = std::modf(angle, &angle_int_part);
  const int end_index = (start_index + 1) % 360;
  return m_boundary[start_index].m_defect_status * a + m_boundary[end_index].m_defect_status * (1.f - a);
}

Vec4 LogWoodIntersection::GetColor(const float angle) const {
  assert(m_boundary.size() == 360);
  const int start_index = static_cast<int>(angle) % 360;
  float angle_int_part;
  const float a = std::modf(angle, &angle_int_part);
  const int end_index = (start_index + 1) % 360;
  return m_boundary[start_index].m_color.Multiply(a).Add(m_boundary[end_index].m_color.Multiply(1.f - a));
}

float LogWoodIntersection::GetAverageDistance() const {
  float ret_val = 0.f;
  for (const auto& i : m_boundary)
    ret_val += i.m_center_distance;
  return ret_val / static_cast<float>(m_boundary.size());
}

float LogWoodIntersection::GetMaxDistance() const {
  float ret_val = 0.f;
  for (const auto& i : m_boundary) {
    ret_val = std::max(ret_val, i.m_center_distance);
  }
  return ret_val;
}

float LogWoodIntersection::GetMinDistance() const {
  float ret_val = 3.402823466e+38F;
  for (const auto& i : m_boundary) {
    ret_val = std::min(ret_val, i.m_center_distance);
  }
  return ret_val;
}

Vec2 LogWood::GetSurfacePoint(const float height, const float angle) const {
  const float height_step = m_length / static_cast<float>(m_intersections.size());
  const int index = static_cast<int>(height / height_step);
  const float a = (height - height_step * static_cast<float>(index)) / height_step;
  const auto actual_angle = std::fmod(angle, 360.0f);
  if (index < static_cast<int>(m_intersections.size() - 1)) {
    return m_intersections.at(index)
        .GetBoundaryPoint(actual_angle)
        .Multiply(a)
        .Add(m_intersections.at(index + 1).GetBoundaryPoint(actual_angle).Multiply(1.f - a));
  }
  return m_intersections.back().GetBoundaryPoint(actual_angle);
}

float LogWood::GetCenterDistance(const float height, const float angle) const {
  const float height_step = m_length / static_cast<float>(m_intersections.size());
  const int index = static_cast<int>(height / height_step);
  const float a = (height - height_step * static_cast<float>(index)) / height_step;
  const auto actual_angle = std::fmod(angle, 360.0f);
  if (index < static_cast<int>(m_intersections.size() - 1)) {
    return m_intersections.at(index).GetCenterDistance(actual_angle) * a +
           m_intersections.at(index + 1).GetCenterDistance(actual_angle) * (1.f - a);
  }
  return m_intersections.back().GetCenterDistance(actual_angle);
}

float LogWood::GetDefectStatus(const float height, const float angle) const {
  const float height_step = m_length / static_cast<float>(m_intersections.size());
  const int index = static_cast<int>(height / height_step);
  const float a = (height - height_step * static_cast<float>(index)) / height_step;
  const auto actual_angle = std::fmod(angle, 360.0f);
  if (index < static_cast<int>(m_intersections.size() - 1)) {
    return m_intersections.at(index).GetDefectStatus(actual_angle) * a +
           m_intersections.at(index + 1).GetDefectStatus(actual_angle) * (1.f - a);
  }
  return m_intersections.back().GetDefectStatus(actual_angle);
}

Vec4 LogWood::GetColor(const float height, const float angle) const {
  const float height_step = m_length / static_cast<float>(m_intersections.size());
  const int index = static_cast<int>(height / height_step);
  const float a = (height - height_step * static_cast<float>(index)) / height_step;
  const auto actual_angle = std::fmod(angle, 360.0f);
  if (index < static_cast<int>(m_intersections.size() - 1)) {
    return m_intersections.at(index)
        .GetColor(actual_angle)
        .Multiply(a)
        .Add(m_intersections.at(index + 1).GetColor(actual_angle).Multiply(1.f - a));
  }
  return m_intersections.back().GetColor(actual_angle);
}

LogWoodIntersectionBoundaryPoint& LogWood::GetBoundaryPoint(const float height, const float angle) {
  const float height_step = m_length / static_cast<float>(m_intersections.size());
  const int index = static_cast<int>(height / height_step);
  const float a = (height - height_step * static_cast<float>(index)) / height_step;
  const auto actual_angle = static_cast<uint32_t>(std::fmod(angle, 360.0f));
  if (index < static_cast<int>(m_intersections.size() - 1)) {
    if (a < 0.5f) {
      return m_intersections.at(index).m_boundary.at(actual_angle);
    }
    return m_intersections.at(index + 1).m_boundary.at(actual_angle);
  }
  return m_intersections.back().m_boundary.at(actual_angle);
}

void LogWood::Rotate(int degrees) {
  while (degrees < 0.)
    degrees += 360;
  degrees = degrees % 360;
  if (degrees == 0)
    return;
  for (auto& intersection : m_intersections) {
    const float cos = std::cos(radians(static_cast<float>(degrees)));
    const float sin = std::sin(radians(static_cast<float>(degrees)));
    const Vec2 result = {intersection.m_center.v0 * cos - intersection.m_center.v1 * sin,
                         intersection.m_center.v0 * sin - intersection.m_center.v1 * cos};
    intersection.m_center = result;
    const std::vector<LogWoodIntersectionBoundaryPoint> last = {intersection.m_boundary.end() - degrees,
                                                                intersection.m_boundary.end()};
    std::copy(intersection.m_boundary.begin(), intersection.m_boundary.end() - degrees,
              intersection.m_boundary.begin() + degrees);
    std::copy(last.begin(), last.end(), intersection.m_boundary.begin());
  }
}

float LogWood::GetAverageDistance(const float height) const {
  const float height_step = m_length / static_cast<float>(m_intersections.size());
  const int index = static_cast<int>(height / height_step);
  const float a = (height - height_step * static_cast<float>(index)) / height_step;
  if (index < static_cast<int>(m_intersections.size() - 1)) {
    if (a < 0.5f) {
      return m_intersections.at(index).GetAverageDistance();
    }
    return m_intersections.at(index + 1).GetAverageDistance();
  }
  return m_intersections.back().GetAverageDistance();
}

float LogWood::GetAverageDistance() const {
  float sum_distance = 0.0f;
  for (const auto& intersection : m_intersections) {
    for (const auto& point : intersection.m_boundary)
      sum_distance += point.m_center_distance;
  }
  return sum_distance / static_cast<float>(m_intersections.size()) / 360.0f;
}

float LogWood::GetMaxAverageDistance() const {
  float ret_val = 0.f;
  for (const auto& i : m_intersections) {
    ret_val = std::max(i.GetAverageDistance(), ret_val);
  }
  return ret_val;
}

float LogWood::GetMinAverageDistance() const {
  float ret_val = 3.402823466e+38F;
  for (const auto& i : m_intersections) {
    ret_val = std::min(i.GetAverageDistance(), ret_val);
  }
  return ret_val;
}

float LogWood::GetMaxDistance() const {
  float ret_val = 0.f;
  for (const auto& i : m_intersections) {
    ret_val = std::max(i.GetMaxDistance(), ret_val);
  }
  return ret_val;
}

float LogWood::GetMinDistance() const {
  float ret_val = 3.402823466e+38F;
  for (const auto& i : m_intersections) {
    ret_val = std::min(i.GetMinDistance(), ret_val);
  }
  return ret_val;
}

void LogWood::MarkDefectRegion(const float height, const float angle, const float height_range,
                               const float angle_range) {
  const float height_step = m_length / static_cast<float>(m_intersections.size());
  const int height_step_size = static_cast<int>(height_range / height_step);
  for (int y_index = -height_step_size; y_index <= height_step_size; y_index++) {
    const auto sin_val = std::abs(static_cast<float>(y_index)) / static_cast<float>(height_step_size);
    const auto sin_angle = std::asin(sin_val);
    const float max_angle_range = std::cos(sin_angle);

    for (int x_index = -static_cast<int>(angle_range); x_index <= static_cast<int>(angle_range); x_index++) {
      const int actual_y_index = y_index + static_cast<int>(height / height_step);
      if (actual_y_index < 0 || actual_y_index >= static_cast<int>(m_intersections.size()))
        continue;
      if (std::abs(static_cast<float>(x_index)) / angle_range > max_angle_range)
        continue;
      const auto actual_x_index = static_cast<int>(360.0f + static_cast<float>(x_index) + angle) % 360;
      m_intersections.at(actual_y_index).m_boundary.at(actual_x_index).m_defect_status = 1.0f;
    }
  }
}

void LogWood::EraseDefectRegion(const float height, const float angle, const float height_range,
                                const float angle_range) {
  const float height_step = m_length / static_cast<float>(m_intersections.size());
  const int height_step_size = static_cast<int>(height_range / height_step);
  for (int y_index = -height_step_size; y_index <= height_step_size; y_index++) {
    const auto sin_val = std::abs(static_cast<float>(y_index)) / static_cast<float>(height_step_size);
    const auto sin_angle = std::asin(sin_val);
    const float max_angle_range = std::cos(sin_angle);
    for (int x_index = -static_cast<int>(angle_range); x_index <= static_cast<int>(angle_range); x_index++) {
      const int actual_y_index = y_index + static_cast<int>(height / height_step);
      if (actual_y_index < 0 || actual_y_index >= static_cast<int>(m_intersections.size()))
        continue;
      if (std::abs(static_cast<float>(x_index)) / angle_range > max_angle_range)
        continue;
      const auto actual_x_index = static_cast<int>(360.0f + static_cast<float>(x_index) + angle) % 360;
      m_intersections.at(actual_y_index).m_boundary.at(actual_x_index).m_defect_status = 0.0f;
    }
  }
}

void LogWood::ClearDefects() {
  for (auto& intersection : m_intersections) {
    for (auto& boundary_point : intersection.m_boundary) {
      boundary_point.m_defect_status = 0.0f;
      boundary_point.m_color = Vec4{212.f / 255, 175.f / 255, 55.f / 255, 1};
    }
  }
}

bool LogWood::RayCastSelection(const glm::mat4& transform, const float point_distance_threshold, const Ray& ray,
                               float& height, float& angle) const {
  float min_distance = 3.402823466e+38F;
  bool found = false;
  const float height_step = m_length / static_cast<float>(m_intersections.size());
  for (int y_index = 0; y_index < static_cast<int>(m_intersections.size()); y_index++) {
    const auto test_height = static_cast<float>(y_index) * height_step;
    for (int x_index = 0; x_index < 360; x_index++) {
      const auto surface_point = GetSurfacePoint(test_height, static_cast<float>(x_index));
      const glm::vec3 position =
          (transform * glm::translate(glm::vec3(surface_point.v0, test_height, surface_point.v1)))[3];
      glm::vec3 closest_point = Ray::ClosestPointOnLine(position, ray.start, ray.start + ray.direction * 10000.0f);
      const float point_distance = glm::distance(closest_point, position);
      if (const float distance_to_start = glm::distance(ray.start, position);
          distance_to_start < min_distance && point_distance < point_distance_threshold) {
        min_distance = distance_to_start;
        height = test_height;
        angle = static_cast<float>(x_index);
        found = true;
      }
    }
  }
  return found;
}

std::vector<LogGradeFaceCutting> CalculateCuttings(const float trim, const std::vector<bool>& defect_marks,
                                                   const float height_step, const float min_distance,
                                                   const float start_height) {
  int last_defect_index = 0.0f;
  std::vector<LogGradeFaceCutting> cuttings{};
  for (int intersection_index = 0; intersection_index < static_cast<int>(defect_marks.size()); intersection_index++) {
    if (defect_marks[intersection_index] || intersection_index == static_cast<int>(defect_marks.size() - 1)) {
      if (height_step * static_cast<float>(intersection_index - last_defect_index) >= min_distance + trim + trim) {
        LogGradeFaceCutting cutting;
        cutting.m_start_in_meters = height_step * static_cast<float>(last_defect_index) + start_height + trim;
        cutting.m_end_in_meters = height_step * static_cast<float>(intersection_index) + start_height - trim;
        cuttings.emplace_back(cutting);
      }
      last_defect_index = intersection_index;
    }
  }
  return cuttings;
}

bool TestCutting(const std::vector<LogGradeFaceCutting>& cuttings, const int max_number, const float min_proportion,
                 const float log_length, float& min_cutting_length, float& proportion) {
  const auto cutting_number = static_cast<int>(cuttings.size());
  if (cutting_number == 0)
    return false;
  float cutting_sum_length = 0.0f;
  min_cutting_length = 3.402823466e+38F;
  for (const auto& cutting : cuttings) {
    const float length = cutting.m_end_in_meters - cutting.m_start_in_meters;
    cutting_sum_length += length;
    min_cutting_length = std::min(min_cutting_length, length);
  }
  proportion = cutting_sum_length / log_length;
  if (cutting_number <= max_number && proportion >= min_proportion) {
    return true;
  }
  return false;
}

float LogWood::InchesToMeters(const float inches) {
  return inches * 0.0254f;
}

float LogWood::FeetToMeter(const float feet) {
  return feet * 0.3048f;
}

float LogWood::MetersToInches(const float meters) {
  return meters * 39.3701f;
}

float LogWood::MetersToFeet(const float meters) {
  return meters * 3.28084f;
}

void LogWood::CalculateGradingData(std::vector<LogGrading>& log_grading) const {
  std::vector<LogGrading> results{};
  const float intersection_length = m_length / static_cast<float>(m_intersections.size());
  const int grading_section_intersection_count = static_cast<int>(
      std::min(std::ceil(FeetToMeter(12) / intersection_length), static_cast<float>(m_intersections.size()) - 1));

  float cutting_trim = InchesToMeters(3);
  for (int start_intersection_index = 0;
       start_intersection_index < static_cast<int>(m_intersections.size()) - grading_section_intersection_count;
       start_intersection_index++) {
    for (int angle_offset = 0; angle_offset < 90; angle_offset++) {
      results.emplace_back();
      auto& temp_log_grading = results.back();
      temp_log_grading.m_angle_offset = angle_offset;
      // TODO: Calculate Scaling Diameter correctly.
      temp_log_grading.m_scaling_diameter_in_meters = GetMinAverageDistance() * 2.f;
      temp_log_grading.m_start_height_in_meters = static_cast<float>(start_intersection_index) * intersection_length;
      temp_log_grading.m_start_intersection_index = start_intersection_index;
      temp_log_grading.m_length_without_trim_in_meters =
          static_cast<float>(grading_section_intersection_count) * intersection_length;
      const float d = std::round(MetersToInches(temp_log_grading.m_scaling_diameter_in_meters));
      const float l = std::round(MetersToFeet(temp_log_grading.m_length_without_trim_in_meters));
      temp_log_grading.m_doyle_rule_scale = (d - 4.f) * (d - 4.f) * l / 16.f;
      temp_log_grading.m_scribner_rule_scale = (0.79f * d * d - d * 2.f - 4.f) * l / 16.f;
      temp_log_grading.m_international_rule_scale =
          static_cast<float>(0.04976191 * l * d * d + 0.006220239 * l * l * d - 0.1854762 * l * d +
                             0.0002591767 * l * l * l - 0.01159226 * l * l + 0.04222222 * l);

      if (l <= 10) {
        temp_log_grading.m_sweep_deduction = std::max(0.0f, (m_sweep_in_inches - 1.f) / d);
      } else if (l <= 13) {
        temp_log_grading.m_sweep_deduction = std::max(0.0f, (m_sweep_in_inches - 1.5f) / d);
      } else {
        temp_log_grading.m_sweep_deduction = std::max(0.0f, (m_sweep_in_inches - 2.f) / d);
      }
      temp_log_grading.m_crook_deduction = m_crook_cl_in_inches / d * m_crook_cl_in_feet / l;

      for (int face_index = 0; face_index < 4; face_index++) {
        auto& face = temp_log_grading.m_faces[face_index];
        face.m_start_angle = (angle_offset + face_index * 90) % 360;
        face.m_end_angle = (angle_offset + face_index * 90 + 90) % 360;
        std::vector<bool> defect_marks;
        defect_marks.resize(grading_section_intersection_count);
        for (int intersection_index = 0; intersection_index < grading_section_intersection_count;
             intersection_index++) {
          const auto& intersection = m_intersections[intersection_index + start_intersection_index];
          for (int angle = 0; angle < 90; angle++) {
            const int actual_angle = (face.m_start_angle + angle) % 360;
            if (intersection.m_boundary[actual_angle].m_defect_status != 0.0f) {
              defect_marks[intersection_index] = true;
              break;
            }
          }
        }

        bool f1_possible = true;

        if (!m_sound_defect) {
          if (temp_log_grading.m_crook_deduction > 0.15f || temp_log_grading.m_sweep_deduction > 0.15f) {
            f1_possible = false;
          }
        } else {
          if (temp_log_grading.m_crook_deduction > 0.1f || temp_log_grading.m_sweep_deduction > 0.1f) {
            f1_possible = false;
          }
        }

        bool f2_possible = true;
        if (!f1_possible) {
          if (!m_sound_defect) {
            if (temp_log_grading.m_crook_deduction > 0.3f || temp_log_grading.m_sweep_deduction > 0.3f) {
              f2_possible = false;
            }
          } else {
            if (temp_log_grading.m_crook_deduction > 0.2f || temp_log_grading.m_sweep_deduction > 0.2f) {
              f2_possible = false;
            }
          }
        }
        bool f3_possible = true;
        if (!f2_possible) {
          if (!m_sound_defect) {
            if (temp_log_grading.m_crook_deduction > 0.5f || temp_log_grading.m_sweep_deduction > 0.5f) {
              f3_possible = false;
            }
          } else {
            if (temp_log_grading.m_crook_deduction > 0.35f || temp_log_grading.m_sweep_deduction > 0.35f) {
              f3_possible = false;
            }
          }
        }
        bool succeed = false;
        if (f1_possible && m_butt && temp_log_grading.m_scaling_diameter_in_meters >= InchesToMeters(13) &&
            temp_log_grading.m_scaling_diameter_in_meters <= InchesToMeters(16) &&
            temp_log_grading.m_length_without_trim_in_meters >= FeetToMeter(10)) {
          // F1: Butt, Scaling diameter 13-15(16), Length 10+
          const auto cuttings7 = CalculateCuttings(cutting_trim, defect_marks, intersection_length, FeetToMeter(7),
                                                   temp_log_grading.m_start_height_in_meters);
          float min_cutting_length = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings7, 2, 5.0f / 6.0f, temp_log_grading.m_length_without_trim_in_meters,
                                min_cutting_length, proportion);

          if (succeed) {
            face.m_face_grade = 1;
            face.m_clear_cuttings = cuttings7;
            face.m_clear_cutting_min_length_in_meters = min_cutting_length;
            face.m_clear_cutting_min_proportion = proportion;
          }
        }
        if (f1_possible && !succeed && temp_log_grading.m_scaling_diameter_in_meters > InchesToMeters(16) &&
            temp_log_grading.m_scaling_diameter_in_meters <= InchesToMeters(20) &&
            temp_log_grading.m_length_without_trim_in_meters >= FeetToMeter(10)) {
          // F1: Butt & uppers, Scaling diameter 16-19(20), Length 10+
          const auto cuttings5 = CalculateCuttings(cutting_trim, defect_marks, intersection_length, FeetToMeter(5),
                                                   temp_log_grading.m_start_height_in_meters);
          float min_cutting_length = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings5, 2, 5.0f / 6.0f, temp_log_grading.m_length_without_trim_in_meters,
                                min_cutting_length, proportion);

          if (succeed) {
            face.m_face_grade = 2;
            face.m_clear_cuttings = cuttings5;
            face.m_clear_cutting_min_length_in_meters = min_cutting_length;
            face.m_clear_cutting_min_proportion = proportion;
          }
        }
        const auto cuttings3 = CalculateCuttings(cutting_trim, defect_marks, intersection_length, FeetToMeter(3),
                                                 temp_log_grading.m_start_height_in_meters);
        if (f1_possible && !succeed && temp_log_grading.m_scaling_diameter_in_meters > InchesToMeters(20) &&
            temp_log_grading.m_length_without_trim_in_meters >= FeetToMeter(10)) {
          // F1: Butt & uppers, Scaling diameter 20+, Length 10+
          // const auto cuttings3 = CalculateCuttings(defectMarks, height_step, FeetToMeter(3));
          float min_cutting_length = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings3, 2, 5.0f / 6.0f, temp_log_grading.m_length_without_trim_in_meters,
                                min_cutting_length, proportion);

          if (succeed) {
            face.m_face_grade = 3;
            face.m_clear_cuttings = cuttings3;
            face.m_clear_cutting_min_length_in_meters = min_cutting_length;
            face.m_clear_cutting_min_proportion = proportion;
          }
        }

        if (f2_possible && !succeed && temp_log_grading.m_scaling_diameter_in_meters > InchesToMeters(11) &&
            temp_log_grading.m_length_without_trim_in_meters >= FeetToMeter(10)) {
          // F2: Butt & uppers, Scaling diameter 11+, Length 10+
          float min_cutting_length = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings3, 2, 2.0f / 3.0f, temp_log_grading.m_length_without_trim_in_meters,
                                min_cutting_length, proportion);

          if (succeed) {
            face.m_face_grade = 4;
            face.m_clear_cuttings = cuttings3;
            face.m_clear_cutting_min_length_in_meters = min_cutting_length;
            face.m_clear_cutting_min_proportion = proportion;
          }
        }
        if (f2_possible && !succeed && temp_log_grading.m_scaling_diameter_in_meters > InchesToMeters(12) &&
            temp_log_grading.m_length_without_trim_in_meters > FeetToMeter(8) &&
            temp_log_grading.m_length_without_trim_in_meters <= FeetToMeter(10)) {
          // F2: Butt & uppers, Scaling diameter 12+, Length 8-9(10)
          float min_cutting_length = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings3, 2, 3.0f / 4.0f, temp_log_grading.m_length_without_trim_in_meters,
                                min_cutting_length, proportion);
          if (succeed) {
            if (!m_sound_defect) {
              if (temp_log_grading.m_crook_deduction > 0.3f || temp_log_grading.m_sweep_deduction > 0.3f) {
                succeed = false;
              }
            } else {
              if (temp_log_grading.m_crook_deduction > 0.2f || temp_log_grading.m_sweep_deduction > 0.2f) {
                succeed = false;
              }
            }
          }
          if (succeed) {
            face.m_face_grade = 5;
            face.m_clear_cuttings = cuttings3;
            face.m_clear_cutting_min_length_in_meters = min_cutting_length;
            face.m_clear_cutting_min_proportion = proportion;
          }
        }
        if (f2_possible && !succeed && temp_log_grading.m_scaling_diameter_in_meters > InchesToMeters(12) &&
            temp_log_grading.m_length_without_trim_in_meters > FeetToMeter(10) &&
            temp_log_grading.m_length_without_trim_in_meters <= FeetToMeter(12)) {
          // F2: Butt & uppers, Scaling diameter 12+, Length 10-11(12)
          float min_cutting_length = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings3, 2, 2.0f / 3.0f, temp_log_grading.m_length_without_trim_in_meters,
                                min_cutting_length, proportion);
          if (succeed) {
            if (!m_sound_defect) {
              if (temp_log_grading.m_crook_deduction > 0.3f || temp_log_grading.m_sweep_deduction > 0.3f) {
                succeed = false;
              }
            } else {
              if (temp_log_grading.m_crook_deduction > 0.2f || temp_log_grading.m_sweep_deduction > 0.2f) {
                succeed = false;
              }
            }
          }
          if (succeed) {
            face.m_face_grade = 6;
            face.m_clear_cuttings = cuttings3;
            face.m_clear_cutting_min_length_in_meters = min_cutting_length;
            face.m_clear_cutting_min_proportion = proportion;
          }
        }
        if (f2_possible && !succeed && temp_log_grading.m_scaling_diameter_in_meters > InchesToMeters(12) &&
            temp_log_grading.m_length_without_trim_in_meters > FeetToMeter(12)) {
          // F2: Butt & uppers, Scaling diameter 12+, Length 12+
          float min_cutting_length = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings3, 3, 2.0f / 3.0f, temp_log_grading.m_length_without_trim_in_meters,
                                min_cutting_length, proportion);
          if (succeed) {
            if (!m_sound_defect) {
              if (temp_log_grading.m_crook_deduction > 0.3f || temp_log_grading.m_sweep_deduction > 0.3f) {
                succeed = false;
              }
            } else {
              if (temp_log_grading.m_crook_deduction > 0.2f || temp_log_grading.m_sweep_deduction > 0.2f) {
                succeed = false;
              }
            }
          }
          if (succeed) {
            face.m_face_grade = 7;
            face.m_clear_cuttings = cuttings3;
            face.m_clear_cutting_min_length_in_meters = min_cutting_length;
            face.m_clear_cutting_min_proportion = proportion;
          }
        }
        const auto cuttings2 = CalculateCuttings(cutting_trim, defect_marks, intersection_length, FeetToMeter(2),
                                                 temp_log_grading.m_start_height_in_meters);
        if (f3_possible && !succeed && temp_log_grading.m_scaling_diameter_in_meters > InchesToMeters(8) &&
            temp_log_grading.m_length_without_trim_in_meters > FeetToMeter(8)) {
          // F3: Butt & uppers, Scaling diameter 8+, Length 8+
          float min_cutting_length = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings2, 999, 1.0f / 2.0f, temp_log_grading.m_length_without_trim_in_meters,
                                min_cutting_length, proportion);

          if (succeed) {
            face.m_face_grade = 8;
            face.m_clear_cuttings = cuttings2;
            face.m_clear_cutting_min_length_in_meters = min_cutting_length;
            face.m_clear_cutting_min_proportion = proportion;
          }
        }
        if (!succeed) {
          float min_cutting_length = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings2, 999, 0.f, temp_log_grading.m_length_without_trim_in_meters,
                                min_cutting_length, proportion);
          face.m_face_grade = 9;
          face.m_clear_cuttings = cuttings2;
          face.m_clear_cutting_min_length_in_meters = min_cutting_length;
          face.m_clear_cutting_min_proportion = proportion;
        }
      }
      int worst_grade_index = -1;
      int worst_grade = 0;
      for (int i = 0; i < 4; i++) {
        if (temp_log_grading.m_faces[i].m_face_grade > worst_grade) {
          worst_grade_index = i;
          worst_grade = temp_log_grading.m_faces[i].m_face_grade;
        }
      }
      int second_worst_grade_index = -1;
      worst_grade = 0;
      for (int i = 0; i < 4; i++) {
        if (i != worst_grade_index && temp_log_grading.m_faces[i].m_face_grade > worst_grade) {
          second_worst_grade_index = i;
          worst_grade = temp_log_grading.m_faces[i].m_face_grade;
        }
      }
      temp_log_grading.m_grade_determine_face_index = second_worst_grade_index;
      temp_log_grading.m_grade = worst_grade;
    }
  }
  int best_grade = INT_MAX;
  for (const auto& result : results) {
    if (result.m_grade < best_grade) {
      best_grade = result.m_grade;
      log_grading.clear();
      log_grading.emplace_back(result);
    } else if (result.m_grade == best_grade) {
      log_grading.emplace_back(result);
    }
  }
}

void LogWood::ColorBasedOnGrading(const LogGrading& log_grading_data) {
  const float intersection_length = m_length / static_cast<float>(m_intersections.size());
  const int grading_section_intersection_count = static_cast<int>(std::ceil(FeetToMeter(12) / intersection_length));
  for (const auto& face : log_grading_data.m_faces) {
    for (int intersection_index = 0; intersection_index < m_intersections.size(); intersection_index++) {
      auto& intersection = m_intersections[intersection_index];
      for (int angle = 0; angle < 90; angle++) {
        const int actual_angle = (face.m_start_angle + angle) % 360;
        auto& point = intersection.m_boundary[actual_angle];

        if (point.m_defect_status != 0.0f) {
          point.m_color = Vec4{1, 0, 0, 1};
          continue;
        }
        const float height = intersection_length * static_cast<float>(intersection_index);
        bool is_cutting = false;
        for (const auto& cutting : face.m_clear_cuttings) {
          if (height >= cutting.m_start_in_meters && height <= cutting.m_end_in_meters) {
            if (face.m_face_grade <= 3) {
              point.m_color = Vec4{212.f / 255, 175.f / 255, 55.f / 255, 1};
            } else if (face.m_face_grade <= 7) {
              point.m_color = Vec4{170.f / 255, 169.f / 255, 173.f / 255, 1};
            } else if (face.m_face_grade <= 8) {
              point.m_color = Vec4{131.f / 255, 105.f / 255, 83.f / 255, 1};
            } else {
              point.m_color = Vec4{0.1f, 0.1f, 0.1f, 1};
            }

            is_cutting = true;
            break;
          }
        }
        if (!is_cutting) {
          if (intersection_index < log_grading_data.m_start_intersection_index ||
              intersection_index > log_grading_data.m_start_intersection_index + grading_section_intersection_count) {
            point.m_color = Vec4{0, 0, 0, 1};
          } else {
            point.m_color = Vec4{0.2f, 0.0f, 0.0f, 1};
          }
        }
      }
    }
  }
}
