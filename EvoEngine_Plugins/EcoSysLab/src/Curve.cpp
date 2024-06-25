#include "Curve.hpp"

using namespace eco_sys_lab;

void ICurve::GetUniformCurve(size_t point_amount, std::vector<glm::vec3>& points) const {
  float step = 1.0f / (point_amount - 1);
  for (size_t i = 0; i <= point_amount; i++) {
    points.push_back(GetPoint(step * i));
  }
}

BezierCurve::BezierCurve() {
}

BezierCurve::BezierCurve(glm::vec3 cp0, glm::vec3 cp1, glm::vec3 cp2, glm::vec3 cp3)
    : ICurve(), p0(cp0), p1(cp1), p2(cp2), p3(cp3) {
}

glm::vec3 BezierCurve::GetPoint(float t) const {
  t = glm::clamp(t, 0.f, 1.f);
  return p0 * (1.0f - t) * (1.0f - t) * (1.0f - t) + p1 * 3.0f * t * (1.0f - t) * (1.0f - t) +
         p2 * 3.0f * t * t * (1.0f - t) + p3 * t * t * t;
}

glm::vec3 BezierCurve::GetAxis(float t) const {
  t = glm::clamp(t, 0.f, 1.f);
  float mt = 1.0f - t;
  return (p1 - p0) * 3.0f * mt * mt + 6.0f * t * mt * (p2 - p1) + 3.0f * t * t * (p3 - p2);
}

glm::vec3 BezierCurve::GetStartAxis() const {
  return glm::normalize(p1 - p0);
}

glm::vec3 BezierCurve::GetEndAxis() const {
  return glm::normalize(p3 - p2);
}

float BezierCurve::GetLength() const {
  return glm::distance(p0, p3);
}

glm::vec3 BezierSpline::EvaluatePointFromCurves(float point) const {
  const float spline_u = glm::clamp(point, 0.0f, 1.0f) * float(curves.size());

  // Decompose the global u coordinate on the spline
  float integer_part;
  const float fractional_part = modff(spline_u, &integer_part);

  auto curve_index = static_cast<int>(integer_part);
  auto curve_u = fractional_part;

  // If evaluating the very last point on the spline
  if (curve_index == curves.size() && curve_u <= 0.0f) {
    // Flip to the end of the last patch
    curve_index--;
    curve_u = 1.0f;
  }
  return curves.at(curve_index).GetPoint(curve_u);
}
void BezierSpline::OnInspect() {
  int size = curves.size();
  if (ImGui::DragInt("Size of curves", &size, 0, 10)) {
    size = glm::clamp(size, 0, 10);
    curves.resize(size);
  }
  if (ImGui::TreeNode("Curves")) {
    int index = 1;
    for (auto& curve : curves) {
      if (ImGui::TreeNode(("Curve " + std::to_string(index)).c_str())) {
        ImGui::DragFloat3("CP0", &curve.p0.x, 0.01f);
        ImGui::DragFloat3("CP1", &curve.p1.x, 0.01f);
        ImGui::DragFloat3("CP2", &curve.p2.x, 0.01f);
        ImGui::DragFloat3("CP3", &curve.p3.x, 0.01f);
        ImGui::TreePop();
      }
      index++;
    }
    ImGui::TreePop();
  }
}
void BezierSpline::Serialize(YAML::Emitter& out) const {
  if (!curves.empty()) {
    out << YAML::Key << "curves" << YAML::Value << YAML::BeginSeq;
    for (const auto& i : curves) {
      out << YAML::BeginMap;
      out << YAML::Key << "p0" << YAML::Value << i.p0;
      out << YAML::Key << "p1" << YAML::Value << i.p1;
      out << YAML::Key << "p2" << YAML::Value << i.p2;
      out << YAML::Key << "p3" << YAML::Value << i.p3;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
}
void BezierSpline::Deserialize(const YAML::Node& in) {
  if (in["curves"]) {
    curves.clear();
    for (const auto& i_curve : in["curves"]) {
      curves.emplace_back(i_curve["p0"].as<glm::vec3>(), i_curve["p1"].as<glm::vec3>(),
                            i_curve["p2"].as<glm::vec3>(), i_curve["p3"].as<glm::vec3>());
    }
  }
}

void BezierSpline::Import(std::ifstream& stream) {
  int curve_amount;
  stream >> curve_amount;
  curves.clear();
  for (int i = 0; i < curve_amount; i++) {
    glm::vec3 cp[4];
    float x, y, z;
    for (auto& j : cp) {
      stream >> x >> z >> y;
      j = glm::vec3(x, y, z);
    }
    curves.emplace_back(cp[0], cp[1], cp[2], cp[3]);
  }
}

glm::vec3 BezierSpline::EvaluateAxisFromCurves(float point) const {
  const float spline_u = glm::clamp(point, 0.0f, 1.0f) * float(curves.size());

  // Decompose the global u coordinate on the spline
  float integer_part;
  const float fractional_part = modff(spline_u, &integer_part);

  auto curve_index = static_cast<int>(integer_part);
  auto curve_u = fractional_part;

  // If evaluating the very last point on the spline
  if (curve_index == curves.size() && curve_u <= 0.0f) {
    // Flip to the end of the last patch
    curve_index--;
    curve_u = 1.0f;
  }
  return curves.at(curve_index).GetAxis(curve_u);
}
