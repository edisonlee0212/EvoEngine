#include "Plot2D.hpp"
#include "Serialization.hpp"
using namespace evo_engine;

std::vector<glm::vec2>& Curve2D::UnsafeGetValues() {
  return values_;
}
void Curve2D::SetTangent(bool value) {
  tangent_ = value;
  Clear();
}
bool Curve2D::IsTangent() const {
  return tangent_;
}
float Curve2D::GetValue(float x, const unsigned iteration) const {
  x = glm::clamp(x, 0.0f, 1.0f);
  if (tangent_) {
    const int point_size = values_.size() / 3;
    for (int i = 0; i < point_size - 1; i++) {
      auto& prev = values_[i * 3 + 1];
      auto& next = values_[i * 3 + 4];
      if (x == prev.x) {
        return prev.y;
      }
      if (x > prev.x && x < next.x) {
        const float real_x = (x - prev.x) / (next.x - prev.x);
        float upper = 1.0f;
        float lower = 0.0f;
        float temp_t = 0.5f;
        for (unsigned iteration_index = 0; iteration_index < iteration; iteration_index++) {
          const float temp_t1 = 1.0f - temp_t;
          const float global_x = temp_t1 * temp_t1 * temp_t1 * prev.x +
                                 3.0f * temp_t1 * temp_t1 * temp_t * (prev.x + values_[i * 3 + 2].x) +
                                 3.0f * temp_t1 * temp_t * temp_t * (next.x + values_[i * 3 + 3].x) +
                                 temp_t * temp_t * temp_t * next.x;
          if (const float test_x = (global_x - prev.x) / (next.x - prev.x); test_x > real_x) {
            upper = temp_t;
            temp_t = (temp_t + lower) / 2.0f;
          } else {
            lower = temp_t;
            temp_t = (temp_t + upper) / 2.0f;
          }
        }
        const float temp_t1 = 1.0f - temp_t;
        return temp_t1 * temp_t1 * temp_t1 * prev.y +
               3.0f * temp_t1 * temp_t1 * temp_t * (prev.y + values_[i * 3 + 2].y) +
               3.0f * temp_t1 * temp_t * temp_t * (next.y + values_[i * 3 + 3].y) + temp_t * temp_t * temp_t * next.y;
      }
    }
    return values_[values_.size() - 2].y;
  }
  for (int i = 0; i < values_.size() - 1; i++) {
    auto& prev = values_[i];
    if (auto& next = values_[i + 1]; x >= prev.x && x < next.x) {
      return prev.y + (next.y - prev.y) * (x - prev.x) / (next.x - prev.x);
    }
  }
  return values_[values_.size() - 1].y;
}

Curve2D::Curve2D(const glm::vec2& min, const glm::vec2& max, bool tangent) {
  tangent_ = tangent;
  min_ = min;
  max_ = max;
  Clear();
}
Curve2D::Curve2D(float start, float end, const glm::vec2& min, const glm::vec2& max, bool tangent) {
  min_ = min;
  max_ = max;
  start = glm::clamp(start, min_.y, max_.y);
  end = glm::clamp(end, min_.y, max_.y);
  tangent_ = tangent;
  if (!tangent_) {
    values_.clear();
    values_.emplace_back(min_.x, start);
    values_.emplace_back(max_.x, end);
  } else {
    values_.clear();
    values_.emplace_back(-(max_.y - min_.y) / 10.0f, 0.0f);
    values_.emplace_back(min_.x, start);
    values_.emplace_back((max_.y - min_.y) / 10.0f, 0.0f);

    values_.emplace_back(-(max_.y - min_.y) / 10.0f, 0.0f);
    values_.emplace_back(max_.x, end);
    values_.emplace_back((max_.y - min_.y) / 10.0f, 0.0f);
  }
}
void Curve2D::SetStart(float value) {
  if (!tangent_) {
    values_.front().y = glm::clamp(value, min_.y, max_.y);
  } else {
    values_[1].y = glm::clamp(value, min_.y, max_.y);
  }
}
void Curve2D::SetEnd(float value) {
  if (!tangent_) {
    values_.back().y = glm::clamp(value, min_.y, max_.y);
  } else {
    values_[values_.size() - 2].y = glm::clamp(value, min_.y, max_.y);
  }
}
void Curve2D::Clear() {
  if (!tangent_) {
    auto start = 0.0f;
    if (!values_.empty())
      start = values_.front().y;
    auto end = 0.0f;
    if (!values_.empty())
      end = values_.back().y;

    values_.clear();
    values_.emplace_back(min_.x, (min_.y + max_.y) / 2.0f);
    values_.emplace_back(max_.x, (min_.y + max_.y) / 2.0f);
    if (!values_.empty())
      SetStart(start);
    if (!values_.empty())
      SetEnd(end);
  } else {
    auto start = 0.0f;
    auto end = 0.0f;
    if (values_.size() >= 6)
      start = values_[1].y;
    if (values_.size() >= 6)
      end = values_[values_.size() - 2].y;

    values_.clear();
    values_.emplace_back(-(max_.y - min_.y) / 10.0f, 0.0f);
    values_.emplace_back(min_.x, (min_.y + max_.y) / 2.0f);
    values_.emplace_back((max_.y - min_.y) / 10.0f, 0.0f);

    values_.emplace_back(-(max_.y - min_.y) / 10.0f, 0.0f);
    values_.emplace_back(max_.x, (min_.y + max_.y) / 2.0f);
    values_.emplace_back((max_.y - min_.y) / 10.0f, 0.0f);

    if (values_.size() >= 6)
      SetStart(start);
    if (values_.size() >= 6)
      SetEnd(end);
  }
}

void Curve2D::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  {
    out << YAML::Key << "tangent_" << tangent_;
    out << YAML::Key << "min_" << min_;
    out << YAML::Key << "max_" << max_;

    if (!values_.empty()) {
      out << YAML::Key << "values_" << YAML::BeginSeq;
      for (auto& i : values_) {
        out << i;
      }
      out << YAML::EndSeq;
    }
  }
  out << YAML::EndMap;
}

void Curve2D::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& cd = in[name];
    if (cd["tangent_"])
      tangent_ = cd["tangent_"].as<bool>();
    else if (cd["m_tangent"])
      tangent_ = cd["m_tangent"].as<bool>();
    if (cd["min_"])
      min_ = cd["min_"].as<glm::vec2>();
    else if (cd["m_min"])
      min_ = cd["m_min"].as<glm::vec2>();
    if (cd["max_"])
      max_ = cd["max_"].as<glm::vec2>();
    else if (cd["m_max"])
      max_ = cd["m_max"].as<glm::vec2>();
    if (cd["values_"]) {
      values_.clear();
      for (const auto& i : cd["values_"]) {
        values_.push_back(i.as<glm::vec2>());
      }
    } else if (cd["m_values"]) {
      values_.clear();
      for (const auto& i : cd["m_values"]) {
        values_.push_back(i.as<glm::vec2>());
      }
    }
  }
}
