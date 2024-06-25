#include "Plot2D.hpp"
#include "Serialization.hpp"
using namespace evo_engine;

// TAKEN FROM (with much cleaning + tweaking):
// https://github.com/nem0/LumixEngine/blob/39e46c18a58111cc3c8c10a4d5ebbb614f19b1b8/external/imgui/imgui_user.inl#L505-L930

bool Curve2D::OnInspect(const std::string& label, const ImVec2& editor_size, unsigned flags) {
  enum class StorageValues : ImGuiID { FromX = 100, FromY, Width, Height, IsPanning, PointStartX, PointStartY };
  int changed_idx = -1;
  bool changed = false;

  bool no_tangent = !tangent_;
  auto& values = values_;
  if (no_tangent && values.empty() || !no_tangent && values.size() < 6) {
    Clear();
  }
  if (ImGui::Button("Clear")) {
    changed = true;
    Clear();
  }
  static ImVec2 start_pan;
  ImGuiContext& g = *GImGui;
  const ImGuiStyle& style = g.Style;
  ImVec2 size = editor_size;

  size.x = size.x < 0 ? ImGui::CalcItemWidth() : size.x;
  size.y = size.y < 0 ? size.x / 2.0f : size.y;

  ImGuiWindow* parent_window = ImGui::GetCurrentWindow();
  if (ImGuiID id = parent_window->GetID(label.c_str());
      !ImGui::BeginChildFrame(id, size, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) {
    ImGui::EndChildFrame();
    return false;
  }

  int hovered_idx = -1;

  ImGuiWindow* window = ImGui::GetCurrentWindow();
  if (window->SkipItems) {
    ImGui::EndChildFrame();
    return false;
  }

  ImVec2 points_min(FLT_MAX, FLT_MAX);
  ImVec2 points_max(-FLT_MAX, -FLT_MAX);

  bool allow_remove_sides = flags & (unsigned)CurveEditorFlags::AllowRemoveSides;

  int points_count = 0;
  if (no_tangent) {
    points_count = values.size();
  } else {
    points_count = values.size() / 3;
  }
  for (int point_idx = 0; point_idx < points_count; ++point_idx) {
    ImVec2 point;
    if (no_tangent) {
      point = reinterpret_cast<ImVec2*>(values.data())[point_idx];
    } else {
      point = reinterpret_cast<ImVec2*>(values.data())[1 + point_idx * 3];
    }
    points_max = ImMax(points_max, point);
    points_min = ImMin(points_min, point);
  }
  points_max.y = ImMax(points_max.y, points_min.y + 0.0001f);

  float from_x = window->StateStorage.GetFloat(static_cast<ImGuiID>(StorageValues::FromX), min_.x);
  float from_y = window->StateStorage.GetFloat(static_cast<ImGuiID>(StorageValues::FromY), min_.y);
  float width = window->StateStorage.GetFloat(static_cast<ImGuiID>(StorageValues::Width), max_.x - min_.x);
  float height = window->StateStorage.GetFloat(static_cast<ImGuiID>(StorageValues::Height), max_.y - min_.y);
  window->StateStorage.SetFloat(static_cast<ImGuiID>(StorageValues::FromX), from_x);
  window->StateStorage.SetFloat(static_cast<ImGuiID>(StorageValues::FromY), from_y);
  window->StateStorage.SetFloat(static_cast<ImGuiID>(StorageValues::Width), width);
  window->StateStorage.SetFloat(static_cast<ImGuiID>(StorageValues::Height), height);

  const ImRect inner_bb = window->InnerRect;

  auto transform = [&](const ImVec2& pos) -> ImVec2 {
    const float x = (pos.x - from_x) / width;
    const float y = (pos.y - from_y) / height;

    return {inner_bb.Min.x * (1 - x) + inner_bb.Max.x * x, inner_bb.Min.y * y + inner_bb.Max.y * (1 - y)};
  };

  auto inv_transform = [&](const ImVec2& pos) -> ImVec2 {
    float x = (pos.x - inner_bb.Min.x) / (inner_bb.Max.x - inner_bb.Min.x);
    float y = (inner_bb.Max.y - pos.y) / (inner_bb.Max.y - inner_bb.Min.y);

    return {from_x + width * x, from_y + height * y};
  };

  if (flags & static_cast<unsigned>(CurveEditorFlags::ShowGrid)) {
    int exp;
    frexp(width / 5, &exp);
    auto step_x = static_cast<float>(ldexp(1.0, exp));
    int cell_cols = static_cast<int>(width / step_x);

    float x = step_x * int(from_x / step_x);
    for (int i = -1; i < cell_cols + 2; ++i) {
      ImVec2 a = transform({x + i * step_x, from_y});
      ImVec2 b = transform({x + i * step_x, from_y + height});
      window->DrawList->AddLine(a, b, 0x55000000);
      char buf[64];
      if (exp > 0) {
        ImFormatString(buf, sizeof(buf), " %d", int(x + i * step_x));
      } else {
        ImFormatString(buf, sizeof(buf), " %f", x + i * step_x);
      }
      window->DrawList->AddText(b, 0x55000000, buf);
    }

    frexp(height / 5, &exp);
    auto step_y = static_cast<float>(ldexp(1.0, exp));
    int cell_rows = static_cast<int>(height / step_y);

    float y = step_y * static_cast<int>(from_y / step_y);
    for (int i = -1; i < cell_rows + 2; ++i) {
      ImVec2 a = transform({from_x, y + i * step_y});
      ImVec2 b = transform({from_x + width, y + i * step_y});
      window->DrawList->AddLine(a, b, 0x55000000);
      char buf[64];
      if (exp > 0) {
        ImFormatString(buf, sizeof(buf), " %d", static_cast<int>(y + i * step_y));
      } else {
        ImFormatString(buf, sizeof(buf), " %f", y + i * step_y);
      }
      window->DrawList->AddText(a, 0x55000000, buf);
    }
  }

  if (ImGui::GetIO().MouseWheel != 0 && ImGui::IsItemHovered()) {
    float scale = powf(2, ImGui::GetIO().MouseWheel);
    width *= scale;
    height *= scale;
    window->StateStorage.SetFloat(static_cast<ImGuiID>(StorageValues::Width), width);
    window->StateStorage.SetFloat(static_cast<ImGuiID>(StorageValues::Height), height);
  }
  if (ImGui::IsMouseReleased(2)) {
    window->StateStorage.SetBool(static_cast<ImGuiID>(StorageValues::IsPanning), false);
  }
  if (window->StateStorage.GetBool(static_cast<ImGuiID>(StorageValues::IsPanning), false)) {
    ImVec2 drag_offset = ImGui::GetMouseDragDelta(2);
    from_x = start_pan.x;
    from_y = start_pan.y;
    from_x -= drag_offset.x * width / (inner_bb.Max.x - inner_bb.Min.x);
    from_y += drag_offset.y * height / (inner_bb.Max.y - inner_bb.Min.y);
    window->StateStorage.SetFloat(static_cast<ImGuiID>(StorageValues::FromX), from_x);
    window->StateStorage.SetFloat(static_cast<ImGuiID>(StorageValues::FromY), from_y);
  } else if (ImGui::IsMouseDragging(2) && ImGui::IsItemHovered()) {
    window->StateStorage.SetBool(static_cast<ImGuiID>(StorageValues::IsPanning), true);
    start_pan.x = from_x;
    start_pan.y = from_y;
  }

  for (int point_idx = points_count - 2; point_idx >= 0; --point_idx) {
    ImVec2* points;
    if (no_tangent) {
      points = reinterpret_cast<ImVec2*>(values.data()) + point_idx;
    } else {
      points = reinterpret_cast<ImVec2*>(values.data()) + 1 + point_idx * 3;
    }

    ImVec2 p_prev = points[0];
    ImVec2 tangent_last;
    ImVec2 tangent;
    ImVec2 p;
    if (no_tangent) {
      p = points[1];
    } else {
      tangent_last = points[1];
      tangent = points[2];
      p = points[3];
    }
    int* selected_point = 0;
    auto handle_point = [&](ImVec2& p, int idx) -> bool {
      float screen_size = size.x / 100.0f;

      const ImVec2 cursor_pos = ImGui::GetCursorScreenPos();
      ImVec2 pos = transform(p);

      ImGui::SetCursorScreenPos(pos - ImVec2(screen_size, screen_size));
      ImGui::PushID(idx);
      ImGui::InvisibleButton("", ImVec2(screen_size * 2, screen_size * 2));

      const bool is_selected = selected_point && *selected_point == point_idx + idx;
      const float thickness = is_selected ? 2.0f : 1.0f;
      ImU32 col = ImGui::IsItemActive() || ImGui::IsItemHovered() ? ImGui::GetColorU32(ImGuiCol_PlotLinesHovered)
                                                                  : ImGui::GetColorU32(ImGuiCol_PlotLines);

      window->DrawList->AddLine(pos + ImVec2(-screen_size, 0), pos + ImVec2(0, screen_size), col, thickness);
      window->DrawList->AddLine(pos + ImVec2(screen_size, 0), pos + ImVec2(0, screen_size), col, thickness);
      window->DrawList->AddLine(pos + ImVec2(screen_size, 0), pos + ImVec2(0, -screen_size), col, thickness);
      window->DrawList->AddLine(pos + ImVec2(-screen_size, 0), pos + ImVec2(0, -screen_size), col, thickness);

      if (ImGui::IsItemHovered())
        hovered_idx = point_idx + idx;

      if (ImGui::IsItemActive() && ImGui::IsMouseClicked(0)) {
        if (selected_point)
          *selected_point = point_idx + idx;
        window->StateStorage.SetFloat((ImGuiID)StorageValues::PointStartX, pos.x);
        window->StateStorage.SetFloat((ImGuiID)StorageValues::PointStartY, pos.y);
      }

      if (ImGui::IsItemHovered() || ImGui::IsItemActive() && ImGui::IsMouseDragging(0)) {
        char tmp[64];
        ImFormatString(tmp, sizeof(tmp), "%0.2f, %0.2f", p.x, p.y);
        window->DrawList->AddText({pos.x, pos.y - ImGui::GetTextLineHeight()}, 0xff000000, tmp);
      }
      bool value_changed = false;
      if (ImGui::IsItemActive() && ImGui::IsMouseDragging(0)) {
        pos.x = window->StateStorage.GetFloat((ImGuiID)StorageValues::PointStartX, pos.x);
        pos.y = window->StateStorage.GetFloat((ImGuiID)StorageValues::PointStartY, pos.y);
        pos += ImGui::GetMouseDragDelta();
        ImVec2 v = inv_transform(pos);

        p = v;
        value_changed = true;
      }
      ImGui::PopID();

      ImGui::SetCursorScreenPos(cursor_pos);
      return value_changed;
    };

    auto handle_tangent = [&](ImVec2& t, const ImVec2& p, int idx) -> bool {
      const float screen_size = size.x / 100.0f;

      auto normalized = [](const ImVec2& v) -> ImVec2 {
        float len = 1.0f / sqrtf(v.x * v.x + v.y * v.y);
        return ImVec2(v.x * len, v.y * len);
      };

      ImVec2 cursor_pos = ImGui::GetCursorScreenPos();
      ImVec2 pos = transform(p);
      ImVec2 tang = pos + ImVec2(t.x, -t.y) * size.x;

      ImGui::SetCursorScreenPos(tang - ImVec2(screen_size, screen_size));
      ImGui::PushID(-idx);
      ImGui::InvisibleButton("", ImVec2(screen_size * 2, screen_size * 2));

      window->DrawList->AddLine(pos, tang, ImGui::GetColorU32(ImGuiCol_PlotLines));

      ImU32 col = ImGui::IsItemHovered() ? ImGui::GetColorU32(ImGuiCol_PlotLinesHovered)
                                         : ImGui::GetColorU32(ImGuiCol_PlotLines);

      window->DrawList->AddLine(tang + ImVec2(-screen_size, screen_size), tang + ImVec2(screen_size, screen_size), col);
      window->DrawList->AddLine(tang + ImVec2(screen_size, screen_size), tang + ImVec2(screen_size, -screen_size), col);
      window->DrawList->AddLine(tang + ImVec2(screen_size, -screen_size), tang + ImVec2(-screen_size, -screen_size),
                                col);
      window->DrawList->AddLine(tang + ImVec2(-screen_size, -screen_size), tang + ImVec2(-screen_size, screen_size),
                                col);
      bool tangent_changed = false;
      if (ImGui::IsItemActive() && ImGui::IsMouseDragging(0)) {
        tang = ImGui::GetIO().MousePos - pos;
        tang = tang / size.x;
        tang.y *= -1;
        t = tang;
        tangent_changed = true;
      }
      ImGui::PopID();

      ImGui::SetCursorScreenPos(cursor_pos);
      return tangent_changed;
    };

    ImGui::PushID(point_idx);
    if (!no_tangent) {
      window->DrawList->AddBezierCubic(transform(p_prev), transform(p_prev + tangent_last), transform(p + tangent),
                                       transform(p), ImGui::GetColorU32(ImGuiCol_PlotLines), 1.0f, 20);
      if (handle_tangent(tangent_last, p_prev, 0)) {
        auto diff = p - p_prev + tangent;
        points[1] = ImClamp(tangent_last, ImVec2(0, -1), ImVec2(diff.x, 1));
        changed_idx = point_idx;
      }
      if (handle_tangent(tangent, p, 1)) {
        auto diff = p - p_prev - tangent_last;
        points[2] = ImClamp(tangent, ImVec2(-diff.x, -1), ImVec2(0, 1));
        changed_idx = point_idx + 1;
      }
      if (point_idx < points_count - 2 && handle_point(p, 1)) {
        points[3] = ImClamp(p, ImVec2(p_prev.x + tangent_last.x - tangent.x + 0.001f, min_.y),
                            ImVec2(points[6].x + points[5].x - points[4].x - 0.001f, max_.y));
        changed_idx = point_idx + 1;
      }
    } else {
      window->DrawList->AddLine(transform(p_prev), transform(p), ImGui::GetColorU32(ImGuiCol_PlotLines), 1.0f);
      if (handle_point(p, 1)) {
        if (p.x <= p_prev.x)
          p.x = p_prev.x + 0.001f;
        if (point_idx < points_count - 2 && p.x >= points[2].x) {
          p.x = points[2].x - 0.001f;
        }
        points[1] = p;
        changed_idx = point_idx + 1;
      }
    }
    ImGui::PopID();
  }

  ImGui::SetCursorScreenPos(inner_bb.Min);

  ImGui::InvisibleButton("bg", inner_bb.Max - inner_bb.Min);
  bool allow_resize = flags & static_cast<unsigned>(CurveEditorFlags::AllowResize);
  if (ImGui::IsItemActive() && ImGui::IsMouseDoubleClicked(0) && allow_resize) {
    ImVec2 mp = ImGui::GetMousePos();
    ImVec2 new_p = inv_transform(mp);
    if (!no_tangent) {
      bool suitable = false;
      for (int i = 0; i < points_count - 1; i++) {
        auto& prev = values[i * 3 + 1];
        auto& last_t = values[i * 3 + 2];
        auto& next_t = values[i * 3 + 3];
        auto& next = values[i * 3 + 4];

        if (new_p.x - 0.001 > prev.x + last_t.x && new_p.x + 0.001 < next.x + next_t.x) {
          suitable = true;
          break;
        }
      }
      if (suitable) {
        changed = true;
        values.resize(values.size() + 3);
        values[points_count * 3 + 0] = glm::vec2(-0.1f, 0);
        values[points_count * 3 + 1] = glm::vec2(new_p.x, new_p.y);
        values[points_count * 3 + 2] = glm::vec2(0.1f, 0);
        auto compare = [](const void* a, const void* b) -> int {
          float fa = (((const ImVec2*)a) + 1)->x;
          float fb = (((const ImVec2*)b) + 1)->x;
          return fa < fb ? -1 : (fa > fb) ? 1 : 0;
        };
        qsort(values.data(), points_count + 1, sizeof(ImVec2) * 3, compare);
        for (int i = 0; i < points_count + 1; i++) {
          if (values[i * 3 + 1].x != new_p.x)
            continue;
          if (i > 0) {
            values[i * 3].x =
                glm::clamp(values[i * 3].x, (values[i * 3 - 2].x + values[i * 3 - 1].x) - values[i * 3 + 1].x, 0.0f);
          }
          if (i < points_count) {
            values[i * 3 + 2].x = glm::clamp(values[i * 3 + 2].x, 0.0f,
                                             (values[i * 3 + 4].x + values[i * 3 + 3].x) - values[i * 3 + 1].x);
          }
        }
      }
    } else {
      changed = true;
      values.resize(values.size() + 1);
      values[points_count] = glm::vec2(new_p.x, new_p.y);

      auto compare = [](const void* a, const void* b) -> int {
        const float fa = static_cast<const ImVec2*>(a)->x;
        const float fb = static_cast<const ImVec2*>(b)->x;
        return fa < fb ? -1 : (fa > fb) ? 1 : 0;
      };

      qsort(values.data(), points_count + 1, sizeof(ImVec2), compare);
    }
  }
  if (hovered_idx >= 0 && ImGui::IsMouseDoubleClicked(0) && allow_resize && points_count > 2) {
    if (allow_remove_sides || (hovered_idx > 0 && hovered_idx < points_count - 1)) {
      changed = true;
      auto* points = reinterpret_cast<ImVec2*>(values.data());
      if (!no_tangent) {
        for (int j = hovered_idx * 3; j < points_count * 3 - 3; j += 3) {
          points[j + 0] = points[j + 3];
          points[j + 1] = points[j + 4];
          points[j + 2] = points[j + 5];
        }
        values.resize(values.size() - 3);
      } else {
        for (int j = hovered_idx; j < points_count - 1; ++j) {
          points[j] = points[j + 1];
        }
        values.resize(values.size() - 1);
      }
    }
  }

  ImGui::EndChildFrame();
  if (!((unsigned)flags & (unsigned)CurveEditorFlags::DisableStartEndY)) {
    if (no_tangent) {
      if (ImGui::SliderFloat("Begin Y", &values.front().y, min_.y, max_.y)) {
        changed = true;
      }
      if (ImGui::SliderFloat("End Y", &values.back().y, min_.y, max_.y)) {
        changed = true;
      }
    } else {
      if (ImGui::SliderFloat("Begin Y", &values[1].y, min_.y, max_.y)) {
        changed = true;
      }
      if (ImGui::SliderFloat("End Y", &values[values.size() - 2].y, min_.y, max_.y)) {
        changed = true;
      }
    }
  }
  if ((unsigned)flags & (unsigned)CurveEditorFlags::ShowDebug) {
    static float test = 0.5f;
    ImGui::SliderFloat("X", &test, 0.0f, 1.0f);
    ImGui::Text("Y: %.3f", GetValue(test));
  }
  return changed_idx != -1 || changed;
}
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
float Curve2D::GetValue(float x, unsigned iteration) const {
  x = glm::clamp(x, 0.0f, 1.0f);
  if (tangent_) {
    const int point_size = values_.size() / 3;
    for (int i = 0; i < point_size - 1; i++) {
      auto& prev = values_[i * 3 + 1];
      auto& next = values_[i * 3 + 4];
      if (x == prev.x) {
        return prev.y;
      } else if (x > prev.x && x < next.x) {
        const float real_x = (x - prev.x) / (next.x - prev.x);
        float upper = 1.0f;
        float lower = 0.0f;
        float temp_t = 0.5f;
        for (unsigned iteration = 0; iteration < iteration; iteration++) {
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
  } else {
    for (int i = 0; i < values_.size() - 1; i++) {
      auto& prev = values_[i];
      auto& next = values_[i + 1];
      if (x >= prev.x && x < next.x) {
        return prev.y + (next.y - prev.y) * (x - prev.x) / (next.x - prev.x);
      }
    }
    return values_[values_.size() - 1].y;
  }
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
    values_.push_back({min_.x, start});
    values_.push_back({max_.x, end});
  } else {
    values_.clear();
    values_.push_back({-(max_.y - min_.y) / 10.0f, 0.0f});
    values_.push_back({min_.x, start});
    values_.push_back({(max_.y - min_.y) / 10.0f, 0.0f});

    values_.push_back({-(max_.y - min_.y) / 10.0f, 0.0f});
    values_.push_back({max_.x, end});
    values_.push_back({(max_.y - min_.y) / 10.0f, 0.0f});
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
    tangent_ = cd["tangent_"].as<bool>();
    min_ = cd["min_"].as<glm::vec2>();
    max_ = cd["max_"].as<glm::vec2>();
    values_.clear();
    if (cd["values_"]) {
      for (const auto& i : cd["values_"]) {
        values_.push_back(i.as<glm::vec2>());
      }
    }
  }
}
