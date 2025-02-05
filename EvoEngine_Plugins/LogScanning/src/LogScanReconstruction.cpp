#include "LogScanReconstruction.hpp"

using namespace log_scanning_plugin;

bool LogScanReconstruction::ReconstructionParameter::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (ImGui::DragFloat("Outlier discard", &boundary_outlier_percentage, 0.01f, 0.0f, 0.5f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Tie width", &tie_width_inch, 0.01f, 1.f, 10.f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Tie height", &tie_height_inch, 0.01f, 1.f, 10.f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Max split depth", &max_split_depth_detection_inch, 0.01f, 0.0f, 4.f)) {
    changed = true;
  }
  return changed;
}
void LogScanReconstruction::ProfileGrid::Clear() {
  for (auto& cell : RefCells()) {
    cell.type = CellData::Type::Invalid;
  }
}

void LogScanReconstruction::ProfileGrid::AddPoint(const glm::vec2& grid_center, const ProcessedPoint& processed_point,
                                                  const ReconstructionParameter& reconstruction_parameter) {
  const float start_x = (processed_point.position.x - GetMinBound().x) / GetCellSize();
  const float start_y = (processed_point.position.y - GetMinBound().y) / GetCellSize();
  auto coordinate = glm::clamp(glm::ivec2(static_cast<int>(std::floor(start_x)), static_cast<int>(std::floor(start_y))),
                               glm::ivec2(0, 0), GetResolution());

  const float tie_width = reconstruction_parameter.tie_width_inch * 0.0254f;
  const float tie_height = reconstruction_parameter.tie_height_inch * 0.0254f;
  bool level_check = true;
  bool face_check = true;
  bool range_check = true;
  while (level_check && face_check && range_check) {
    const auto current_position = GetPosition(coordinate) - grid_center;
    auto& cell = RefCell(coordinate);
    if (cell.type == CellData::Type::Skipped)
      break;
    switch (processed_point.face) {
      case ProcessedPoint::Face::Top: {
        level_check = current_position.y > 0;
        face_check = current_position.y > current_position.x * tie_height / tie_width &&
                     current_position.y > -current_position.x * tie_height / tie_width;
        coordinate.y -= 1;
        cell.type = CellData::Type::ValidTop;
      } break;
      case ProcessedPoint::Face::Bottom: {
        level_check = current_position.y < 0;
        face_check = current_position.y < current_position.x * tie_height / tie_width &&
                     current_position.y < -current_position.x * tie_height / tie_width;
        coordinate.y += 1;
        cell.type = CellData::Type::ValidBottom;
      } break;
      case ProcessedPoint::Face::Left: {
        level_check = current_position.x < 0;
        face_check = current_position.y > current_position.x * tie_height / tie_width &&
                     current_position.y < -current_position.x * tie_height / tie_width;
        coordinate.x += 1;
        cell.type = CellData::Type::ValidLeft;
      } break;
      case ProcessedPoint::Face::Right: {
        level_check = current_position.x > 0;
        face_check = current_position.y < current_position.x * tie_height / tie_width &&
                     current_position.y > -current_position.x * tie_height / tie_width;
        coordinate.x -= 1;
        cell.type = CellData::Type::ValidRight;
      } break;
    }
    if (coordinate.y < 0 || coordinate.y > GetResolution().y || coordinate.x < 0 || coordinate.x > GetResolution().x)
      range_check = false;
  }
}
void LogScanReconstruction::ProfileGrid::SkipCenterRegion(const glm::vec2& grid_center,
                                                          const ReconstructionParameter& reconstruction_parameter) {
  const float max_split_depth = reconstruction_parameter.max_split_depth_detection_inch * 0.0254f;
  const float tie_width = reconstruction_parameter.tie_width_inch * 0.0254f;
  const float tie_height = reconstruction_parameter.tie_height_inch * 0.0254f;
  for (int index = 0; index < RefCells().size(); index++) {
    const auto current_position = GetPosition(index) - grid_center;
    if (current_position.y >= -tie_height * .5f + max_split_depth &&
        current_position.y <= tie_height * .5f - max_split_depth &&
        current_position.x <= tie_width * .5f - max_split_depth &&
        current_position.x >= -tie_width * .5f + max_split_depth) {
      RefCell(index).type = CellData::Type::Skipped;
    }
  }
}
void LogScanReconstruction::Initialize(const ReconstructionParameter& reconstruction_parameter,
                                       const LogScanProfile& target_profile) {
  Clear();
  if (target_profile.points.empty())
    return;

  constexpr struct {
    bool operator()(const glm::vec2& a, const glm::vec2& b) const {
      return a.x < b.x;
    }
  } x_compare;

  constexpr struct {
    bool operator()(const glm::vec2& a, const glm::vec2& b) const {
      return a.y < b.y;
    }
  } y_compare;

  auto x_sort = target_profile.points;
  auto y_sort = target_profile.points;
  std::sort(x_sort.begin(), x_sort.end(), x_compare);
  std::sort(y_sort.begin(), y_sort.end(), y_compare);

  const int start_index = static_cast<int>(static_cast<float>(target_profile.points.size() - 1) *
                                           reconstruction_parameter.boundary_outlier_percentage);
  const int end_index = static_cast<int>(static_cast<float>(target_profile.points.size() - 1) *
                                         (1.f - reconstruction_parameter.boundary_outlier_percentage));
  const float x_min = x_sort[start_index].x;
  const float x_max = x_sort[end_index].x;
  const float y_min = y_sort[start_index].y;
  const float y_max = y_sort[end_index].y;
  points_min = glm::vec2(x_min, y_min);
  points_max = glm::vec2(x_max, y_max);
  profile_center = (points_min + points_max) * .5f;

  const float tie_width = reconstruction_parameter.tie_width_inch * 0.0254f;
  const float tie_height = reconstruction_parameter.tie_height_inch * 0.0254f;

  processed_points.resize(target_profile.points.size());
  Jobs::RunParallelFor(processed_points.size(), [&](const auto i) {
    auto& processed_point = processed_points[i];
    processed_point.position = target_profile.points[i];
    const auto relative_position = processed_point.position - profile_center;
    if (relative_position.y > relative_position.x * tie_height / tie_width) {
      if (relative_position.y > -relative_position.x * tie_height / tie_width) {
        processed_point.face = ProcessedPoint::Face::Top;
        processed_point.color = glm::vec4(1, 0, 1, 1);
      } else {
        processed_point.face = ProcessedPoint::Face::Left;
        processed_point.color = glm::vec4(0, 0, 1, 1);
      }
    } else {
      if (relative_position.y < -relative_position.x * tie_height / tie_width) {
        processed_point.face = ProcessedPoint::Face::Bottom;
        processed_point.color = glm::vec4(0, 1, 0, 1);
      } else {
        processed_point.face = ProcessedPoint::Face::Right;
        processed_point.color = glm::vec4(1, 0, 0, 1);
      }
    }
  });

  profile_grid.Reset(0.003f, points_min - glm::vec2(0.01f), points_max + glm::vec2(0.01f));
  profile_grid.Clear();

  profile_grid.SkipCenterRegion(profile_center, reconstruction_parameter);

  for (const auto& i : processed_points) {
    profile_grid.AddPoint(profile_center, i, reconstruction_parameter);
  }
}
void LogScanReconstruction::Clear() {
  processed_points.clear();
  profile_center = {};

  points_min = glm::vec2(FLT_MAX);
  points_max = glm::vec2(FLT_MIN);
}