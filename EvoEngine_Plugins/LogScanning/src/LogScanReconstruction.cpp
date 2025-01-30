#include "LogScanReconstruction.hpp"

using namespace log_scanning_plugin;

void LogScanReconstruction::ProfileGrid::Clear() {
  for (auto& cell : RefCells()) {
    cell.occluded = false;
  }
}
void LogScanReconstruction::ProfileGrid::Block(const glm::vec2& start, const glm::vec2& direction) {
  const int step_x = direction.x > 0 ? 1 : -1;
  const int step_y = direction.y > 0 ? 1 : -1;
  const float start_x = (start.x - GetMinBound().x) / GetCellSize();
  const float start_y = (start.y - GetMinBound().y) / GetCellSize();
  int grid_x = static_cast<int>(std::floor(start_x));
  int grid_y = static_cast<int>(std::floor(start_y));
  // Calculate tMax and tDelta
  float t_max_x = direction.x != 0.f ? (step_x > 0 ? static_cast<float>(grid_x) + 1.f - start_x
                                                   : start_x - static_cast<float>(grid_x)) /
                                           std::abs(direction.x)
                                     : std::numeric_limits<float>::infinity();
  float t_max_y = direction.y != 0.f ? (step_y > 0 ? static_cast<float>(grid_y) + 1.f - start_y
                                                   : start_y - static_cast<float>(grid_y)) /
                                           std::abs(direction.y)
                                     : std::numeric_limits<float>::infinity();
  const float t_delta_x = direction.x != 0.f ? 1.0f / std::abs(direction.x) : std::numeric_limits<float>::infinity();
  const float t_delta_y = direction.y != 0.f ? 1.0f / std::abs(direction.y) : std::numeric_limits<float>::infinity();
  // Traverse the grid cells
  while (grid_x >= 0 && grid_y >= 0 && grid_x < GetResolution().x && grid_y < GetResolution().y) {
    RefCell(glm::ivec2(grid_x, grid_y)).occluded = true;
    // Move to the next cell in the x or y direction
    if (t_max_x < t_max_y) {
      t_max_x += t_delta_x;
      grid_x += step_x;
    } else {
      t_max_y += t_delta_y;
      grid_y += step_y;
    }
  }
}
void LogScanReconstruction::Initialize(const ReconstructionParameter& reconstruction_parameter,
                                       const LogScanProfile& target_profile, const JoeScanConfig& joe_scan_config) {
}