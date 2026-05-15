#pragma once
#include "CellGrid.hpp"
#include "LogScan.hpp"

namespace log_scanning_package {
using namespace evo_engine;
class LogScanReconstruction {
 public:
  glm::vec2 profile_center{};

  glm::vec2 points_min = glm::vec2(FLT_MAX);
  glm::vec2 points_max = glm::vec2(FLT_MIN);

  struct ReconstructionParameter {
    float boundary_outlier_percentage = 0.01f;
    float tie_width_inch = 9.f;
    float tie_height_inch = 7.f;
    float max_split_depth_detection_inch = 2.f;
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  struct ProcessedPoint {
    glm::vec2 position = glm::vec2(0.f);
    enum class Face { Top, Bottom, Left, Right };
    Face face = Face::Top;
    glm::vec4 color = glm::vec4(1.f);
  };

  std::vector<ProcessedPoint> processed_points{};

  struct CellData {
    enum class Type {
      Invalid,
      ValidTop,
      ValidBottom,
      ValidLeft,
      ValidRight,
      Skipped,
    } type = Type::Invalid;
  };
  class ProfileGrid : public eco_sys_lab_package::CellGrid<CellData> {
   public:
    void Clear() override;
    void AddPoint(const glm::vec2& grid_center, const ProcessedPoint& processed_point,
                  const ReconstructionParameter& reconstruction_parameter);
    void SkipCenterRegion(const glm::vec2& grid_center, const ReconstructionParameter& reconstruction_parameter);
  };
  ProfileGrid profile_grid;
  void Initialize(const ReconstructionParameter& reconstruction_parameter, const LogScanProfile& target_profile);

  void Clear();
};
}  // namespace log_scanning_package
