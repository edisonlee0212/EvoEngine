#pragma once
#include "CellGrid.hpp"
#include "LogScan.hpp"
using namespace evo_engine;
namespace log_scanning_plugin {
class LogScanReconstruction {
  struct CellData {
    int blocker_size = 0;
    bool occluded = false;
  };
  class ProfileGrid : public eco_sys_lab_plugin::CellGrid<CellData> {
   public:
    void Clear() override;

    void Block(const glm::vec2& start, const glm::vec2& direction);
  };

 public:
  glm::vec2 profile_center;
  struct ReconstructionParameter {};
  void Initialize(const ReconstructionParameter& reconstruction_parameter, const LogScanProfile& target_profile,
                  const JoeScanConfig& joe_scan_config);
};
}  // namespace log_scanning_plugin