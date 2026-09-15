#pragma once
#include "InspectorRegistry.hpp"
#include "SorghumPointCloudScanner.hpp"
#include "TreePointCloudScanner.hpp"
namespace dataset_generation_package {
struct TreePointCloudScannerInspector {
  std::shared_ptr<TreePointCloudCircularCaptureSettings> circular_settings =
      std::make_shared<TreePointCloudCircularCaptureSettings>();
  std::shared_ptr<TreePointCloudGridCaptureSettings> grid_settings =
      std::make_shared<TreePointCloudGridCaptureSettings>();
  bool Inspect(evo_engine::InspectorContext& context, TreePointCloudScanner& scanner);
};
struct SorghumPointCloudScannerInspector {
  std::shared_ptr<TreePointCloudGridCaptureSettings> grid_settings =
      std::make_shared<TreePointCloudGridCaptureSettings>();
  bool Inspect(evo_engine::InspectorContext& context, SorghumPointCloudScanner& scanner);
};
void InspectCaptureSettings(TreePointCloudPointSettings& target);
bool InspectCaptureSettings(TreePointCloudCircularCaptureSettings& target);
bool InspectCaptureSettings(TreePointCloudGridCaptureSettings& target);
bool InspectCaptureSettings(SorghumPointCloudGridCaptureSettings& target);
bool InspectCaptureSettings(SorghumGantryCaptureSettings& target);
}  // namespace dataset_generation_package
