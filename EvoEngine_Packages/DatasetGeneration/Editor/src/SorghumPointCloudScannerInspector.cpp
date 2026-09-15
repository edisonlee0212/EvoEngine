#include "CpuRayTracer.hpp"
#include "DatasetGenerationInspectionAdapters.hpp"
#include "DatasetGenerationSerializationAdapters.hpp"
#include "EcoSysLabLayer.hpp"
#include "EditorFileDialogs.hpp"
#include "EditorLayer.hpp"
#include "PointCloud.hpp"
#include "Sorghum.hpp"
#include "Tinyply.hpp"
#include "TreePointCloudScanner.hpp"
using namespace digital_agriculture_package;
using namespace dataset_generation_package;
bool dataset_generation_package::InspectCaptureSettings(SorghumPointCloudGridCaptureSettings& target) {
  bool changed = false;
  if (ImGui::DragInt2("Grid size", &target.grid_size.x, 1, 0, 100))
    changed = true;
  if (ImGui::DragFloat("Grid distance", &target.grid_distance, 0.1f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Step", &target.step, 0.01f, 0.0f, 0.5f))
    changed = true;
  return changed;
}
bool dataset_generation_package::InspectCaptureSettings(SorghumGantryCaptureSettings& target) {
  bool changed = false;
  if (ImGui::DragInt2("Grid size", &target.grid_size.x, 1, 0, 100))
    changed = true;
  if (ImGui::DragFloat2("Grid distance", &target.grid_distance.x, 0.1f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Step", &target.step, 0.00001f, 0.0f, 0.5f))
    changed = true;

  return changed;
}
bool SorghumPointCloudScannerInspector::Inspect(InspectorContext& context, SorghumPointCloudScanner& scanner) {
  (void)context;
  bool changed = false;
  if (ImGui::TreeNodeEx("Grid Capture")) {
    const auto& capture_settings = grid_settings;
    InspectCaptureSettings(*capture_settings);
    EditorFileDialogs::SaveFile(
        "Capture", "Point Cloud", {".ply"},
        [&](const std::filesystem::path& path) {
          scanner.Capture(path, capture_settings);
        },
        false);
    ImGui::TreePop();
  }

  return changed;
}
