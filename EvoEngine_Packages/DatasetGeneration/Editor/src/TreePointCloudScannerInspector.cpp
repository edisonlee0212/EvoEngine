#include "CpuRayTracer.hpp"
#include "DatasetGenerationInspectionAdapters.hpp"
#include "DatasetGenerationSerializationAdapters.hpp"
#include "EcoSysLabLayer.hpp"
#include "EditorFileDialogs.hpp"
#include "EditorLayer.hpp"
#include "PointCloud.hpp"
#include "Soil.hpp"
#include "Tinyply.hpp"
using namespace eco_sys_lab_package;
using namespace dataset_generation_package;
void dataset_generation_package::InspectCaptureSettings(TreePointCloudPointSettings& target) {
  ImGui::DragFloat("Point variance", &target.variance, 0.01f);
  ImGui::DragFloat("Point uniform random radius", &target.ball_rand_radius, 0.01f);
  ImGui::DragFloat("Bounding box offset", &target.bounding_box_limit, 0.01f);
  ImGui::Checkbox("Type Index", &target.type_index);
  ImGui::Checkbox("Instance Index", &target.instance_index);
  ImGui::Checkbox("Branch Index", &target.branch_index);
  ImGui::Checkbox("Tree Part Index", &target.tree_part_index);
  ImGui::Checkbox("Line Index", &target.line_index);
  ImGui::Checkbox("Internode Index", &target.internode_index);
}
bool dataset_generation_package::InspectCaptureSettings(TreePointCloudCircularCaptureSettings& target) {
  bool changed = false;
  if (ImGui::DragFloat("Distance to focus point", &target.distance_from_trees, 0.01f))
    changed = true;
  if (ImGui::DragFloat("Height to ground", &target.capture_height, 0.01f))
    changed = true;
  ImGui::Separator();
  ImGui::Text("Rotation:");
  if (ImGui::DragInt3("Pitch Angle Start/Step/End", &target.pitch_angle_start, 1))
    changed = true;
  if (ImGui::DragInt3("Turn Angle Start/Step/End", &target.turn_angle_start, 1))
    changed = true;
  ImGui::Separator();
  ImGui::Text("Camera Settings:");
  if (ImGui::DragFloat("FOV", &target.camera_fov))
    changed = true;
  if (ImGui::DragInt("Resolution", &target.scan_resolution))
    changed = true;
  if (ImGui::DragFloat("Max Depth", &target.max_capture_depth))
    changed = true;
  return changed;
}
bool dataset_generation_package::InspectCaptureSettings(TreePointCloudGridCaptureSettings& target) {
  bool changed = false;
  if (ImGui::DragFloat("Max size", &target.bounding_box_size, 0.1f, 0.f, 999.f))
    changed = true;
  if (ImGui::DragInt2("Grid size", &target.grid_size.x, 1, 0, 100))
    changed = true;
  if (ImGui::DragFloat2("Grid distance", &target.grid_distance.x, 0.1f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Step", &target.step, 0.01f, 0.0f, 0.5f))
    changed = true;
  if (ImGui::DragInt("Sample", &target.ground_sample_size, 1, 1, INT_MAX))
    changed = true;
  return changed;
}
bool TreePointCloudScannerInspector::Inspect(InspectorContext& context, TreePointCloudScanner& scanner) {
  (void)context;
  bool changed = false;
  const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
  if (!eco_sys_lab_layer)
    return false;
  if (ImGui::TreeNodeEx("Circular Capture")) {
    const auto& capture_settings = circular_settings;
    InspectCaptureSettings(*capture_settings);
    EditorFileDialogs::SaveFile(
        "Capture", "Point Cloud", {".ply"},
        [&](const std::filesystem::path& path) {
          scanner.Capture(eco_sys_lab_layer->mesh_generator_settings, path, capture_settings);
        },
        false);
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Grid Capture")) {
    const auto& capture_settings = grid_settings;
    InspectCaptureSettings(*capture_settings);
    EditorFileDialogs::SaveFile(
        "Capture", "Point Cloud", {".ply"},
        [&](const std::filesystem::path& path) {
          scanner.Capture(eco_sys_lab_layer->mesh_generator_settings, path, capture_settings);
        },
        false);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Point settings")) {
    InspectCaptureSettings(scanner.point_settings);
    ImGui::TreePop();
  }

  return changed;
}
