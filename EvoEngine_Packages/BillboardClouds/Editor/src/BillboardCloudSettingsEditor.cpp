#include "BillboardCloudSettingsEditor.hpp"
#include "EditorWidgets.hpp"
using namespace billboard_clouds_package;

bool billboard_clouds_package::InspectBillboardSettings(BillboardCloud::OriginalClusterizationSettings& settings) {
  bool changed = false;
  if (ImGui::TreeNode("Original clusterization settings")) {
    if (ImGui::DragFloat("Epsilon percentage", &settings.epsilon_percentage, 0.01f, 0.01f, 1.f))
      changed = true;
    if (ImGui::DragInt("Discretization size", &settings.discretization_size, 1, 1, 1000))
      changed = true;
    if (ImGui::DragInt("Timeout", &settings.timeout, 1, 1, 1000))
      changed = true;

    ImGui::Checkbox("Skip remaining triangles", &settings.skip_remain_triangles);
    ImGui::TreePop();
  }
  return changed;
}

bool billboard_clouds_package::InspectBillboardSettings(BillboardCloud::FoliageClusterizationSettings& settings) {
  bool changed = false;
  if (ImGui::TreeNode("Foliage clusterization settings")) {
    if (ImGui::DragFloat("Complexity", &settings.density, 0.01f, 0.0f, 0.95f))
      changed = true;
    if (ImGui::DragInt("Iteration", &settings.iteration, 1, 1, 1000))
      changed = true;
    if (ImGui::DragInt("Timeout", &settings.timeout, 1, 1, 1000))
      changed = true;
    if (ImGui::DragFloat("Density", &settings.sample_range, 0.01f, 0.1f, 2.f))
      changed = true;

    if (ImGui::Checkbox("Fill band", &settings.fill_band))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

bool billboard_clouds_package::InspectBillboardSettings(BillboardCloud::ClusterizationSettings& settings) {
  bool changed = false;

  if (ImGui::TreeNode("Clusterization settings")) {
    if (ImGui::Combo("Clusterize mode", {"FlipBook", "Original", "Foliage"}, settings.clusterize_mode)) {
      changed = true;
    }
    switch (settings.clusterize_mode) {
      case static_cast<unsigned>(BillboardCloud::ClusterizationMode::FlipBook):
        break;
      case static_cast<unsigned>(BillboardCloud::ClusterizationMode::Foliage): {
        if (InspectBillboardSettings(settings.foliage_clusterization_settings))
          changed = true;
      } break;
      case static_cast<unsigned>(BillboardCloud::ClusterizationMode::Original): {
        if (InspectBillboardSettings(settings.original_clusterization_settings))
          changed = true;
      } break;
    }
    ImGui::TreePop();
  }
  return changed;
}

bool billboard_clouds_package::InspectBillboardSettings(BillboardCloud::ProjectSettings& settings) {
  bool changed = false;
  if (ImGui::TreeNode("Project settings")) {
    ImGui::TreePop();
  }
  return changed;
}

bool billboard_clouds_package::InspectBillboardSettings(BillboardCloud::JoinSettings& settings) {
  bool changed = false;
  if (ImGui::TreeNode("Join settings")) {
    ImGui::TreePop();
  }
  return changed;
}

bool billboard_clouds_package::InspectBillboardSettings(BillboardCloud::RasterizeSettings& settings) {
  bool changed = false;
  if (ImGui::TreeNode("Rasterize settings")) {
    if (ImGui::Checkbox("(Debug) Opaque", &settings.debug_opaque))
      changed = true;
    if (ImGui::Checkbox("Transfer albedo texture", &settings.transfer_albedo_map))
      changed = true;
    if (ImGui::Checkbox("Transfer normal texture", &settings.transfer_normal_map))
      changed = true;
    if (ImGui::Checkbox("Transfer roughness texture", &settings.transfer_roughness_map))
      changed = true;
    if (ImGui::Checkbox("Transfer metallic texture", &settings.transfer_metallic_map))
      changed = true;
    if (ImGui::Checkbox("Transfer ao texture", &settings.transfer_ao_map))
      changed = true;
    if (ImGui::DragInt2("Base resolution", &settings.base_resolution.x, 1, 1, 8192))
      changed = true;
    if (ImGui::DragInt2("Output color resolution", &settings.output_albedo_resolution.x, 1, 1, 8192))
      changed = true;
    if (ImGui::DragInt2("Output material props resolution", &settings.output_material_props_resolution.x, 1, 1, 8192))
      changed = true;
    if (ImGui::DragInt("Dilate", &settings.dilate, 1, -1, 1024))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

bool billboard_clouds_package::InspectBillboardSettings(BillboardCloud::GenerateSettings& settings,
                                                        const std::string& title) {
  bool changed = false;
  if (ImGui::TreeNodeEx(title.c_str())) {
    if (InspectBillboardSettings(settings.clusterization_settings))
      changed = true;
    if (InspectBillboardSettings(settings.project_settings))
      changed = true;
    if (InspectBillboardSettings(settings.join_settings))
      changed = true;
    if (InspectBillboardSettings(settings.rasterize_settings))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}
