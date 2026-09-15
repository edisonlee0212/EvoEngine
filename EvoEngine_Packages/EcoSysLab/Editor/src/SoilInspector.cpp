#include "EcoSysLabAuthoringInspectors.hpp"
#include "EcoSysLabLayer.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorFileDialogs.hpp"
#include "EditorLayer.hpp"
#include "HeightField.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "SDKInspectionAdapters.hpp"
#include "Soil.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;
bool SoilInspector::Inspect(InspectorContext& context, Soil& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (editor_layer->DragAndDropButton<SoilDescriptor>(target.soil_descriptor_ref, "SoilDescriptor", true)) {
    target.InitializeSoilModel();
  }
  auto sd = target.soil_descriptor_ref.Get<SoilDescriptor>();
  if (sd) {
    if (ImGui::Button("Generate surface mesh")) {
      target.GenerateMesh();
    }
    // Show some general properties:

    ImGui::DragFloat("Cutout X Depth", &x_depth, 0.01f, 0.0f, 1.0f, "%.2f");
    ImGui::DragFloat("Cutout Z Depth", &z_depth, 0.01f, 0.0f, 1.0f, "%.2f");
    ImGui::DragFloat("Water factor", &water_factor, 0.0001f, 0.0f, 1.0f, "%.4f");
    ImGui::DragFloat("Nutrient factor", &nutrient_factor, 0.0001f, 0.0f, 1.0f, "%.4f");
    ImGui::Checkbox("Ground surface", &ground_surface);
    if (ImGui::Button("Generate Cutout")) {
      auto scene = ApplicationContext::Get().GetActiveScene();
      auto owner = target.GetOwner();
      for (const auto& child : scene->GetChildren(owner)) {
        if (scene->GetEntityName(child) == "CutOut") {
          scene->DeleteEntity(child);
          break;
        }
      }

      const auto cut_out_entity =
          target.GenerateCutOut(x_depth, z_depth, water_factor, nutrient_factor, ground_surface);

      scene->SetParent(cut_out_entity, owner);
    }
    if (ImGui::Button("Generate Cube")) {
      auto scene = ApplicationContext::Get().GetActiveScene();
      auto owner = target.GetOwner();
      for (const auto& child : scene->GetChildren(owner)) {
        if (scene->GetEntityName(child) == "Cube") {
          scene->DeleteEntity(child);
          break;
        }
      }

      const auto cut_out_entity = target.GenerateFullBox(water_factor, nutrient_factor, ground_surface);

      scene->SetParent(cut_out_entity, owner);
    }

    if (ImGui::Button("Temporal Progression")) {
      target.temporal_progression_progress_ = 0;
      target.temporal_progression_ = true;
    }

    // auto soilDescriptor = soil_descriptor_ref.Get<SoilDescriptor>();
    // if (!soil_model.initialized_) soil_model.Initialize(soilDescriptor->soil_parameters);
    assert(target.soil_model.m_initialized);
    if (ImGui::Button("Initialize")) {
      target.InitializeSoilModel();
    }
    if (ImGui::Button("Reset")) {
      target.soil_model.Reset();
    }

    if (ImGui::Button("Split root test")) {
      target.SplitRootTestSetup();
    }

    editor_layer->DragAndDropButton<Texture2D>(soil_albedo_texture, "Albedo", true);
    editor_layer->DragAndDropButton<Texture2D>(soil_normal_texture, "Normal", true);
    editor_layer->DragAndDropButton<Texture2D>(soil_roughness_texture, "Roughness", true);
    editor_layer->DragAndDropButton<Texture2D>(soil_height_texture, "Height", true);
    editor_layer->DragAndDropButton<Texture2D>(soil_metallic_texture, "Metallic", true);
    if (ImGui::Button("Nutrient Transport: Sand")) {
      auto albedo = soil_albedo_texture.Get<Texture2D>();
      auto normal = soil_normal_texture.Get<Texture2D>();
      auto roughness = soil_roughness_texture.Get<Texture2D>();
      auto height = soil_height_texture.Get<Texture2D>();
      auto metallic = soil_metallic_texture.Get<Texture2D>();
      const std::shared_ptr<SoilMaterialTexture> soil_material_texture = std::make_shared<SoilMaterialTexture>();
      {
        if (albedo) {
          albedo->GetRgbaChannelData(soil_material_texture->m_color_map, sd->texture_resolution.x,
                                     sd->texture_resolution.y);
        } else {
          soil_material_texture->m_color_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
          std::fill(soil_material_texture->m_color_map.begin(), soil_material_texture->m_color_map.end(), glm::vec4(1));
        }
        if (height) {
          height->GetRedChannelData(soil_material_texture->m_height_map, sd->texture_resolution.x,
                                    sd->texture_resolution.y);
        } else {
          soil_material_texture->m_height_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
          std::fill(soil_material_texture->m_height_map.begin(), soil_material_texture->m_height_map.end(), 1.0f);
        }
        if (metallic) {
          metallic->GetRedChannelData(soil_material_texture->m_metallic_map, sd->texture_resolution.x,
                                      sd->texture_resolution.y);
        } else {
          soil_material_texture->m_metallic_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
          std::fill(soil_material_texture->m_metallic_map.begin(), soil_material_texture->m_metallic_map.end(), 0.2f);
        }
        if (roughness) {
          roughness->GetRedChannelData(soil_material_texture->m_roughness_map, sd->texture_resolution.x,
                                       sd->texture_resolution.y);
        } else {
          soil_material_texture->m_roughness_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
          std::fill(soil_material_texture->m_roughness_map.begin(), soil_material_texture->m_roughness_map.end(), 0.8f);
        }
        if (normal) {
          normal->GetRgbChannelData(soil_material_texture->m_normal_map, sd->texture_resolution.x,
                                    sd->texture_resolution.y);
        } else {
          soil_material_texture->m_normal_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
          std::fill(soil_material_texture->m_normal_map.begin(), soil_material_texture->m_normal_map.end(),
                    glm::vec3(0, 0, 1));
        }
      }
      target.soil_model.Test_NutrientTransport_Sand(soil_material_texture);
    }
    if (ImGui::Button("Nutrient Transport: Loam")) {
      const auto albedo = soil_albedo_texture.Get<Texture2D>();
      const auto normal = soil_normal_texture.Get<Texture2D>();
      const auto roughness = soil_roughness_texture.Get<Texture2D>();
      const auto height = soil_height_texture.Get<Texture2D>();
      const auto metallic = soil_metallic_texture.Get<Texture2D>();
      const std::shared_ptr<SoilMaterialTexture> soil_material_texture = std::make_shared<SoilMaterialTexture>();
      {
        if (albedo) {
          albedo->GetRgbaChannelData(soil_material_texture->m_color_map, sd->texture_resolution.x,
                                     sd->texture_resolution.y);
        } else {
          soil_material_texture->m_color_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
          std::fill(soil_material_texture->m_color_map.begin(), soil_material_texture->m_color_map.end(), glm::vec4(1));
        }
        if (height) {
          height->GetRedChannelData(soil_material_texture->m_height_map, sd->texture_resolution.x,
                                    sd->texture_resolution.y);
        } else {
          soil_material_texture->m_height_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
          std::fill(soil_material_texture->m_height_map.begin(), soil_material_texture->m_height_map.end(), 1.0f);
        }
        if (metallic) {
          metallic->GetRedChannelData(soil_material_texture->m_metallic_map, sd->texture_resolution.x,
                                      sd->texture_resolution.y);
        } else {
          soil_material_texture->m_metallic_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
          std::fill(soil_material_texture->m_metallic_map.begin(), soil_material_texture->m_metallic_map.end(), 0.2f);
        }
        if (roughness) {
          roughness->GetRedChannelData(soil_material_texture->m_roughness_map, sd->texture_resolution.x,
                                       sd->texture_resolution.y);
        } else {
          soil_material_texture->m_roughness_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
          std::fill(soil_material_texture->m_roughness_map.begin(), soil_material_texture->m_roughness_map.end(), 0.8f);
        }
        if (normal) {
          normal->GetRgbChannelData(soil_material_texture->m_normal_map, sd->texture_resolution.x,
                                    sd->texture_resolution.y);
        } else {
          soil_material_texture->m_normal_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
          std::fill(soil_material_texture->m_normal_map.begin(), soil_material_texture->m_normal_map.end(),
                    glm::vec3(0, 0, 1));
        }
      }
      target.soil_model.Test_NutrientTransport_Loam(soil_material_texture);
    }
    if (ImGui::Button("Nutrient Transport: Silt")) {
      auto albedo = soil_albedo_texture.Get<Texture2D>();
      auto normal = soil_normal_texture.Get<Texture2D>();
      auto roughness = soil_roughness_texture.Get<Texture2D>();
      auto height = soil_height_texture.Get<Texture2D>();
      auto metallic = soil_metallic_texture.Get<Texture2D>();
      std::shared_ptr<SoilMaterialTexture> soil_material_texture = std::make_shared<SoilMaterialTexture>();
      {
        if (albedo) {
          albedo->GetRgbaChannelData(soil_material_texture->m_color_map, sd->texture_resolution.x,
                                     sd->texture_resolution.y);
        } else {
          soil_material_texture->m_color_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
          std::fill(soil_material_texture->m_color_map.begin(), soil_material_texture->m_color_map.end(), glm::vec4(1));
        }
        if (height) {
          height->GetRedChannelData(soil_material_texture->m_height_map, sd->texture_resolution.x,
                                    sd->texture_resolution.y);
        } else {
          soil_material_texture->m_height_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
          std::fill(soil_material_texture->m_height_map.begin(), soil_material_texture->m_height_map.end(), 1.0f);
        }
        if (metallic) {
          metallic->GetRedChannelData(soil_material_texture->m_metallic_map, sd->texture_resolution.x,
                                      sd->texture_resolution.y);
        } else {
          soil_material_texture->m_metallic_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
          std::fill(soil_material_texture->m_metallic_map.begin(), soil_material_texture->m_metallic_map.end(), 0.2f);
        }
        if (roughness) {
          roughness->GetRedChannelData(soil_material_texture->m_roughness_map, sd->texture_resolution.x,
                                       sd->texture_resolution.y);
        } else {
          soil_material_texture->m_roughness_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
          std::fill(soil_material_texture->m_roughness_map.begin(), soil_material_texture->m_roughness_map.end(), 0.8f);
        }
        if (normal) {
          normal->GetRgbChannelData(soil_material_texture->m_normal_map, sd->texture_resolution.x,
                                    sd->texture_resolution.y);
        } else {
          soil_material_texture->m_normal_map.resize(sd->texture_resolution.x * sd->texture_resolution.y);
          std::fill(soil_material_texture->m_normal_map.begin(), soil_material_texture->m_normal_map.end(),
                    glm::vec3(0, 0, 1));
        }
      }
      target.soil_model.Test_NutrientTransport_Silt(soil_material_texture);
    }
    ImGui::InputFloat("Diffusion Force", &target.soil_model.m_diffusionForce);
    ImGui::InputFloat3("Gravity Force", &target.soil_model.m_gravityForce.x);

    ImGui::Checkbox("Auto step", &target.auto_step_);
    if (ImGui::Button("Step") || target.auto_step_) {
      if (target.irrigation_)
        target.soil_model.Irrigation();
      target.soil_model.Step();
    }
    ImGui::SliderFloat("Irrigation amount", &target.soil_model.m_irrigationAmount, 0.01, 100, "%.2f",
                       ImGuiSliderFlags_Logarithmic);
    ImGui::Checkbox("apply Irrigation", &target.irrigation_);

    ImGui::InputFloat3("Source position", (float*)&target.source_position_);
    ImGui::SliderFloat("Source amount", &target.source_amount_, 1, 10000, "%.4f", ImGuiSliderFlags_Logarithmic);
    ImGui::InputFloat("Source width", &target.source_width_, 0.1, 100, "%.4f", ImGuiSliderFlags_Logarithmic);
    if (ImGui::Button("Apply Source")) {
      target.soil_model.ChangeWater(target.source_position_, target.source_amount_, target.source_width_);
    }
  }
  return changed;
}
