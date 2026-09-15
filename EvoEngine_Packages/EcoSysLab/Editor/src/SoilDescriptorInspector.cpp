#include "EcoSysLabLayer.hpp"
#include "EcoSysLabObjectInspectors.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorLayer.hpp"
#include "HeightField.hpp"
#include "Material.hpp"
#include "SDKInspectionAdapters.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;

bool DrawSoilParametersGui(SoilParameters& soil_parameters) {
  bool changed = false;
  if (ImGui::TreeNodeEx("Soil Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::InputInt3("VoxelGrid Resolution", (int*)&soil_parameters.m_voxelResolution)) {
      changed = true;
    }
    if (ImGui::DragFloat("Delta X", &soil_parameters.m_deltaX, 0.01f, 0.01f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Delta time", &soil_parameters.m_deltaTime, 0.01f, 0.0f, 10.0f)) {
      changed = true;
    }
    if (ImGui::InputFloat3("Bounding Box Min", (float*)&soil_parameters.m_boundingBoxMin)) {
      changed = true;
    }
    // TODO: boundaries
    if (ImGui::DragFloat("Diffusion Force", &soil_parameters.m_diffusionForce, 0.01f, 0.0f, 999.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat3("Gravity Force", &soil_parameters.m_gravityForce.x, 0.01f, 0.0f, 999.0f)) {
      changed = true;
    }
    ImGui::TreePop();
  }
  return changed;
}

void SetSoilPhysicalMaterial(procedural_noise::ProceduralNoise3D& c, procedural_noise::ProceduralNoise3D& p,
                             float sand_ratio, float silt_ratio, float clay_ratio, float compactness) {
  assert(compactness <= 1.0f && compactness >= 0.0f);

  const float weight = sand_ratio + silt_ratio + clay_ratio;
  sand_ratio = sand_ratio * compactness / weight;
  silt_ratio = silt_ratio * compactness / weight;
  clay_ratio = clay_ratio * compactness / weight;
  const float air_ratio = 1.f - compactness;

  static glm::vec2 sand_physical_coefficients = glm::vec2(0.9f, 15.0f);
  static glm::vec2 silt_physical_coefficients = glm::vec2(1.9f, 1.5f);
  static glm::vec2 clay_physical_coefficients = glm::vec2(2.1f, 0.05f);
  static glm::vec2 air_physical_coefficients = glm::vec2(5.0f, 30.0f);
  /*
  c.noise_descriptors.resize(1);
  p.noise_descriptors.resize(1);
  c.noise_descriptors[0].type = 0;
  c.noise_descriptors[1].type = 0;
  c.noise_descriptors[0].offset = sand_ratio * sand_physical_coefficients.x + silt_ratio * silt_physical_coefficients.x
  + clay_ratio * clay_physical_coefficients.x + air_ratio * air_physical_coefficients.x; p.noise_descriptors[0].offset =
  sand_ratio * sand_physical_coefficients.y + silt_ratio * silt_physical_coefficients.y + clay_ratio *
  clay_physical_coefficients.y + air_ratio * air_physical_coefficients.y;*/
}

bool SoilLayerDescriptorInspector::Inspect(InspectorContext& context, SoilLayerDescriptor& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (ImGui::TreeNodeEx("Generate from preset soil ratio")) {
    ImGui::SliderFloat("Sand ratio", &sand_ratio, 0.0f, 1.0f);
    ImGui::SliderFloat("Silt ratio", &silt_ratio, 0.0f, 1.0f);
    ImGui::SliderFloat("Clay ratio", &clay_ratio, 0.0f, 1.0f);
    ImGui::SliderFloat("Compactness", &compactness, 0.0f, 1.0f);
    if (ImGui::Button("Generate soil")) {
      SetSoilPhysicalMaterial(target.capacity_graph, target.permeability_graph, sand_ratio, silt_ratio, clay_ratio,
                              compactness);
      changed = true;
    }
    if (ImGui::TreeNode("Generate from preset combination")) {
      ImGui::Combo({"Select soil combination preset"}, {"Clay", "Silty Clay", "Loam", "Sand", "Loamy Sand"},
                   soil_type_preset);
      if (ImGui::Button("Apply combination")) {
        switch (static_cast<SoilMaterialType>(soil_type_preset)) {
          case SoilMaterialType::Clay:
            sand_ratio = 0.1f;
            silt_ratio = 0.1f;
            clay_ratio = 0.8f;
            compactness = 1.f;
            break;
          case SoilMaterialType::SiltyClay:
            sand_ratio = 0.1f;
            silt_ratio = 0.4f;
            clay_ratio = 0.5f;
            compactness = 1.f;
            break;
          case SoilMaterialType::Loam:
            sand_ratio = 0.4f;
            silt_ratio = 0.4f;
            clay_ratio = 0.2f;
            compactness = 1.f;
            break;
          case SoilMaterialType::Sand:
            sand_ratio = 1.f;
            silt_ratio = 0.f;
            clay_ratio = 0.f;
            compactness = 1.f;
            break;
          case SoilMaterialType::LoamySand:
            sand_ratio = 0.8f;
            silt_ratio = 0.1f;
            clay_ratio = 0.1f;
            compactness = 1.f;
            break;
        }
      }
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }

  ImGui::Checkbox("Show capacity", &show_capacity);
  if (show_capacity) {
    changed = evo_engine::DrawProceduralNoiseGraph(target.capacity_graph, "Capacity graph", editor_layer) | changed;
  }

  ImGui::Checkbox("Show permeability", &show_permeability);
  if (show_permeability) {
    changed =
        evo_engine::DrawProceduralNoiseGraph(target.permeability_graph, "Permeability graph", editor_layer) | changed;
  }

  ImGui::Checkbox("Show density", &show_density);
  if (show_density) {
    changed = evo_engine::DrawProceduralNoiseGraph(target.density_graph, "Density graph", editor_layer) | changed;
  }

  ImGui::Checkbox("Show initial nutrients", &show_initial_nutrients);
  if (show_initial_nutrients) {
    changed =
        evo_engine::DrawProceduralNoiseGraph(target.initial_nutrients_graph, "Initial nutrients graph", editor_layer) |
        changed;
  }

  ImGui::Checkbox("Show initial water", &show_initial_water);
  if (show_initial_water) {
    changed =
        evo_engine::DrawProceduralNoiseGraph(target.initial_water_graph, "Initial water graph", editor_layer) | changed;
  }

  ImGui::Checkbox("Show thickness", &show_thickness);
  if (show_thickness) {
    changed = evo_engine::DrawProceduralNoiseGraph(target.thickness_graph, "Thickness graph", editor_layer) | changed;
  }

  if (ImGui::TreeNode("Textures")) {
    if (editor_layer->DragAndDropButton<Texture2D>(target.albedo_texture, "Albedo"))
      changed = true;
    if (editor_layer->DragAndDropButton<Texture2D>(target.roughness_texture, "Roughness"))
      changed = true;
    if (editor_layer->DragAndDropButton<Texture2D>(target.metallic_texture, "Metallic"))
      changed = true;
    if (editor_layer->DragAndDropButton<Texture2D>(target.normal_texture, "Normal"))
      changed = true;
    if (editor_layer->DragAndDropButton<Texture2D>(target.height_texture, "Height"))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

bool SoilDescriptorInspector::Inspect(InspectorContext& context, SoilDescriptor& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (editor_layer->DragAndDropButton<HeightField>(target.height_field, "Height Field", true)) {
    changed = true;
  }

  /*
  glm::ivec3 resolution = m_voxelResolution;
  if (ImGui::DragInt3("VoxelGrid Resolution", &resolution.x, 1, 1, 100))
  {
          m_voxelResolution = resolution;
          changed = true;
  }
  if (ImGui::DragFloat3("VoxelGrid Bounding box min", &m_boundingBoxMin.x, 0.01f))
  {
          changed = true;
  }
  */

  if (ImGui::Button("Instantiate")) {
    auto scene = ApplicationContext::Get().GetActiveScene();
    auto soil_entity = scene->CreateEntity(target.GetTitle());
    auto soil = scene->GetOrSetPrivateComponent<Soil>(soil_entity).lock();
    soil->soil_descriptor_ref = target.GetSelf();
    soil->InitializeSoilModel();
  }

  if (DrawSoilParametersGui(target.soil_parameters)) {
    changed = true;
  }
  if (AssetRef temp_soil_layer_descriptor_holder; editor_layer->DragAndDropButton<SoilLayerDescriptor>(
          temp_soil_layer_descriptor_holder, "Drop new SoilLayerDescriptor here...")) {
    if (auto sld = temp_soil_layer_descriptor_holder.Get<SoilLayerDescriptor>()) {
      target.soil_layer_descriptors.emplace_back(sld);
      changed = true;
    }
    temp_soil_layer_descriptor_holder.Clear();
  }
  for (int i = 0; i < target.soil_layer_descriptors.size(); i++) {
    if (auto soil_layer_descriptor = target.soil_layer_descriptors[i].Get<SoilLayerDescriptor>()) {
      if (ImGui::TreeNodeEx(("No." + std::to_string(i + 1)).c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text(("Name: " + soil_layer_descriptor->GetTitle()).c_str());

        if (ImGui::Button("Remove")) {
          target.soil_layer_descriptors.erase(target.soil_layer_descriptors.begin() + i);
          changed = true;
          ImGui::TreePop();
          continue;
        }
        if (!soil_layer_descriptor->Saved()) {
          ImGui::SameLine();
          if (ImGui::Button("Save")) {
            soil_layer_descriptor->Save();
          }
        }
        if (i < target.soil_layer_descriptors.size() - 1) {
          ImGui::SameLine();
          if (ImGui::Button("Move down")) {
            changed = true;
            const auto temp = target.soil_layer_descriptors[i];
            target.soil_layer_descriptors[i] = target.soil_layer_descriptors[i + 1];
            target.soil_layer_descriptors[i + 1] = temp;
          }
        }
        if (i > 0) {
          ImGui::SameLine();
          if (ImGui::Button("Move up")) {
            changed = true;
            const auto temp = target.soil_layer_descriptors[i - 1];
            target.soil_layer_descriptors[i - 1] = target.soil_layer_descriptors[i];
            target.soil_layer_descriptors[i] = temp;
          }
        }
        if (ImGui::TreeNode("Settings")) {
          InspectorRegistry::GetInstance().Inspect(context, *soil_layer_descriptor);
          ImGui::TreePop();
        }
        ImGui::TreePop();
      }
    } else {
      target.soil_layer_descriptors.erase(target.soil_layer_descriptors.begin() + i);
      i--;
    }
  }

  return changed;
}
