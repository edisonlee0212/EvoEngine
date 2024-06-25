#include "Soil.hpp"

#include "Material.hpp"
#include "Mesh.hpp"

#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "Graphics.hpp"
#include "HeightField.hpp"
using namespace eco_sys_lab;

bool OnInspectSoilParameters(SoilParameters& soil_parameters) {
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

void SetSoilPhysicalMaterial(Noise3D& c, Noise3D& p, float sand_ratio, float silt_ratio, float clay_ratio,
                             float compactness) {
  assert(compactness <= 1.0f && compactness >= 0.0f);

  const float weight = sand_ratio + silt_ratio + clay_ratio;
  sand_ratio = sand_ratio * compactness / weight;
  silt_ratio = silt_ratio * compactness / weight;
  clay_ratio = clay_ratio * compactness / weight;
  const float air_ratio = 1.f - compactness;

  static glm::vec2 sand_material_properties = glm::vec2(0.9f, 15.0f);
  static glm::vec2 silt_material_properties = glm::vec2(1.9f, 1.5f);
  static glm::vec2 clay_material_properties = glm::vec2(2.1f, 0.05f);
  static glm::vec2 air_material_properties = glm::vec2(5.0f, 30.0f);

  c.noise_descriptors.resize(1);
  p.noise_descriptors.resize(1);
  c.noise_descriptors[0].type = 0;
  c.noise_descriptors[1].type = 0;
  c.noise_descriptors[0].offset = sand_ratio * sand_material_properties.x + silt_ratio * silt_material_properties.x +
                                     clay_ratio * clay_material_properties.x + air_ratio * air_material_properties.x;
  p.noise_descriptors[0].offset = sand_ratio * sand_material_properties.y + silt_ratio * silt_material_properties.y +
                                     clay_ratio * clay_material_properties.y + air_ratio * air_material_properties.y;
}

bool SoilLayerDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNodeEx("Generate from preset soil ratio")) {
    static float sand_ratio = 0.1f;
    static float silt_ratio = 0.1f;
    static float clay_ratio = 0.8f;
    static float compactness = 1.0f;
    ImGui::SliderFloat("Sand ratio", &sand_ratio, 0.0f, 1.0f);
    ImGui::SliderFloat("Silt ratio", &silt_ratio, 0.0f, 1.0f);
    ImGui::SliderFloat("Clay ratio", &clay_ratio, 0.0f, 1.0f);
    ImGui::SliderFloat("Compactness", &compactness, 0.0f, 1.0f);
    if (ImGui::Button("Generate soil")) {
      SetSoilPhysicalMaterial(capacity, permeability, sand_ratio, silt_ratio, clay_ratio, compactness);
      changed = true;
    }
    if (ImGui::TreeNode("Generate from preset combination")) {
      static unsigned soil_type_preset = 0;
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

  if (ImGui::TreeNode("Capacity")) {
    changed = capacity.OnInspect() || changed;
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Permeability")) {
    changed = permeability.OnInspect() || changed;
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Density")) {
    changed = density.OnInspect() || changed;
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Initial nutrients")) {
    changed = initial_nutrients.OnInspect() || changed;
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Initial water")) {
    changed = initial_water.OnInspect() || changed;
    ImGui::TreePop();
  }

  if (ImGui::TreeNode("Thickness")) {
    changed = thickness.OnInspect() || changed;
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Textures")) {
    if (editor_layer->DragAndDropButton<Texture2D>(albedo_texture, "Albedo"))
      changed = true;
    if (editor_layer->DragAndDropButton<Texture2D>(roughness_texture, "Roughness"))
      changed = true;
    if (editor_layer->DragAndDropButton<Texture2D>(metallic_texture, "Metallic"))
      changed = true;
    if (editor_layer->DragAndDropButton<Texture2D>(normal_texture, "Normal"))
      changed = true;
    if (editor_layer->DragAndDropButton<Texture2D>(height_texture, "Height"))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

void SoilLayerDescriptor::Serialize(YAML::Emitter& out) const {
  capacity.Save("capacity", out);
  permeability.Save("permeability", out);
  density.Save("density", out);
  initial_nutrients.Save("initial_nutrients", out);
  initial_water.Save("initial_water", out);

  thickness.Save("thickness", out);

  albedo_texture.Save("albedo_texture", out);
  roughness_texture.Save("roughness_texture", out);
  metallic_texture.Save("metallic_texture", out);
  normal_texture.Save("normal_texture", out);
  height_texture.Save("height_texture", out);
}

void SoilLayerDescriptor::Deserialize(const YAML::Node& in) {
  capacity.Load("capacity", in);
  permeability.Load("permeability", in);
  density.Load("density", in);
  initial_nutrients.Load("initial_nutrients", in);
  initial_water.Load("initial_water", in);
  thickness.Load("thickness", in);

  albedo_texture.Load("albedo_texture", in);
  roughness_texture.Load("roughness_texture", in);
  metallic_texture.Load("metallic_texture", in);
  normal_texture.Load("normal_texture", in);
  height_texture.Load("height_texture", in);
}

void SoilLayerDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(albedo_texture);
  list.push_back(roughness_texture);
  list.push_back(metallic_texture);
  list.push_back(normal_texture);
  list.push_back(height_texture);
}

bool SoilDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (editor_layer->DragAndDropButton<HeightField>(height_field, "Height Field", true)) {
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
    auto scene = Application::GetActiveScene();
    auto soil_entity = scene->CreateEntity(GetTitle());
    auto soil = scene->GetOrSetPrivateComponent<Soil>(soil_entity).lock();
    soil->soil_descriptor = ProjectManager::GetAsset(GetHandle());
    soil->InitializeSoilModel();
  }

  if (OnInspectSoilParameters(soil_parameters)) {
    changed = true;
  }
  if (AssetRef temp_soil_layer_descriptor_holder; editor_layer->DragAndDropButton<SoilLayerDescriptor>(temp_soil_layer_descriptor_holder,
                                                                                                   "Drop new SoilLayerDescriptor here...")) {
    if (auto sld = temp_soil_layer_descriptor_holder.Get<SoilLayerDescriptor>()) {
      soil_layer_descriptors.emplace_back(sld);
      changed = true;
    }
    temp_soil_layer_descriptor_holder.Clear();
  }
  for (int i = 0; i < soil_layer_descriptors.size(); i++) {
    if (auto soil_layer_descriptor = soil_layer_descriptors[i].Get<SoilLayerDescriptor>()) {
      if (ImGui::TreeNodeEx(("No." + std::to_string(i + 1)).c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text(("Name: " + soil_layer_descriptor->GetTitle()).c_str());

        if (ImGui::Button("Remove")) {
          soil_layer_descriptors.erase(soil_layer_descriptors.begin() + i);
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
        if (i < soil_layer_descriptors.size() - 1) {
          ImGui::SameLine();
          if (ImGui::Button("Move down")) {
            changed = true;
            const auto temp = soil_layer_descriptors[i];
            soil_layer_descriptors[i] = soil_layer_descriptors[i + 1];
            soil_layer_descriptors[i + 1] = temp;
          }
        }
        if (i > 0) {
          ImGui::SameLine();
          if (ImGui::Button("Move up")) {
            changed = true;
            const auto temp = soil_layer_descriptors[i - 1];
            soil_layer_descriptors[i - 1] = soil_layer_descriptors[i];
            soil_layer_descriptors[i] = temp;
          }
        }
        if (ImGui::TreeNode("Settings")) {
          soil_layer_descriptor->OnInspect(editor_layer);
          ImGui::TreePop();
        }
        ImGui::TreePop();
      }
    } else {
      soil_layer_descriptors.erase(soil_layer_descriptors.begin() + i);
      i--;
    }
  }

  return changed;
}

void SoilDescriptor::RandomOffset(const float min, const float max) {
  if (const auto hf = height_field.Get<HeightField>()) {
    hf->RandomOffset(min, max);
  }
}

bool Soil::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (editor_layer->DragAndDropButton<SoilDescriptor>(soil_descriptor, "SoilDescriptor", true)) {
    InitializeSoilModel();
  }
  auto sd = soil_descriptor.Get<SoilDescriptor>();
  if (sd) {
    if (ImGui::Button("Generate surface mesh")) {
      GenerateMesh();
    }
    // Show some general properties:

    static float x_depth = 1;
    static float z_depth = 1;
    static float water_factor = 20.f;
    static float nutrient_factor = 1.f;
    static bool ground_surface = false;
    ImGui::DragFloat("Cutout X Depth", &x_depth, 0.01f, 0.0f, 1.0f, "%.2f");
    ImGui::DragFloat("Cutout Z Depth", &z_depth, 0.01f, 0.0f, 1.0f, "%.2f");
    ImGui::DragFloat("Water factor", &water_factor, 0.0001f, 0.0f, 1.0f, "%.4f");
    ImGui::DragFloat("Nutrient factor", &nutrient_factor, 0.0001f, 0.0f, 1.0f, "%.4f");
    ImGui::Checkbox("Ground surface", &ground_surface);
    if (ImGui::Button("Generate Cutout")) {
      auto scene = Application::GetActiveScene();
      auto owner = GetOwner();
      for (const auto& child : scene->GetChildren(owner)) {
        if (scene->GetEntityName(child) == "CutOut") {
          scene->DeleteEntity(child);
          break;
        }
      }

      const auto cut_out_entity = GenerateCutOut(x_depth, z_depth, water_factor, nutrient_factor, ground_surface);

      scene->SetParent(cut_out_entity, owner);
    }
    if (ImGui::Button("Generate Cube")) {
      auto scene = Application::GetActiveScene();
      auto owner = GetOwner();
      for (const auto& child : scene->GetChildren(owner)) {
        if (scene->GetEntityName(child) == "Cube") {
          scene->DeleteEntity(child);
          break;
        }
      }

      const auto cut_out_entity = GenerateFullBox(water_factor, nutrient_factor, ground_surface);

      scene->SetParent(cut_out_entity, owner);
    }

    if (ImGui::Button("Temporal Progression")) {
      temporal_progression_progress_ = 0;
      temporal_progression_ = true;
    }

    // auto soilDescriptor = soil_descriptor.Get<SoilDescriptor>();
    // if (!soil_model.m_initialized) soil_model.Initialize(soilDescriptor->soil_parameters);
    assert(soil_model.m_initialized);
    if (ImGui::Button("Initialize")) {
      InitializeSoilModel();
    }
    if (ImGui::Button("Reset")) {
      soil_model.Reset();
    }

    if (ImGui::Button("Split root test")) {
      SplitRootTestSetup();
    }
    static AssetRef soil_albedo_texture;
    static AssetRef soil_normal_texture;
    static AssetRef soil_roughness_texture;
    static AssetRef soil_height_texture;
    static AssetRef soil_metallic_texture;
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
          soil_material_texture->m_color_map.resize(sd->texture_resolution.x *
                                                  sd->texture_resolution.y);
          std::fill(soil_material_texture->m_color_map.begin(), soil_material_texture->m_color_map.end(), glm::vec4(1));
        }
        if (height) {
          height->GetRedChannelData(soil_material_texture->m_height_map, sd->texture_resolution.x,
                                    sd->texture_resolution.y);
        } else {
          soil_material_texture->m_height_map.resize(sd->texture_resolution.x *
                                                   sd->texture_resolution.y);
          std::fill(soil_material_texture->m_height_map.begin(), soil_material_texture->m_height_map.end(), 1.0f);
        }
        if (metallic) {
          metallic->GetRedChannelData(soil_material_texture->m_metallic_map, sd->texture_resolution.x,
                                      sd->texture_resolution.y);
        } else {
          soil_material_texture->m_metallic_map.resize(sd->texture_resolution.x *
                                                     sd->texture_resolution.y);
          std::fill(soil_material_texture->m_metallic_map.begin(), soil_material_texture->m_metallic_map.end(), 0.2f);
        }
        if (roughness) {
          roughness->GetRedChannelData(soil_material_texture->m_roughness_map, sd->texture_resolution.x,
                                       sd->texture_resolution.y);
        } else {
          soil_material_texture->m_roughness_map.resize(sd->texture_resolution.x *
                                                      sd->texture_resolution.y);
          std::fill(soil_material_texture->m_roughness_map.begin(), soil_material_texture->m_roughness_map.end(), 0.8f);
        }
        if (normal) {
          normal->GetRgbChannelData(soil_material_texture->m_normal_map, sd->texture_resolution.x,
                                    sd->texture_resolution.y);
        } else {
          soil_material_texture->m_normal_map.resize(sd->texture_resolution.x *
                                                   sd->texture_resolution.y);
          std::fill(soil_material_texture->m_normal_map.begin(), soil_material_texture->m_normal_map.end(),
                    glm::vec3(0, 0, 1));
        }
      }
      soil_model.Test_NutrientTransport_Sand(soil_material_texture);
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
          soil_material_texture->m_color_map.resize(sd->texture_resolution.x *
                                                  sd->texture_resolution.y);
          std::fill(soil_material_texture->m_color_map.begin(), soil_material_texture->m_color_map.end(), glm::vec4(1));
        }
        if (height) {
          height->GetRedChannelData(soil_material_texture->m_height_map, sd->texture_resolution.x,
                                    sd->texture_resolution.y);
        } else {
          soil_material_texture->m_height_map.resize(sd->texture_resolution.x *
                                                   sd->texture_resolution.y);
          std::fill(soil_material_texture->m_height_map.begin(), soil_material_texture->m_height_map.end(), 1.0f);
        }
        if (metallic) {
          metallic->GetRedChannelData(soil_material_texture->m_metallic_map, sd->texture_resolution.x,
                                      sd->texture_resolution.y);
        } else {
          soil_material_texture->m_metallic_map.resize(sd->texture_resolution.x *
                                                     sd->texture_resolution.y);
          std::fill(soil_material_texture->m_metallic_map.begin(), soil_material_texture->m_metallic_map.end(), 0.2f);
        }
        if (roughness) {
          roughness->GetRedChannelData(soil_material_texture->m_roughness_map, sd->texture_resolution.x,
                                       sd->texture_resolution.y);
        } else {
          soil_material_texture->m_roughness_map.resize(sd->texture_resolution.x *
                                                      sd->texture_resolution.y);
          std::fill(soil_material_texture->m_roughness_map.begin(), soil_material_texture->m_roughness_map.end(), 0.8f);
        }
        if (normal) {
          normal->GetRgbChannelData(soil_material_texture->m_normal_map, sd->texture_resolution.x,
                                    sd->texture_resolution.y);
        } else {
          soil_material_texture->m_normal_map.resize(sd->texture_resolution.x *
                                                   sd->texture_resolution.y);
          std::fill(soil_material_texture->m_normal_map.begin(), soil_material_texture->m_normal_map.end(),
                    glm::vec3(0, 0, 1));
        }
      }
      soil_model.Test_NutrientTransport_Loam(soil_material_texture);
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
          soil_material_texture->m_color_map.resize(sd->texture_resolution.x *
                                                  sd->texture_resolution.y);
          std::fill(soil_material_texture->m_color_map.begin(), soil_material_texture->m_color_map.end(), glm::vec4(1));
        }
        if (height) {
          height->GetRedChannelData(soil_material_texture->m_height_map, sd->texture_resolution.x,
                                    sd->texture_resolution.y);
        } else {
          soil_material_texture->m_height_map.resize(sd->texture_resolution.x *
                                                   sd->texture_resolution.y);
          std::fill(soil_material_texture->m_height_map.begin(), soil_material_texture->m_height_map.end(), 1.0f);
        }
        if (metallic) {
          metallic->GetRedChannelData(soil_material_texture->m_metallic_map, sd->texture_resolution.x,
                                      sd->texture_resolution.y);
        } else {
          soil_material_texture->m_metallic_map.resize(sd->texture_resolution.x *
                                                     sd->texture_resolution.y);
          std::fill(soil_material_texture->m_metallic_map.begin(), soil_material_texture->m_metallic_map.end(), 0.2f);
        }
        if (roughness) {
          roughness->GetRedChannelData(soil_material_texture->m_roughness_map, sd->texture_resolution.x,
                                       sd->texture_resolution.y);
        } else {
          soil_material_texture->m_roughness_map.resize(sd->texture_resolution.x *
                                                      sd->texture_resolution.y);
          std::fill(soil_material_texture->m_roughness_map.begin(), soil_material_texture->m_roughness_map.end(), 0.8f);
        }
        if (normal) {
          normal->GetRgbChannelData(soil_material_texture->m_normal_map, sd->texture_resolution.x,
                                    sd->texture_resolution.y);
        } else {
          soil_material_texture->m_normal_map.resize(sd->texture_resolution.x *
                                                   sd->texture_resolution.y);
          std::fill(soil_material_texture->m_normal_map.begin(), soil_material_texture->m_normal_map.end(),
                    glm::vec3(0, 0, 1));
        }
      }
      soil_model.Test_NutrientTransport_Silt(soil_material_texture);
    }
    ImGui::InputFloat("Diffusion Force", &soil_model.m_diffusionForce);
    ImGui::InputFloat3("Gravity Force", &soil_model.m_gravityForce.x);

    ImGui::Checkbox("Auto step", &auto_step_);
    if (ImGui::Button("Step") || auto_step_) {
      if (irrigation_)
        soil_model.Irrigation();
      soil_model.Step();
    }
    ImGui::SliderFloat("Irrigation amount", &soil_model.m_irrigationAmount, 0.01, 100, "%.2f",
                       ImGuiSliderFlags_Logarithmic);
    ImGui::Checkbox("apply Irrigation", &irrigation_);

    ImGui::InputFloat3("Source position", (float*)&source_position_);
    ImGui::SliderFloat("Source amount", &source_amount_, 1, 10000, "%.4f", ImGuiSliderFlags_Logarithmic);
    ImGui::InputFloat("Source width", &source_width_, 0.1, 100, "%.4f", ImGuiSliderFlags_Logarithmic);
    if (ImGui::Button("Apply Source")) {
      soil_model.ChangeWater(source_position_, source_amount_, source_width_);
    }
  }
  return changed;
}

void Soil::RandomOffset(float min, float max) {
  if (const auto sd = soil_descriptor.Get<SoilDescriptor>()) {
    sd->RandomOffset(min, max);
  }
}

Entity Soil::GenerateSurfaceQuadX(bool back_facing, float depth, const glm::vec2& min_xy, const glm::vec2 max_xy,
                                  float water_factor, float nutrient_factor) {
  auto scene = Application::GetActiveScene();
  auto quad_entity = scene->CreateEntity("Slice");
  auto material = ProjectManager::CreateTemporaryAsset<Material>();
  auto albedo_tex = ProjectManager::CreateTemporaryAsset<Texture2D>();
  auto normal_tex = ProjectManager::CreateTemporaryAsset<Texture2D>();
  auto metallic_tex = ProjectManager::CreateTemporaryAsset<Texture2D>();
  auto roughness_tex = ProjectManager::CreateTemporaryAsset<Texture2D>();
  std::vector<glm::vec4> albedo_data;
  std::vector<glm::vec3> normal_data;
  std::vector<float> metallic_data;
  std::vector<float> roughness_data;
  glm::ivec2 texture_resolution;
  soil_model.GetSoilTextureSlideX(back_facing, depth, min_xy, max_xy, albedo_data, normal_data, roughness_data, metallic_data,
                                   texture_resolution, water_factor, nutrient_factor);
  albedo_tex->SetRgbaChannelData(albedo_data, texture_resolution);
  normal_tex->SetRgbChannelData(normal_data, texture_resolution);
  metallic_tex->SetRedChannelData(metallic_data, texture_resolution);
  roughness_tex->SetRedChannelData(roughness_data, texture_resolution);
  material->SetAlbedoTexture(albedo_tex);
  material->SetNormalTexture(normal_tex);
  material->SetMetallicTexture(metallic_tex);
  material->SetRoughnessTexture(roughness_tex);
  const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(quad_entity).lock();
  mesh_renderer->material = material;
  material->draw_settings.cull_mode = VK_CULL_MODE_NONE;
  mesh_renderer->mesh = Resources::GetResource<Mesh>("PRIMITIVE_QUAD");

  GlobalTransform global_transform;
  glm::vec3 scale;
  glm::vec3 position;
  glm::vec3 rotation;
  auto soil_model_size = glm::vec3(soil_model.m_resolution) * soil_model.m_dx;

  scale = glm::vec3(soil_model_size.z * (max_xy.x - min_xy.x), 1.0f, soil_model_size.y * (max_xy.y - min_xy.y));
  rotation = glm::vec3(glm::radians(90.0f), glm::radians(back_facing ? 90.0f : -90.0f), 0.0f);
  position =
      soil_model.m_boundingBoxMin + glm::vec3(soil_model_size.x * depth, soil_model_size.y * (min_xy.y + max_xy.y) * 0.5f,
                                               soil_model_size.z * (min_xy.x + max_xy.x) * 0.5f);
  global_transform.SetPosition(position);
  global_transform.SetEulerRotation(rotation);
  global_transform.SetScale(scale);
  scene->SetDataComponent(quad_entity, global_transform);
  return quad_entity;
}

Entity Soil::GenerateSurfaceQuadZ(bool back_facing, float depth, const glm::vec2& min_xy, const glm::vec2 max_xy,
                                  float water_factor, float nutrient_factor) {
  auto scene = Application::GetActiveScene();
  auto quad_entity = scene->CreateEntity("Slice");

  const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(quad_entity).lock();
  auto material = ProjectManager::CreateTemporaryAsset<Material>();
  auto albedo_tex = ProjectManager::CreateTemporaryAsset<Texture2D>();
  auto normal_tex = ProjectManager::CreateTemporaryAsset<Texture2D>();
  auto metallic_tex = ProjectManager::CreateTemporaryAsset<Texture2D>();
  auto roughness_tex = ProjectManager::CreateTemporaryAsset<Texture2D>();
  std::vector<glm::vec4> albedo_data;
  std::vector<glm::vec3> normal_data;
  std::vector<float> metallic_data;
  std::vector<float> roughness_data;
  glm::ivec2 texture_resolution;
  soil_model.GetSoilTextureSlideZ(back_facing, depth, min_xy, max_xy, albedo_data, normal_data, roughness_data, metallic_data,
                                   texture_resolution, water_factor, nutrient_factor);
  albedo_tex->SetRgbaChannelData(albedo_data, texture_resolution);
  normal_tex->SetRgbChannelData(normal_data, texture_resolution);
  metallic_tex->SetRedChannelData(metallic_data, texture_resolution);
  roughness_tex->SetRedChannelData(roughness_data, texture_resolution);
  material->SetAlbedoTexture(albedo_tex);
  material->SetNormalTexture(normal_tex);
  material->SetMetallicTexture(metallic_tex);
  material->SetRoughnessTexture(roughness_tex);

  mesh_renderer->material = material;
  material->draw_settings.cull_mode = VK_CULL_MODE_NONE;
  mesh_renderer->mesh = Resources::GetResource<Mesh>("PRIMITIVE_QUAD");

  GlobalTransform global_transform;
  glm::vec3 scale;
  glm::vec3 position;
  glm::vec3 rotation;
  auto soil_model_size = glm::vec3(soil_model.m_resolution) * soil_model.m_dx;

  scale = glm::vec3(soil_model_size.x * (max_xy.x - min_xy.x), 1.0f, soil_model_size.y * (max_xy.y - min_xy.y));
  rotation = glm::vec3(glm::radians(90.0f), glm::radians(back_facing ? 180.0f : 0.0f), 0.0f);
  position =
      soil_model.m_boundingBoxMin + glm::vec3(soil_model_size.x * (min_xy.x + max_xy.x) * 0.5f,
                                               soil_model_size.y * (min_xy.y + max_xy.y) * 0.5f, soil_model_size.z * depth);

  global_transform.SetPosition(position);
  global_transform.SetEulerRotation(rotation);
  global_transform.SetScale(scale);
  scene->SetDataComponent(quad_entity, global_transform);
  return quad_entity;
}

Entity Soil::GenerateCutOut(float x_depth, float z_depth, float water_factor, float nutrient_factor, bool enable_ground_surface) {
  auto scene = Application::GetActiveScene();
  const auto combined_entity = scene->CreateEntity("CutOut");

  if (z_depth <= 0.99f) {
    auto quad1 = GenerateSurfaceQuadX(false, 0, {0, 0}, {1.0 - z_depth, 1}, water_factor, nutrient_factor);
    scene->SetParent(quad1, combined_entity);
  }
  if (z_depth >= 0.01f && x_depth <= 0.99f) {
    auto quad2 = GenerateSurfaceQuadX(true, x_depth, {1.0 - z_depth, 0}, {1, 1}, water_factor, nutrient_factor);
    scene->SetParent(quad2, combined_entity);
  }
  if (x_depth >= 0.01f) {
    auto quad3 = GenerateSurfaceQuadZ(false, 1.0 - z_depth, {0, 0}, {x_depth, 1}, water_factor, nutrient_factor);
    scene->SetParent(quad3, combined_entity);
  }
  if (x_depth <= 0.99f) {
    auto quad4 = GenerateSurfaceQuadZ(true, 1.0, {x_depth, 0}, {1, 1}, water_factor, nutrient_factor);
    scene->SetParent(quad4, combined_entity);
  }

  if (enable_ground_surface) {
    auto ground_surface = GenerateMesh(x_depth, z_depth);
    if (const auto sd = soil_descriptor.Get<SoilDescriptor>()) {
      if (auto& soil_layer_descriptors = sd->soil_layer_descriptors; !soil_layer_descriptors.empty()) {
        if (auto first_descriptor = soil_layer_descriptors[0].Get<SoilLayerDescriptor>()) {
          auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(ground_surface).lock();
          auto mat = mmr->material.Get<Material>();
          mat->SetAlbedoTexture(first_descriptor->albedo_texture.Get<Texture2D>());
          mat->SetNormalTexture(first_descriptor->normal_texture.Get<Texture2D>());
          mat->SetRoughnessTexture(first_descriptor->roughness_texture.Get<Texture2D>());
          mat->SetMetallicTexture(first_descriptor->metallic_texture.Get<Texture2D>());
        }
      }
    }
  }
  return combined_entity;
}

Entity Soil::GenerateFullBox(float water_factor, float nutrient_factor, bool ground_surface) {
  const auto scene = Application::GetActiveScene();
  const auto combined_entity = scene->CreateEntity("Cube");

  auto quad1 = GenerateSurfaceQuadX(false, 0, {0, 0}, {1, 1}, water_factor, nutrient_factor);
  scene->SetParent(quad1, combined_entity);

  auto quad2 = GenerateSurfaceQuadX(true, 1, {0, 0}, {1, 1}, water_factor, nutrient_factor);
  scene->SetParent(quad2, combined_entity);

  auto quad3 = GenerateSurfaceQuadZ(true, 0, {0, 0}, {1, 1}, water_factor, nutrient_factor);
  scene->SetParent(quad3, combined_entity);

  auto quad4 = GenerateSurfaceQuadZ(false, 1, {0, 0}, {1, 1}, water_factor, nutrient_factor);
  scene->SetParent(quad4, combined_entity);

  if (ground_surface) {
    auto surface = GenerateMesh(0, 0);
    if (const auto sd = soil_descriptor.Get<SoilDescriptor>()) {
      if (auto& soil_layer_descriptors = sd->soil_layer_descriptors; !soil_layer_descriptors.empty()) {
        if (const auto first_descriptor = soil_layer_descriptors[0].Get<SoilLayerDescriptor>()) {
          auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(surface).lock();
          auto mat = mmr->material.Get<Material>();
          mat->SetAlbedoTexture(first_descriptor->albedo_texture.Get<Texture2D>());
          mat->SetNormalTexture(first_descriptor->normal_texture.Get<Texture2D>());
          mat->SetRoughnessTexture(first_descriptor->roughness_texture.Get<Texture2D>());
          mat->SetMetallicTexture(first_descriptor->metallic_texture.Get<Texture2D>());
        }
      }
    }
  }
  return combined_entity;
}

void Soil::Serialize(YAML::Emitter& out) const {
  soil_descriptor.Save("soil_descriptor", out);
}

void Soil::Deserialize(const YAML::Node& in) {
  soil_descriptor.Load("soil_descriptor", in);
  InitializeSoilModel();
}

void Soil::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(soil_descriptor);
}

Entity Soil::GenerateMesh(float x_depth, float z_depth) {
  const auto sd = soil_descriptor.Get<SoilDescriptor>();
  if (!sd) {
    EVOENGINE_ERROR("No soil descriptor!");
    return {};
  }
  const auto height_field = sd->height_field.Get<HeightField>();
  if (!height_field) {
    EVOENGINE_ERROR("No height field!");
    return {};
  }
  std::vector<Vertex> vertices;
  std::vector<glm::uvec3> triangles;
  height_field->GenerateMesh(glm::vec2(sd->soil_parameters.m_boundingBoxMin.x,
                                      sd->soil_parameters.m_boundingBoxMin.z),
                            glm::uvec2(sd->soil_parameters.m_voxelResolution.x,
                                       sd->soil_parameters.m_voxelResolution.z),
                            sd->soil_parameters.m_deltaX, vertices, triangles, x_depth, z_depth);

  const auto scene = Application::GetActiveScene();
  const auto self = GetOwner();
  Entity ground_surface_entity;
  const auto children = scene->GetChildren(self);

  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Ground Mesh") {
      ground_surface_entity = child;
      break;
    }
  }
  if (ground_surface_entity.GetIndex() != 0)
    scene->DeleteEntity(ground_surface_entity);
  ground_surface_entity = scene->CreateEntity("Ground Mesh");
  scene->SetParent(ground_surface_entity, self);

  const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(ground_surface_entity).lock();
  const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
  const auto material = ProjectManager::CreateTemporaryAsset<Material>();
  VertexAttributes vertex_attributes{};
  vertex_attributes.tex_coord = true;
  mesh->SetVertices(vertex_attributes, vertices, triangles);
  mesh_renderer->mesh = mesh;
  mesh_renderer->material = material;

  return ground_surface_entity;
}

void Soil::InitializeSoilModel() {
  if (const auto sd = soil_descriptor.Get<SoilDescriptor>()) {
    auto height_field = sd->height_field.Get<HeightField>();

    auto params = sd->soil_parameters;
    params.m_boundary_x = VoxelSoilModel::Boundary::wrap;
    params.m_boundary_y = VoxelSoilModel::Boundary::absorb;
    params.m_boundary_z = VoxelSoilModel::Boundary::wrap;

    SoilSurface soil_surface;
    std::vector<SoilLayer> soil_layers;

    if (height_field) {
      soil_surface.m_height = [height_field](const glm::vec2& position) {
        return height_field->GetValue(glm::vec2(position.x, position.y));
      };
    } else {
      soil_surface.m_height = [&](const glm::vec2& position) {
        return 0.0f;
      };
    }

    soil_model.m_materialTextureResolution = sd->texture_resolution;
    // Add top air layer
    int material_index = 0;

    soil_layers.emplace_back();
    auto& first_layer = soil_layers.back();
    first_layer.m_mat = SoilPhysicalMaterial({material_index,
                                             [](const glm::vec3& pos) {
                                               return 1.0f;
                                             },
                                             [](const glm::vec3& pos) {
                                               return 0.0f;
                                             },
                                             [](const glm::vec3& pos) {
                                               return 0.0f;
                                             },
                                             [](const glm::vec3& pos) {
                                               return 0.0f;
                                             },
                                             [](const glm::vec3& pos) {
                                               return 0.0f;
                                             }});
    first_layer.m_thickness = [](const glm::vec2& position) {
      return 0.f;
    };
    first_layer.m_mat.m_soilMaterialTexture = std::make_shared<SoilMaterialTexture>();
    first_layer.m_mat.m_soilMaterialTexture->m_color_map.resize(sd->texture_resolution.x *
                                                               sd->texture_resolution.y);
    std::fill(first_layer.m_mat.m_soilMaterialTexture->m_color_map.begin(),
              first_layer.m_mat.m_soilMaterialTexture->m_color_map.end(),
              glm::vec4(62.0f / 255, 49.0f / 255, 23.0f / 255, 0.0f));
    first_layer.m_mat.m_soilMaterialTexture->m_height_map.resize(sd->texture_resolution.x *
                                                                sd->texture_resolution.y);
    std::fill(first_layer.m_mat.m_soilMaterialTexture->m_height_map.begin(),
              first_layer.m_mat.m_soilMaterialTexture->m_height_map.end(), 0.1f);

    first_layer.m_mat.m_soilMaterialTexture->m_metallic_map.resize(sd->texture_resolution.x *
                                                                  sd->texture_resolution.y);
    std::fill(first_layer.m_mat.m_soilMaterialTexture->m_metallic_map.begin(),
              first_layer.m_mat.m_soilMaterialTexture->m_metallic_map.end(), 0.2f);

    first_layer.m_mat.m_soilMaterialTexture->m_roughness_map.resize(sd->texture_resolution.x *
                                                                   sd->texture_resolution.y);
    std::fill(first_layer.m_mat.m_soilMaterialTexture->m_roughness_map.begin(),
              first_layer.m_mat.m_soilMaterialTexture->m_roughness_map.end(), 0.8f);

    first_layer.m_mat.m_soilMaterialTexture->m_normal_map.resize(sd->texture_resolution.x *
                                                                sd->texture_resolution.y);
    std::fill(first_layer.m_mat.m_soilMaterialTexture->m_normal_map.begin(),
              first_layer.m_mat.m_soilMaterialTexture->m_normal_map.end(), glm::vec3(0.0f, 0.0f, 1.0f));

    material_index++;
    // Add user defined layers
    auto& soil_layer_descriptors = sd->soil_layer_descriptors;
    for (int i = 0; i < sd->soil_layer_descriptors.size(); i++) {
      if (auto soil_layer_descriptor = soil_layer_descriptors[i].Get<SoilLayerDescriptor>()) {
        soil_layers.emplace_back();
        auto& soil_layer = soil_layers.back();
        soil_layer.m_mat.m_c = [=](const glm::vec3& position) {
          return soil_layer_descriptor->capacity.GetValue(position);
        };
        soil_layer.m_mat.m_p = [=](const glm::vec3& position) {
          return soil_layer_descriptor->permeability.GetValue(position);
        };
        soil_layer.m_mat.m_d = [=](const glm::vec3& position) {
          return soil_layer_descriptor->density.GetValue(position);
        };
        soil_layer.m_mat.m_n = [=](const glm::vec3& position) {
          return soil_layer_descriptor->initial_nutrients.GetValue(position);
        };
        soil_layer.m_mat.m_w = [=](const glm::vec3& position) {
          return soil_layer_descriptor->initial_water.GetValue(position);
        };
        soil_layer.m_mat.m_id = material_index;
        soil_layer.m_thickness = [soil_layer_descriptor](const glm::vec2& position) {
          return soil_layer_descriptor->thickness.GetValue(position);
        };
        const auto albedo = soil_layer_descriptor->albedo_texture.Get<Texture2D>();
        const auto height = soil_layer_descriptor->height_texture.Get<Texture2D>();
        const auto metallic = soil_layer_descriptor->metallic_texture.Get<Texture2D>();
        const auto normal = soil_layer_descriptor->normal_texture.Get<Texture2D>();
        const auto roughness = soil_layer_descriptor->roughness_texture.Get<Texture2D>();
        soil_layer.m_mat.m_soilMaterialTexture = std::make_shared<SoilMaterialTexture>();
        if (albedo) {
          albedo->GetRgbaChannelData(soil_layer.m_mat.m_soilMaterialTexture->m_color_map,
                                     sd->texture_resolution.x, sd->texture_resolution.y);
          if (i == 0) {
            albedo->GetRgbaChannelData(soil_layers[0].m_mat.m_soilMaterialTexture->m_color_map,
                                       sd->texture_resolution.x, sd->texture_resolution.y);
            for (auto& value : soil_layers[0].m_mat.m_soilMaterialTexture->m_color_map)
              value.w = 0.0f;
          }
        } else {
          soil_layer.m_mat.m_soilMaterialTexture->m_color_map.resize(sd->texture_resolution.x *
                                                                    sd->texture_resolution.y);
          std::fill(soil_layer.m_mat.m_soilMaterialTexture->m_color_map.begin(),
                    soil_layer.m_mat.m_soilMaterialTexture->m_color_map.end(),
                    Application::GetLayer<EcoSysLabLayer>()->m_soilLayerColors[material_index]);
        }
        if (height) {
          height->GetRedChannelData(soil_layer.m_mat.m_soilMaterialTexture->m_height_map,
                                    sd->texture_resolution.x, sd->texture_resolution.y);
        } else {
          soil_layer.m_mat.m_soilMaterialTexture->m_height_map.resize(sd->texture_resolution.x *
                                                                     sd->texture_resolution.y);
          std::fill(soil_layer.m_mat.m_soilMaterialTexture->m_height_map.begin(),
                    soil_layer.m_mat.m_soilMaterialTexture->m_height_map.end(), 1.0f);
        }
        if (metallic) {
          metallic->GetRedChannelData(soil_layer.m_mat.m_soilMaterialTexture->m_metallic_map,
                                      sd->texture_resolution.x, sd->texture_resolution.y);
        } else {
          soil_layer.m_mat.m_soilMaterialTexture->m_metallic_map.resize(sd->texture_resolution.x *
                                                                       sd->texture_resolution.y);
          std::fill(soil_layer.m_mat.m_soilMaterialTexture->m_metallic_map.begin(),
                    soil_layer.m_mat.m_soilMaterialTexture->m_metallic_map.end(), 0.2f);
        }
        if (roughness) {
          roughness->GetRedChannelData(soil_layer.m_mat.m_soilMaterialTexture->m_roughness_map,
                                       sd->texture_resolution.x, sd->texture_resolution.y);
        } else {
          soil_layer.m_mat.m_soilMaterialTexture->m_roughness_map.resize(sd->texture_resolution.x *
                                                                        sd->texture_resolution.y);
          std::fill(soil_layer.m_mat.m_soilMaterialTexture->m_roughness_map.begin(),
                    soil_layer.m_mat.m_soilMaterialTexture->m_roughness_map.end(), 0.8f);
        }
        if (normal) {
          normal->GetRgbChannelData(soil_layer.m_mat.m_soilMaterialTexture->m_normal_map,
                                    sd->texture_resolution.x, sd->texture_resolution.y);
        } else {
          soil_layer.m_mat.m_soilMaterialTexture->m_normal_map.resize(sd->texture_resolution.x *
                                                                     sd->texture_resolution.y);
          std::fill(soil_layer.m_mat.m_soilMaterialTexture->m_normal_map.begin(),
                    soil_layer.m_mat.m_soilMaterialTexture->m_normal_map.end(), glm::vec3(0, 0, 1));
        }
        material_index++;
      } else {
        soil_layer_descriptors.erase(soil_layer_descriptors.begin() + i);
        i--;
      }
    }

    // Add bottom layer
    soil_layers.emplace_back();
    soil_layers.back().m_thickness = [](const glm::vec2& position) {
      return 1000.f;
    };
    soil_layers.back().m_mat.m_id = material_index;
    soil_layers.back().m_mat.m_c = [](const glm::vec3& position) {
      return 1000.f;
    };
    soil_layers.back().m_mat.m_p = [](const glm::vec3& position) {
      return 0.0f;
    };
    soil_layers.back().m_mat.m_d = [](const glm::vec3& position) {
      return 1000.f;
    };
    soil_layers.back().m_mat.m_n = [](const glm::vec3& position) {
      return 0.0f;
    };
    soil_layers.back().m_mat.m_w = [](const glm::vec3& position) {
      return 0.0f;
    };
    soil_model.Initialize(params, soil_surface, soil_layers);
  }
}

void Soil::SplitRootTestSetup() {
  InitializeSoilModel();
  if (const auto sd = soil_descriptor.Get<SoilDescriptor>()) {
    const auto height_field = sd->height_field.Get<HeightField>();
    for (int i = 0; i < soil_model.m_n.size(); i++) {
      auto position = soil_model.GetPositionFromCoordinate(soil_model.GetCoordinateFromIndex(i));
      bool under_ground = true;
      if (height_field) {
        auto height = height_field->GetValue(glm::vec2(position.x, position.z));
        if (position.y >= height)
          under_ground = false;
      }
      if (under_ground) {
        if (position.x > soil_model.m_boundingBoxMin.x &&
            position.x <
                soil_model.GetVoxelResolution().x * soil_model.m_dx * 0.25f + soil_model.m_boundingBoxMin.x) {
          soil_model.m_n[i] = 0.75f;
        } else if (position.x <
                   soil_model.GetVoxelResolution().x * soil_model.m_dx * 0.5f + soil_model.m_boundingBoxMin.x) {
          soil_model.m_n[i] = 0.75f;
        } else if (position.x <
                   soil_model.GetVoxelResolution().x * soil_model.m_dx * 0.75f + soil_model.m_boundingBoxMin.x) {
          soil_model.m_n[i] = 1.25f;
        } else {
          soil_model.m_n[i] = 1.25f;
        }
      } else {
        soil_model.m_n[i] = 0.0f;
      }
    }
  }
}

void Soil::FixedUpdate() {
  if (temporal_progression_) {
    if (temporal_progression_progress_ < 1.0f) {
      auto scene = Application::GetActiveScene();
      auto owner = GetOwner();
      for (const auto& child : scene->GetChildren(owner)) {
        if (scene->GetEntityName(child) == "CutOut") {
          scene->DeleteEntity(child);
          break;
        }
      }
      const auto cut_out_entity = GenerateCutOut(temporal_progression_progress_, 0.99f, 0, 0, true);
      scene->SetParent(cut_out_entity, owner);
      temporal_progression_progress_ += 0.01f;
    } else {
      temporal_progression_progress_ = 0;
      temporal_progression_ = false;
    }
  }
}

void SerializeSoilParameters(const std::string& name, const SoilParameters& soil_parameters, YAML::Emitter& out) {
  out << YAML::Key << name << YAML::BeginMap;
  out << YAML::Key << "m_voxelResolution" << YAML::Value << soil_parameters.m_voxelResolution;
  out << YAML::Key << "m_deltaX" << YAML::Value << soil_parameters.m_deltaX;
  out << YAML::Key << "m_deltaTime" << YAML::Value << soil_parameters.m_deltaTime;
  out << YAML::Key << "m_boundingBoxMin" << YAML::Value << soil_parameters.m_boundingBoxMin;

  out << YAML::Key << "m_boundary_x" << YAML::Value << static_cast<int>(soil_parameters.m_boundary_x);
  out << YAML::Key << "m_boundary_y" << YAML::Value << static_cast<int>(soil_parameters.m_boundary_y);
  out << YAML::Key << "m_boundary_z" << YAML::Value << static_cast<int>(soil_parameters.m_boundary_z);

  out << YAML::Key << "m_diffusionForce" << YAML::Value << soil_parameters.m_diffusionForce;
  out << YAML::Key << "m_gravityForce" << YAML::Value << soil_parameters.m_gravityForce;
  out << YAML::EndMap;
}

void DeserializeSoilParameters(const std::string& name, SoilParameters& soil_parameters, const YAML::Node& in) {
  if (in[name]) {
    auto& param = in[name];
    if (param["m_voxelResolution"])
      soil_parameters.m_voxelResolution = param["m_voxelResolution"].as<glm::uvec3>();
    else {
      EVOENGINE_WARNING("DeserializeSoilParameters: m_voxelResolution not found!");
      // EVOENGINE_ERROR("DeserializeSoilParameters: m_voxelResolution not found!");
      // EVOENGINE_LOG("DeserializeSoilParameters: m_voxelResolution not found!");
    }
    if (param["m_deltaX"])
      soil_parameters.m_deltaX = param["m_deltaX"].as<float>();
    if (param["m_deltaTime"])
      soil_parameters.m_deltaTime = param["m_deltaTime"].as<float>();
    if (param["m_boundingBoxMin"])
      soil_parameters.m_boundingBoxMin = param["m_boundingBoxMin"].as<glm::vec3>();

    if (param["m_boundary_x"])
      soil_parameters.m_boundary_x = static_cast<VoxelSoilModel::Boundary>(param["m_boundary_x"].as<int>());
    if (param["m_boundary_y"])
      soil_parameters.m_boundary_y = static_cast<VoxelSoilModel::Boundary>(param["m_boundary_y"].as<int>());
    if (param["m_boundary_z"])
      soil_parameters.m_boundary_z = static_cast<VoxelSoilModel::Boundary>(param["m_boundary_z"].as<int>());

    if (param["m_diffusionForce"])
      soil_parameters.m_diffusionForce = param["m_diffusionForce"].as<float>();
    if (param["m_gravityForce"])
      soil_parameters.m_gravityForce = param["m_gravityForce"].as<glm::vec3>();
  }
}

void SoilDescriptor::Serialize(YAML::Emitter& out) const {
  height_field.Save("height_field", out);
  SerializeSoilParameters("soil_parameters", soil_parameters, out);

  out << YAML::Key << "soil_layer_descriptors" << YAML::Value << YAML::BeginSeq;
  for (int i = 0; i < soil_layer_descriptors.size(); i++) {
    out << YAML::BeginMap;
    soil_layer_descriptors[i].Serialize(out);
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
}

void SoilDescriptor::Deserialize(const YAML::Node& in) {
  height_field.Load("height_field", in);
  DeserializeSoilParameters("soil_parameters", soil_parameters, in);
  soil_layer_descriptors.clear();
  if (in["soil_layer_descriptors"]) {
    for (const auto& i : in["soil_layer_descriptors"]) {
      soil_layer_descriptors.emplace_back();
      soil_layer_descriptors.back().Deserialize(i);
    }
  }
}

void SoilDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(height_field);

  for (int i = 0; i < soil_layer_descriptors.size(); i++) {
    if (auto soil_layer_descriptor = soil_layer_descriptors[i].Get<SoilLayerDescriptor>()) {
      list.push_back(soil_layer_descriptors[i]);
    } else {
      soil_layer_descriptors.erase(soil_layer_descriptors.begin() + i);
      i--;
    }
  }
}
