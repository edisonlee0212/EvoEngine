#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "HeightField.hpp"
#include "Material.hpp"
using namespace eco_sys_lab_plugin;

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

void SetSoilPhysicalMaterial(procedural_noise::ProceduralNoise3D& c, procedural_noise::ProceduralNoise3D& p,
                             float sand_ratio, float silt_ratio, float clay_ratio, float compactness) {
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
  /*
  c.noise_descriptors.resize(1);
  p.noise_descriptors.resize(1);
  c.noise_descriptors[0].type = 0;
  c.noise_descriptors[1].type = 0;
  c.noise_descriptors[0].offset = sand_ratio * sand_material_properties.x + silt_ratio * silt_material_properties.x +
                                  clay_ratio * clay_material_properties.x + air_ratio * air_material_properties.x;
  p.noise_descriptors[0].offset = sand_ratio * sand_material_properties.y + silt_ratio * silt_material_properties.y +
                                  clay_ratio * clay_material_properties.y + air_ratio * air_material_properties.y;*/
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
      SetSoilPhysicalMaterial(capacity_graph, permeability_graph, sand_ratio, silt_ratio, clay_ratio, compactness);
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
  static bool show_capacity = false;
  ImGui::Checkbox("Show capacity", &show_capacity);
  if (show_capacity) {
    changed = capacity_graph.ShowGraph("Capacity graph", editor_layer) | changed;
  }

  static bool show_permeability = false;
  ImGui::Checkbox("Show permeability", &show_permeability);
  if (show_permeability) {
    changed = permeability_graph.ShowGraph("Permeability graph", editor_layer) | changed;
  }
  static bool show_density = false;
  ImGui::Checkbox("Show density", &show_density);
  if (show_density) {
    changed = density_graph.ShowGraph("Density graph", editor_layer) | changed;
  }
  static bool show_initial_nutrients = false;
  ImGui::Checkbox("Show initial nutrients", &show_initial_nutrients);
  if (show_initial_nutrients) {
    changed = initial_nutrients_graph.ShowGraph("Initial nutrients graph", editor_layer) | changed;
  }
  static bool show_initial_water = false;
  ImGui::Checkbox("Show initial water", &show_initial_water);
  if (show_initial_water) {
    changed = initial_water_graph.ShowGraph("Initial water graph", editor_layer) | changed;
  }
  static bool show_thickness = false;
  ImGui::Checkbox("Show thickness", &show_thickness);
  if (show_thickness) {
    changed = thickness_graph.ShowGraph("Thickness graph", editor_layer) | changed;
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
  capacity_graph.Save("capacity_graph", out);
  permeability_graph.Save("permeability_graph", out);
  density_graph.Save("density_graph", out);
  initial_nutrients_graph.Save("initial_nutrients_graph", out);
  initial_water_graph.Save("initial_water_graph", out);

  thickness_graph.Save("thickness_graph", out);

  albedo_texture.Save("albedo_texture", out);
  roughness_texture.Save("roughness_texture", out);
  metallic_texture.Save("metallic_texture", out);
  normal_texture.Save("normal_texture", out);
  height_texture.Save("height_texture", out);
}

void SoilLayerDescriptor::Deserialize(const YAML::Node& in) {
  capacity_graph.Load("capacity_graph", in);
  permeability_graph.Load("permeability_graph", in);
  density_graph.Load("density_graph", in);
  initial_nutrients_graph.Load("initial_nutrients_graph", in);
  initial_water_graph.Load("initial_water_graph", in);
  thickness_graph.Load("thickness_graph", in);

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

std::shared_ptr<Texture2D> SoilDescriptor::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources") / "Icons/SoilDescriptor.png"));
  }
  return thumbnail;
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
    auto scene = ApplicationContext::Get().GetActiveScene();
    auto soil_entity = scene->CreateEntity(GetTitle());
    auto soil = scene->GetOrSetPrivateComponent<Soil>(soil_entity).lock();
    soil->soil_descriptor_ref = GetSelf();
    soil->InitializeSoilModel();
  }

  if (OnInspectSoilParameters(soil_parameters)) {
    changed = true;
  }
  if (AssetRef temp_soil_layer_descriptor_holder; editor_layer->DragAndDropButton<SoilLayerDescriptor>(
          temp_soil_layer_descriptor_holder, "Drop new SoilLayerDescriptor here...")) {
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