#include "Material.hpp"

#include "EditorLayer.hpp"
#include "RenderLayer.hpp"
#include "Texture2D.hpp"
using namespace evo_engine;

const char* polygon_mode_string[]{"Point", "Line", "Fill"};
const char* culling_mode_string[]{"Front", "Back", "FrontAndBack", "None"};
const char* blending_factor_string[]{"Zero",
                                     "One",
                                     "SrcColor",
                                     "OneMinusSrcColor",
                                     "DstColor",
                                     "OneMinusDstColor",
                                     "SrcAlpha",
                                     "OneMinusSrcAlpha",
                                     "DstAlpha",
                                     "OneMinusDstAlpha",
                                     "ConstantColor",
                                     "OneMinusConstantColor",
                                     "ConstantAlpha",
                                     "OneMinusConstantAlpha",
                                     "SrcAlphaSaturate",
                                     "Src1Color",
                                     "OneMinusSrc1Color",
                                     "Src1Alpha",
                                     "OneMinusSrc1Alpha"};

void Material::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(albedo_texture_);
  list.push_back(normal_texture_);
  list.push_back(metallic_texture_);
  list.push_back(roughness_texture_);
  list.push_back(ao_texture_);
}

bool DrawSettings::OnInspect() {
  bool changed = false;
  int polygon_mode_tmp = 0;
  switch (polygon_mode) {
    case VK_POLYGON_MODE_POINT:
      polygon_mode_tmp = 0;
      break;
    case VK_POLYGON_MODE_LINE:
      polygon_mode_tmp = 1;
      break;
    case VK_POLYGON_MODE_FILL:
      polygon_mode_tmp = 2;
      break;
  }
  if (ImGui::Combo("Polygon Mode", &polygon_mode_tmp, polygon_mode_string, IM_ARRAYSIZE(polygon_mode_string))) {
    changed = true;
    switch (polygon_mode_tmp) {
      case 0:
        polygon_mode = VK_POLYGON_MODE_POINT;
        break;
      case 1:
        polygon_mode = VK_POLYGON_MODE_LINE;
        break;
      case 2:
        polygon_mode = VK_POLYGON_MODE_FILL;
        break;
    }
  }
  if (polygon_mode == VK_POLYGON_MODE_LINE) {
    ImGui::DragFloat("Line width", &line_width, 0.1f, 0.0f, 100.0f);
  }
  int cull_face_mode_tmp = 0;
  switch (cull_mode) {
    case VK_CULL_MODE_FRONT_BIT:
      cull_face_mode_tmp = 0;
      break;
    case VK_CULL_MODE_BACK_BIT:
      cull_face_mode_tmp = 1;
      break;
    case VK_CULL_MODE_FRONT_AND_BACK:
      cull_face_mode_tmp = 2;
      break;
    case VK_CULL_MODE_NONE:
      cull_face_mode_tmp = 3;
      break;
  }
  if (ImGui::Combo("Cull Face Mode", &cull_face_mode_tmp, culling_mode_string, IM_ARRAYSIZE(culling_mode_string))) {
    changed = true;
    switch (cull_face_mode_tmp) {
      case 0:
        cull_mode = VK_CULL_MODE_FRONT_BIT;
        break;
      case 1:
        cull_mode = VK_CULL_MODE_BACK_BIT;
        break;
      case 2:
        cull_mode = VK_CULL_MODE_FRONT_AND_BACK;
        break;
      case 3:
        cull_mode = VK_CULL_MODE_NONE;
        break;
    }
  }

  if (ImGui::Checkbox("Blending", &blending))
    changed = true;

  if (false && blending) {
    if (ImGui::Combo("Blending Source Factor", reinterpret_cast<int*>(&blending_src_factor), blending_factor_string,
                     IM_ARRAYSIZE(blending_factor_string))) {
      changed = true;
    }
    if (ImGui::Combo("Blending Destination Factor", reinterpret_cast<int*>(&blending_dst_factor),
                     blending_factor_string, IM_ARRAYSIZE(blending_factor_string))) {
      changed = true;
    }
  }
  return changed;
}

void DrawSettings::ApplySettings(GraphicsPipelineStates& global_pipeline_state) const {
  global_pipeline_state.cull_mode = cull_mode;
  global_pipeline_state.polygon_mode = polygon_mode;
  global_pipeline_state.line_width = line_width;
  for (auto& i : global_pipeline_state.color_blend_attachment_states) {
    i.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    i.blendEnable = blending;
    i.srcAlphaBlendFactor = i.srcColorBlendFactor = blending_src_factor;
    i.dstAlphaBlendFactor = i.dstColorBlendFactor = blending_dst_factor;
    i.colorBlendOp = i.alphaBlendOp = blend_op;
  }
}

void DrawSettings::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "cull_mode" << YAML::Value << cull_mode;
  out << YAML::Key << "line_width" << YAML::Value << line_width;
  out << YAML::Key << "polygon_mode" << YAML::Value << static_cast<unsigned>(polygon_mode);
  out << YAML::Key << "blending" << YAML::Value << blending;
  out << YAML::Key << "blending_src_factor" << YAML::Value << static_cast<unsigned>(blending_src_factor);
  out << YAML::Key << "blending_dst_factor" << YAML::Value << static_cast<unsigned>(blending_dst_factor);
  out << YAML::EndMap;
}

void DrawSettings::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& draw_settings = in[name];
    if (draw_settings["cull_mode"])
      cull_mode = draw_settings["cull_mode"].as<unsigned>();
    if (draw_settings["line_width"])
      line_width = draw_settings["line_width"].as<float>();
    if (draw_settings["polygon_mode"])
      polygon_mode = static_cast<VkPolygonMode>(draw_settings["polygon_mode"].as<unsigned>());

    if (draw_settings["blending"])
      blending = draw_settings["blending"].as<bool>();
    if (draw_settings["blending_src_factor"])
      blending_src_factor = static_cast<VkBlendFactor>(draw_settings["blending_src_factor"].as<unsigned>());
    if (draw_settings["blending_dst_factor"])
      blending_dst_factor = static_cast<VkBlendFactor>(draw_settings["blending_dst_factor"].as<unsigned>());
  }
}

Material::~Material() {
  albedo_texture_.Clear();
  normal_texture_.Clear();
  metallic_texture_.Clear();
  roughness_texture_.Clear();
  ao_texture_.Clear();
}

void Material::SetAlbedoTexture(const std::shared_ptr<Texture2D>& texture) {
  albedo_texture_ = texture;
  need_update_ = true;
}

void Material::SetNormalTexture(const std::shared_ptr<Texture2D>& texture) {
  normal_texture_ = texture;
  need_update_ = true;
}

void Material::SetMetallicTexture(const std::shared_ptr<Texture2D>& texture) {
  metallic_texture_ = texture;
  need_update_ = true;
}

void Material::SetRoughnessTexture(const std::shared_ptr<Texture2D>& texture) {
  roughness_texture_ = texture;
  need_update_ = true;
}

void Material::SetAoTexture(const std::shared_ptr<Texture2D>& texture) {
  ao_texture_ = texture;
  need_update_ = true;
}

std::shared_ptr<Texture2D> Material::GetAlbedoTexture() {
  return albedo_texture_.Get<Texture2D>();
}

std::shared_ptr<Texture2D> Material::GetNormalTexture() {
  return normal_texture_.Get<Texture2D>();
}

std::shared_ptr<Texture2D> Material::GetMetallicTexture() {
  return metallic_texture_.Get<Texture2D>();
}

std::shared_ptr<Texture2D> Material::GetRoughnessTexture() {
  return roughness_texture_.Get<Texture2D>();
}

std::shared_ptr<Texture2D> Material::GetAoTexture() {
  return ao_texture_.Get<Texture2D>();
}

void Material::UpdateMaterialInfoBlock(MaterialInfoBlock& material_info_block) {
  if (const auto albedo_texture = albedo_texture_.Get<Texture2D>(); albedo_texture && albedo_texture->GetVkSampler()) {
    material_info_block.albedo_texture_index = albedo_texture->GetTextureStorageIndex();
  } else {
    material_info_block.albedo_texture_index = -1;
  }
  if (const auto normal_texture = normal_texture_.Get<Texture2D>(); normal_texture && normal_texture->GetVkSampler()) {
    material_info_block.normal_texture_index = normal_texture->GetTextureStorageIndex();
  } else {
    material_info_block.normal_texture_index = -1;
  }
  if (const auto metallic_texture = metallic_texture_.Get<Texture2D>();
      metallic_texture && metallic_texture->GetVkSampler()) {
    material_info_block.metallic_texture_index = metallic_texture->GetTextureStorageIndex();
  } else {
    material_info_block.metallic_texture_index = -1;
  }
  if (const auto roughness_texture = roughness_texture_.Get<Texture2D>();
      roughness_texture && roughness_texture->GetVkSampler()) {
    material_info_block.roughness_texture_index = roughness_texture->GetTextureStorageIndex();
  } else {
    material_info_block.roughness_texture_index = -1;
  }
  if (const auto ao_texture = ao_texture_.Get<Texture2D>(); ao_texture && ao_texture->GetVkSampler()) {
    material_info_block.ao_texture_index = ao_texture->GetTextureStorageIndex();
  } else {
    material_info_block.ao_texture_index = -1;
  }
  material_info_block.cast_shadow = true;
  material_info_block.subsurface_color = {material_properties.subsurface_color, 0.0f};
  material_info_block.subsurface_radius = {material_properties.subsurface_radius, 0.0f};
  material_info_block.albedo_color_val = glm::vec4(
      material_properties.albedo_color, draw_settings.blending ? (1.0f - material_properties.transmission) : 1.0f);
  material_info_block.metallic_val = material_properties.metallic;
  material_info_block.roughness_val = material_properties.roughness;
  material_info_block.ao_val = 1.0f;
  material_info_block.emission_val = material_properties.emission;
}

bool Material::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (ImGui::Checkbox("Vertex color only", &vertex_color_only)) {
    changed = true;
  }

  ImGui::Separator();
  if (ImGui::TreeNodeEx("PBR##Material", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::ColorEdit3("Albedo##Material", &material_properties.albedo_color.x)) {
      changed = true;
    }
    if (ImGui::DragFloat("Subsurface##Material", &material_properties.subsurface_factor, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (material_properties.subsurface_factor > 0.0f) {
      if (ImGui::DragFloat3("Subsurface Radius##Material", &material_properties.subsurface_radius.x, 0.01f, 0.0f,
                            999.0f)) {
        changed = true;
      }
      if (ImGui::ColorEdit3("Subsurface Color##Material", &material_properties.subsurface_color.x)) {
        changed = true;
      }
    }
    if (ImGui::DragFloat("Metallic##Material", &material_properties.metallic, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }

    if (ImGui::DragFloat("Specular##Material", &material_properties.specular, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Specular Tint##Material", &material_properties.specular_tint, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Roughness##Material", &material_properties.roughness, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Sheen##Material", &material_properties.sheen, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Sheen Tint##Material", &material_properties.sheen_tint, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Clear Coat##Material", &material_properties.clear_coat, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Clear Coat Roughness##Material", &material_properties.clear_coat_roughness, 0.01f, 0.0f,
                         1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("IOR##Material", &material_properties.ior, 0.01f, 0.0f, 5.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Transmission##Material", &material_properties.transmission, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Transmission Roughness##Material", &material_properties.transmission_roughness, 0.01f, 0.0f,
                         1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Emission##Material", &material_properties.emission, 0.01f, 0.0f, 10.0f)) {
      changed = true;
    }

    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Others##Material")) {
    if (draw_settings.OnInspect())
      changed = true;
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Textures##Material", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (editor_layer->DragAndDropButton<Texture2D>(albedo_texture_, "Albedo Tex")) {
      changed = true;
    }
    if (editor_layer->DragAndDropButton<Texture2D>(normal_texture_, "Normal Tex")) {
      changed = true;
    }
    if (editor_layer->DragAndDropButton<Texture2D>(metallic_texture_, "Metallic Tex")) {
      changed = true;
    }
    if (editor_layer->DragAndDropButton<Texture2D>(roughness_texture_, "Roughness Tex")) {
      changed = true;
    }
    if (editor_layer->DragAndDropButton<Texture2D>(ao_texture_, "AO Tex")) {
      changed = true;
    }

    static AssetRef rma_texture_ref{};
    if(EditorLayer::DragAndDropButton<Texture2D>(rma_texture_ref, "Apply RMA Texture")) {
      const auto rma_texture = rma_texture_ref.Get<Texture2D>();
      std::vector<glm::vec3> rma_data;
      rma_texture->GetRgbChannelData(rma_data);
      const auto rma_resolution = rma_texture->GetResolution();
      std::vector<glm::vec3> temp_data(rma_data.size());
      Jobs::RunParallelFor(temp_data.size(), [&](const size_t pixel_index) {
        temp_data[pixel_index] = glm::vec3(rma_data[pixel_index].x);
      });
      const auto roughness_texture = ProjectManager::CreateTemporaryAsset<Texture2D>();
      roughness_texture->SetRgbChannelData(temp_data, rma_resolution);

      Jobs::RunParallelFor(temp_data.size(), [&](const size_t pixel_index) {
        temp_data[pixel_index] = glm::vec3(rma_data[pixel_index].y);
      });
      const auto metallic_texture = ProjectManager::CreateTemporaryAsset<Texture2D>();
      metallic_texture->SetRgbChannelData(temp_data, rma_resolution);

      Jobs::RunParallelFor(temp_data.size(), [&](const size_t pixel_index) {
        temp_data[pixel_index] = glm::vec3(rma_data[pixel_index].z);
      });
      const auto ao_texture = ProjectManager::CreateTemporaryAsset<Texture2D>();
      ao_texture->SetRgbChannelData(temp_data, rma_resolution);

      roughness_texture_ = roughness_texture;
      metallic_texture_ = metallic_texture;
      ao_texture_ = ao_texture;

      rma_texture_ref.Clear();
    }
    ImGui::TreePop();
  }
  if (changed) {
    need_update_ = true;
  }
  return changed;
}
void SaveMaterialProperties(const std::string& name, const MaterialProperties& material_properties,
                            YAML::Emitter& out) {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "albedo_color" << YAML::Value << material_properties.albedo_color;
  out << YAML::Key << "subsurface_color" << YAML::Value << material_properties.subsurface_color;
  out << YAML::Key << "subsurface_factor" << YAML::Value << material_properties.subsurface_factor;
  out << YAML::Key << "subsurface_radius" << YAML::Value << material_properties.subsurface_radius;

  out << YAML::Key << "metallic" << YAML::Value << material_properties.metallic;
  out << YAML::Key << "specular" << YAML::Value << material_properties.specular;
  out << YAML::Key << "specular_tint" << YAML::Value << material_properties.specular_tint;
  out << YAML::Key << "roughness" << YAML::Value << material_properties.roughness;
  out << YAML::Key << "sheen" << YAML::Value << material_properties.sheen;
  out << YAML::Key << "sheen_tint" << YAML::Value << material_properties.sheen_tint;
  out << YAML::Key << "clear_coat" << YAML::Value << material_properties.clear_coat;
  out << YAML::Key << "clear_coat_roughness" << YAML::Value << material_properties.clear_coat_roughness;
  out << YAML::Key << "ior" << YAML::Value << material_properties.ior;
  out << YAML::Key << "transmission" << YAML::Value << material_properties.transmission;
  out << YAML::Key << "transmission_roughness" << YAML::Value << material_properties.transmission_roughness;
  out << YAML::Key << "emission" << YAML::Value << material_properties.emission;
  out << YAML::EndMap;
}
void LoadMaterialProperties(const std::string& name, MaterialProperties& material_properties, const YAML::Node& in) {
  if (in[name]) {
    const auto& in_material_properties = in[name];
    if (in_material_properties["albedo_color"])
      material_properties.albedo_color = in_material_properties["albedo_color"].as<glm::vec3>();
    if (in_material_properties["subsurface_color"])
      material_properties.subsurface_color = in_material_properties["subsurface_color"].as<glm::vec3>();
    if (in_material_properties["subsurface_factor"])
      material_properties.subsurface_factor = in_material_properties["subsurface_factor"].as<float>();
    if (in_material_properties["subsurface_radius"])
      material_properties.subsurface_radius = in_material_properties["subsurface_radius"].as<glm::vec3>();
    if (in_material_properties["metallic"])
      material_properties.metallic = in_material_properties["metallic"].as<float>();
    if (in_material_properties["specular"])
      material_properties.specular = in_material_properties["specular"].as<float>();
    if (in_material_properties["specular_tint"])
      material_properties.specular_tint = in_material_properties["specular_tint"].as<float>();
    if (in_material_properties["roughness"])
      material_properties.roughness = in_material_properties["roughness"].as<float>();
    if (in_material_properties["m_sheen"])
      material_properties.sheen = in_material_properties["sheen"].as<float>();
    if (in_material_properties["sheen_tint"])
      material_properties.sheen_tint = in_material_properties["sheen_tint"].as<float>();
    if (in_material_properties["clear_coat"])
      material_properties.clear_coat = in_material_properties["clear_coat"].as<float>();
    if (in_material_properties["clear_coat_roughness"])
      material_properties.clear_coat_roughness = in_material_properties["clear_coat_roughness"].as<float>();
    if (in_material_properties["ior"])
      material_properties.ior = in_material_properties["ior"].as<float>();
    if (in_material_properties["transmission"])
      material_properties.transmission = in_material_properties["transmission"].as<float>();
    if (in_material_properties["transmission_roughness"])
      material_properties.transmission_roughness = in_material_properties["transmission_roughness"].as<float>();
    if (in_material_properties["emission"])
      material_properties.emission = in_material_properties["emission"].as<float>();
  }
}
void Material::Serialize(YAML::Emitter& out) const {
  albedo_texture_.Save("albedo_texture_", out);
  normal_texture_.Save("normal_texture_", out);
  metallic_texture_.Save("metallic_texture_", out);
  roughness_texture_.Save("roughness_texture_", out);
  ao_texture_.Save("ao_texture_", out);

  draw_settings.Save("draw_settings", out);
  SaveMaterialProperties("material_properties", material_properties, out);
  out << YAML::Key << "vertex_color_only" << YAML::Value << vertex_color_only;
}

void Material::Deserialize(const YAML::Node& in) {
  albedo_texture_.Load("albedo_texture_", in);
  normal_texture_.Load("normal_texture_", in);
  metallic_texture_.Load("metallic_texture_", in);
  roughness_texture_.Load("roughness_texture_", in);
  ao_texture_.Load("ao_texture_", in);

  draw_settings.Load("draw_settings", in);
  LoadMaterialProperties("material_properties", material_properties, in);
  if (in["vertex_color_only"])
    vertex_color_only = in["vertex_color_only"].as<bool>();
  version_ = 0;
}
