#include "Material.hpp"

#include "EditorLayer.hpp"
#include "RenderLayer.hpp"
#include "Texture2D.hpp"

#include <algorithm>

using namespace evo_engine;

void Material::CollectAssetRef(std::vector<AssetRef>& list) {
  ResizeTextureRefs();
  for (const auto& texture_ref : texture_refs_) {
    list.push_back(texture_ref);
  }
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

std::shared_ptr<Texture2D> Material::GenerateThumbnailTexture() {
  return EditorLayer::FindIcon("Material");
}

Material::Material() {
  material_data.shade_material.pbr_metallic_factor = 0.0f;
  SyncRenderStateFromGltfMaterial();
}

Material::~Material() {
  for (auto& texture_ref : texture_refs_) {
    texture_ref.Clear();
  }
}

void Material::ResizeTextureRefs() {
  if (material_data.texture_infos.empty()) {
    material_data.texture_infos.emplace_back();
  }
  if (texture_refs_.size() < material_data.texture_infos.size()) {
    texture_refs_.resize(material_data.texture_infos.size());
  }
}

uint16_t Material::SetTexture(uint16_t GltfShadeMaterial::* slot, const std::shared_ptr<Texture2D>& texture,
                              const int32_t tex_coord, const glm::mat3x2& uv_transform) {
  AssetRef texture_ref(texture);
  return SetTextureRef(slot, texture_ref, tex_coord, uv_transform);
}

void Material::SetTexture(const uint16_t texture_info_slot, const std::shared_ptr<Texture2D>& texture) {
  AssetRef texture_ref(texture);
  SetTextureRef(texture_info_slot, texture_ref);
}

uint16_t Material::SetTextureRef(uint16_t GltfShadeMaterial::* slot, const AssetRef& texture_ref,
                                 const int32_t tex_coord, const glm::mat3x2& uv_transform) {
  auto resolved_ref = texture_ref;
  const auto texture = resolved_ref.Get<Texture2D>();
  uint16_t texture_info_slot = material_data.shade_material.*slot;
  if (!texture && texture_ref.GetAssetHandle() == 0) {
    if (texture_info_slot > 0 && texture_info_slot < texture_refs_.size()) {
      texture_refs_[texture_info_slot].Clear();
    }
    if (texture_info_slot > 0 && texture_info_slot < material_data.texture_infos.size()) {
      material_data.texture_infos[texture_info_slot].index = -1;
    }
    material_data.shade_material.*slot = 0;
    need_update_ = true;
    return 0;
  }
  ResizeTextureRefs();
  if (texture_info_slot == 0 || texture_info_slot >= material_data.texture_infos.size()) {
    texture_info_slot = static_cast<uint16_t>(material_data.texture_infos.size());
    material_data.texture_infos.emplace_back();
    texture_refs_.emplace_back();
    material_data.shade_material.*slot = texture_info_slot;
  }
  auto& texture_info = material_data.texture_infos[texture_info_slot];
  texture_info.index = texture ? static_cast<int32_t>(texture->GetTextureStorageIndex()) : -1;
  texture_info.tex_coord = std::clamp(tex_coord, 0, 1);
#if MAT_EXT_TEXTURE_TRANSFORM
  texture_info.uv_transform = uv_transform;
#endif
  texture_refs_[texture_info_slot] = texture_ref;
  need_update_ = true;
  return texture_info_slot;
}

void Material::SetTextureRef(const uint16_t texture_info_slot, const AssetRef& texture_ref) {
  ResizeTextureRefs();
  if (texture_info_slot == 0 || texture_info_slot >= material_data.texture_infos.size()) {
    return;
  }
  auto resolved_ref = texture_ref;
  const auto texture = resolved_ref.Get<Texture2D>();
  material_data.texture_infos[texture_info_slot].index =
      texture ? static_cast<int32_t>(texture->GetTextureStorageIndex()) : -1;
  texture_refs_[texture_info_slot] = texture_ref;
  need_update_ = true;
}

std::shared_ptr<Texture2D> Material::GetTexture(uint16_t GltfShadeMaterial::* slot) {
  return GetTexture(material_data.shade_material.*slot);
}

std::shared_ptr<Texture2D> Material::GetTexture(const uint16_t texture_info_slot) {
  if (texture_info_slot == 0 || texture_info_slot >= texture_refs_.size()) {
    return {};
  }
  return texture_refs_[texture_info_slot].Get<Texture2D>();
}

const std::vector<AssetRef>& Material::PeekTextureRefs() const {
  return texture_refs_;
}

std::vector<AssetRef>& Material::RefTextureRefs() {
  ResizeTextureRefs();
  need_update_ = true;
  return texture_refs_;
}

GltfMaterialData Material::BuildGltfMaterialData() {
  ResizeTextureRefs();
  auto result = material_data;
  for (size_t i = 1; i < result.texture_infos.size() && i < texture_refs_.size(); ++i) {
    if (const auto texture = texture_refs_[i].Get<Texture2D>()) {
      result.texture_infos[i].index = static_cast<int32_t>(texture->GetTextureStorageIndex());
    }
  }
  return result;
}

void Material::SetGltfMaterialData(const GltfMaterialData& data) {
  material_data = data;
  texture_refs_.resize(material_data.texture_infos.size());
  SyncRenderStateFromGltfMaterial();
  need_update_ = true;
}

void Material::SyncRenderStateFromGltfMaterial() {
  draw_settings.blending = GltfMaterialRequiresTransparentPass(material_data.shade_material);
  draw_settings.cull_mode = material_data.shade_material.double_sided != 0 ? VK_CULL_MODE_NONE : VK_CULL_MODE_BACK_BIT;
}

void Material::MarkDirty() {
  SyncRenderStateFromGltfMaterial();
  need_update_ = true;
}
