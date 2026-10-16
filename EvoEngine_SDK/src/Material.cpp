#include "Material.hpp"

#include "RenderLayer.hpp"
#include "Texture2D.hpp"

#include <algorithm>
#include <cmath>

using namespace evo_engine;

void Material::CollectAssetRef(std::vector<AssetRef>& list) {
  ResizeTextureRefs();
  if (subsurface_profile.GetAssetHandle() != 0)
    list.push_back(subsurface_profile);
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
  if (tex_coord < 0 || tex_coord > 1) {
    EVOENGINE_ERROR("Texture binding disabled because TEXCOORD_" + std::to_string(tex_coord) +
                    " is outside the supported range 0..1.")
    return SetTextureRef(slot, AssetRef{}, 0, uv_transform);
  }
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
  texture_info.tex_coord = tex_coord;
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
  RefreshSubsurfaceProfile();
  auto result = material_data;
  auto& shade = result.shade_material;
  if (const auto profile = subsurface_profile.Get<SubsurfaceProfile>()) {
    const float scale = std::isfinite(subsurface_distance_scale) ? std::max(subsurface_distance_scale, 0.0f) : 1.0f;
    for (int channel = 0; channel < 3; ++channel) {
      shade.subsurface_albedo[channel] =
          std::isfinite(profile->albedo[channel]) ? std::clamp(profile->albedo[channel], 0.0f, 1.0f) : 1.0f;
      const float distance = profile->scattering_distance[channel] * scale;
      shade.subsurface_distance[channel] = std::isfinite(distance) ? std::max(distance, 1e-6f) : 0.001f;
    }
    shade.subsurface_ior = std::isfinite(profile->ior) ? std::max(profile->ior, 1.0f) : 1.4f;
    shade.subsurface_weight =
        std::isfinite(shade.subsurface_weight) ? std::clamp(shade.subsurface_weight, 0.0f, 1.0f) : 0.0f;
    shade.subsurface_thickness =
        std::isfinite(shade.subsurface_thickness) ? std::max(shade.subsurface_thickness, 0.0f) : 0.0f;
    if (shade.subsurface_weight > 0.0f && shade.subsurface_mode == static_cast<uint32_t>(SubsurfaceMode::ThinSurface))
      shade.double_sided = 1;
    const uint64_t handle = profile->GetHandle().GetValue();
    shade.subsurface_profile_lo = static_cast<uint32_t>(handle);
    shade.subsurface_profile_hi = static_cast<uint32_t>(handle >> 32u);
  } else {
    shade.subsurface_weight = 0.0f;
    shade.subsurface_profile_lo = shade.subsurface_profile_hi = 0;
  }
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
  const auto& shade = material_data.shade_material;
  const bool thin_sss = subsurface_profile.GetAssetHandle() != 0 && shade.subsurface_weight > 0.0f &&
                        shade.subsurface_mode == static_cast<uint32_t>(SubsurfaceMode::ThinSurface);
  draw_settings.cull_mode = shade.double_sided != 0 || thin_sss ? VK_CULL_MODE_NONE : VK_CULL_MODE_BACK_BIT;
}

void Material::MarkDirty() {
  SyncRenderStateFromGltfMaterial();
  need_update_ = true;
}

void Material::RefreshSubsurfaceProfile() {
  const uint64_t handle = subsurface_profile.GetAssetHandle().GetValue();
  if (handle == 0 && subsurface_profile_handle_ == 0 && !subsurface_profile_resident_)
    return;
  const auto profile = subsurface_profile.Get<SubsurfaceProfile>();
  const uint32_t version = profile ? profile->GetVersion() : 0;
  if (subsurface_profile_handle_ != handle || subsurface_profile_version_ != version ||
      subsurface_profile_resident_ != static_cast<bool>(profile)) {
    subsurface_profile_handle_ = handle;
    subsurface_profile_version_ = version;
    subsurface_profile_resident_ = static_cast<bool>(profile);
    SetUnsaved();
    MarkDirty();
  }
}
