#include "Material.hpp"

#include "EditorLayer.hpp"
#include "RenderLayer.hpp"
#include "Texture2D.hpp"

using namespace evo_engine;

void Material::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(albedo_texture_);
  list.push_back(normal_texture_);
  list.push_back(metallic_texture_);
  list.push_back(roughness_texture_);
  list.push_back(ao_texture_);
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

const AssetRef& Material::PeekAlbedoTextureRef() const {
  return albedo_texture_;
}

const AssetRef& Material::PeekNormalTextureRef() const {
  return normal_texture_;
}

const AssetRef& Material::PeekMetallicTextureRef() const {
  return metallic_texture_;
}

const AssetRef& Material::PeekRoughnessTextureRef() const {
  return roughness_texture_;
}

const AssetRef& Material::PeekAoTextureRef() const {
  return ao_texture_;
}

AssetRef& Material::RefAlbedoTextureRef() {
  return albedo_texture_;
}

AssetRef& Material::RefNormalTextureRef() {
  return normal_texture_;
}

AssetRef& Material::RefMetallicTextureRef() {
  return metallic_texture_;
}

AssetRef& Material::RefRoughnessTextureRef() {
  return roughness_texture_;
}

AssetRef& Material::RefAoTextureRef() {
  return ao_texture_;
}

void Material::MarkDirty() {
  need_update_ = true;
}
