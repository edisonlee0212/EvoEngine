#include "GltfMaterialCache.hpp"

#include "Console.hpp"
#include "GltfSpecularGlossinessConversion.hpp"
#include "Material.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace evo_engine;

namespace {
struct GltfTextureNodeInfo {
  bool present = false;
  int32_t texture_index = -1;
  int32_t tex_coord = 0;
  glm::mat3x2 uv_transform = glm::mat3x2(1.0f);
};

int32_t ClampTexCoord(const int32_t tex_coord) {
  return tex_coord >= 0 && tex_coord <= 1 ? tex_coord : -1;
}

float ReadFloat(const YAML::Node& node, const float fallback) {
  if (!node) {
    return fallback;
  }
  try {
    return node.as<float>();
  } catch (const YAML::Exception&) {
    return fallback;
  }
}

int32_t ReadInt(const YAML::Node& node, const int32_t fallback) {
  if (!node) {
    return fallback;
  }
  try {
    return node.as<int32_t>();
  } catch (const YAML::Exception&) {
    return fallback;
  }
}

bool ReadBool(const YAML::Node& node, const bool fallback) {
  if (!node) {
    return fallback;
  }
  try {
    return node.as<bool>();
  } catch (const YAML::Exception&) {
    return fallback;
  }
}

float ClampFloat(const float value, const float minimum, const float maximum) {
  return std::clamp(value, minimum, maximum);
}

YAML::Node ChildNode(const YAML::Node& node, const char* key) {
  if (!node || !node.IsMap()) {
    return {};
  }
  try {
    return node[key];
  } catch (const YAML::Exception&) {
    return {};
  }
}

void ReportMaterialError(const GltfMaterialErrorCallback& report_error, const std::string& message) {
  if (report_error) {
    report_error(message);
    return;
  }
  EVOENGINE_ERROR(message)
}

YAML::Node CompatibleMaterialExtension(const YAML::Node& extensions, const char* extension_name,
                                       const std::string& primary_model, const size_t material_index,
                                       const GltfMaterialErrorCallback& report_error) {
  const auto extension = ChildNode(extensions, extension_name);
  if (!extension || !extension.IsMap() || primary_model.empty() || primary_model == extension_name) {
    return extension;
  }
  ReportMaterialError(report_error, "glTF material " + std::to_string(material_index) + " rejects " + extension_name +
                                        " because it is incompatible with " + primary_model + ".");
  return {};
}

std::string ReadString(const YAML::Node& node, const std::string& fallback = {}) {
  if (!node) {
    return fallback;
  }
  try {
    return node.as<std::string>();
  } catch (const YAML::Exception&) {
    return fallback;
  }
}

glm::vec2 ReadVec2(const YAML::Node& node, const glm::vec2& fallback) {
  if (!node || !node.IsSequence() || node.size() < 2) {
    return fallback;
  }
  return {ReadFloat(node[0], fallback.x), ReadFloat(node[1], fallback.y)};
}

glm::vec3 ReadVec3(const YAML::Node& node, const glm::vec3& fallback) {
  if (!node || !node.IsSequence() || node.size() < 3) {
    return fallback;
  }
  return {ReadFloat(node[0], fallback.x), ReadFloat(node[1], fallback.y), ReadFloat(node[2], fallback.z)};
}

glm::vec4 ReadVec4(const YAML::Node& node, const glm::vec4& fallback) {
  if (!node || !node.IsSequence() || node.size() < 4) {
    return fallback;
  }
  return {ReadFloat(node[0], fallback.x), ReadFloat(node[1], fallback.y), ReadFloat(node[2], fallback.z),
          ReadFloat(node[3], fallback.w)};
}

glm::mat3x2 BuildTextureTransform(const glm::vec2& offset, const glm::vec2& scale, const float rotation) {
  const float cos_rotation = std::cos(rotation);
  const float sin_rotation = std::sin(rotation);
  return {scale.x * cos_rotation,
          scale.x * sin_rotation,
          -scale.y * sin_rotation,
          scale.y * cos_rotation,
          offset.x,
          offset.y};
}

glm::mat3x2 FlipTextureTransformY(const glm::mat3x2& uv_transform) {
  glm::mat3x2 result = uv_transform;
  result[0][1] = -result[0][1];
  result[1][1] = -result[1][1];
  result[2][1] = 1.0f - result[2][1];
  return result;
}

GltfTextureNodeInfo ReadTextureNodeInfo(const YAML::Node& texture_info, const size_t material_index,
                                        const GltfMaterialErrorCallback& report_error) {
  GltfTextureNodeInfo result;
  const auto texture_index = ChildNode(texture_info, "index");
  if (!texture_info || texture_info.IsNull() || !texture_index || texture_index.IsNull()) {
    return result;
  }

  result.present = true;
  result.texture_index = ReadInt(texture_index, -1);
  result.tex_coord = ReadInt(ChildNode(texture_info, "texCoord"), 0);
  if (const auto transform = ChildNode(ChildNode(texture_info, "extensions"), "KHR_texture_transform")) {
    const auto offset = ReadVec2(ChildNode(transform, "offset"), glm::vec2(0.0f));
    const auto scale = ReadVec2(ChildNode(transform, "scale"), glm::vec2(1.0f));
    const auto rotation = ReadFloat(ChildNode(transform, "rotation"), 0.0f);
    result.tex_coord = ReadInt(ChildNode(transform, "texCoord"), result.tex_coord);
    result.uv_transform = BuildTextureTransform(offset, scale, rotation);
  }
  if (ClampTexCoord(result.tex_coord) < 0) {
    ReportMaterialError(report_error, "glTF material " + std::to_string(material_index) + " disables texture " +
                                          std::to_string(result.texture_index) + " because TEXCOORD_" +
                                          std::to_string(result.tex_coord) + " is outside the supported range 0..1.");
    result.present = false;
  }
  return result;
}

void AssignTextureNode(GltfMaterialData& material_data, uint16_t GltfShadeMaterial::* slot,
                       const GltfTextureNodeInfo& texture_info,
                       const std::function<int32_t(int32_t texture_index, bool srgb)>& resolve_texture_index,
                       const std::function<bool(int32_t texture_index, bool srgb)>& texture_source_needs_y_flip,
                       const std::function<bool(int32_t texture_index, bool srgb)>& texture_source_decodes_srgb,
                       const bool srgb) {
  if (!texture_info.present) {
    return;
  }
  const auto texture_index = resolve_texture_index(texture_info.texture_index, srgb);
  const auto uv_transform = texture_source_needs_y_flip && texture_source_needs_y_flip(texture_info.texture_index, srgb)
                                ? FlipTextureTransformY(texture_info.uv_transform)
                                : texture_info.uv_transform;
  const auto color_space =
      srgb && !(texture_source_decodes_srgb && texture_source_decodes_srgb(texture_info.texture_index, srgb))
          ? GltfTextureColorSpace::Srgb
          : GltfTextureColorSpace::Linear;
  AssignGltfTextureSlot(material_data, slot, texture_index, texture_info.tex_coord, uv_transform, color_space);
}

void RemapSlot(uint16_t& slot, const std::vector<uint16_t>& remap) {
  slot = slot < remap.size() ? remap[slot] : 0;
}

void RemapTextureSlots(GltfShadeMaterial& material, const std::vector<uint16_t>& remap) {
  RemapSlot(material.pbr_base_color_texture, remap);
  RemapSlot(material.normal_texture, remap);
  RemapSlot(material.pbr_metallic_roughness_texture, remap);
  RemapSlot(material.emissive_texture, remap);
  RemapSlot(material.occlusion_texture, remap);
#if MAT_EXT_TRANSMISSION
  RemapSlot(material.transmission_texture, remap);
#endif
#if MAT_EXT_VOLUME
  RemapSlot(material.thickness_texture, remap);
#endif
#if MAT_EXT_CLEARCOAT
  RemapSlot(material.clearcoat_texture, remap);
  RemapSlot(material.clearcoat_roughness_texture, remap);
  RemapSlot(material.clearcoat_normal_texture, remap);
#endif
#if MAT_EXT_SPECULAR
  RemapSlot(material.specular_texture, remap);
  RemapSlot(material.specular_color_texture, remap);
#endif
#if MAT_EXT_IRIDESCENCE
  RemapSlot(material.iridescence_texture, remap);
  RemapSlot(material.iridescence_thickness_texture, remap);
#endif
#if MAT_EXT_ANISOTROPY
  RemapSlot(material.anisotropy_texture, remap);
#endif
#if MAT_EXT_SHEEN
  RemapSlot(material.sheen_color_texture, remap);
  RemapSlot(material.sheen_roughness_texture, remap);
#endif
#if MAT_EXT_DIFFUSE_TRANSMISSION
  RemapSlot(material.diffuse_transmission_texture, remap);
  RemapSlot(material.diffuse_transmission_color_texture, remap);
#endif
#if MAT_EXT_RETROREFLECTION
  RemapSlot(material.retroreflection_texture, remap);
#endif
}

}  // namespace

GltfTextureInfo evo_engine::MakeGltfTextureInfo(const int32_t texture_index, const int32_t tex_coord,
                                                const glm::mat3x2& uv_transform,
                                                const GltfTextureColorSpace color_space) {
  GltfTextureInfo result;
  result.index = texture_index;
  result.tex_coord = ClampTexCoord(tex_coord);
  if (result.tex_coord < 0) {
    result.index = -1;
  }
  result.color_space = static_cast<int32_t>(color_space);
#if MAT_EXT_TEXTURE_TRANSFORM
  result.uv_transform = uv_transform;
#endif
  return result;
}

uint16_t evo_engine::AppendGltfTextureInfo(GltfMaterialData& material_data, const GltfTextureInfo& texture_info) {
  if (texture_info.index < 0) {
    return 0;
  }
  if (material_data.texture_infos.empty()) {
    material_data.texture_infos.emplace_back();
  }
  const auto index = static_cast<uint16_t>(material_data.texture_infos.size());
  material_data.texture_infos.emplace_back(texture_info);
  return index;
}

void evo_engine::AssignGltfTextureSlot(GltfMaterialData& material_data, uint16_t GltfShadeMaterial::* slot,
                                       const int32_t texture_index, const int32_t tex_coord,
                                       const glm::mat3x2& uv_transform, const GltfTextureColorSpace color_space) {
  material_data.shade_material.*slot =
      AppendGltfTextureInfo(material_data, MakeGltfTextureInfo(texture_index, tex_coord, uv_transform, color_space));
}

GltfMaterialData evo_engine::BuildMaterialGltfData(Material& material) {
  return material.BuildGltfMaterialData();
}

std::string evo_engine::ResolveGltfTextureUri(const YAML::Node& gltf, const int32_t texture_index,
                                              const bool prefer_dds) {
  const auto textures = ChildNode(gltf, "textures");
  if (!textures || texture_index < 0 || texture_index >= static_cast<int32_t>(textures.size())) {
    return {};
  }
  const auto texture = textures[texture_index];
  int32_t image_index = ReadInt(ChildNode(texture, "source"), -1);
  if (prefer_dds) {
    image_index =
        ReadInt(ChildNode(ChildNode(ChildNode(texture, "extensions"), "MSFT_texture_dds"), "source"), image_index);
  }

  const auto images = ChildNode(gltf, "images");
  if (!images || image_index < 0 || image_index >= static_cast<int32_t>(images.size())) {
    return {};
  }
  return ReadString(ChildNode(images[image_index], "uri"));
}

GltfSamplerInfo evo_engine::ReadGltfSamplerInfo(const YAML::Node& gltf, const int32_t texture_index,
                                                const GltfMaterialErrorCallback& report_error) {
  GltfSamplerInfo result;
  const auto textures = ChildNode(gltf, "textures");
  if (!textures || texture_index < 0 || texture_index >= static_cast<int32_t>(textures.size())) {
    ReportMaterialError(report_error, "glTF texture " + std::to_string(texture_index) + " is out of range.");
    return result;
  }
  const int32_t sampler_index = ReadInt(ChildNode(textures[texture_index], "sampler"), -1);
  if (sampler_index < 0) {
    return result;
  }
  const auto samplers = ChildNode(gltf, "samplers");
  if (!samplers || sampler_index >= static_cast<int32_t>(samplers.size())) {
    ReportMaterialError(report_error, "glTF texture " + std::to_string(texture_index) + " references sampler " +
                                          std::to_string(sampler_index) + " outside the sampler array.");
    return result;
  }
  const auto sampler = samplers[sampler_index];
  const auto report_value = [&](const char* field, const int32_t value) {
    ReportMaterialError(report_error, "glTF sampler " + std::to_string(sampler_index) + " uses unsupported " + field +
                                          " value " + std::to_string(value) + "; using the default.");
  };

  const int32_t mag_filter = ReadInt(ChildNode(sampler, "magFilter"), 9729);
  if (mag_filter == 9728) {
    result.mag_filter = VK_FILTER_NEAREST;
  } else if (mag_filter != 9729) {
    report_value("magFilter", mag_filter);
  }

  const int32_t min_filter = ReadInt(ChildNode(sampler, "minFilter"), 9987);
  switch (min_filter) {
    case 9728:
      result.min_filter = VK_FILTER_NEAREST;
      result.mipmap_mode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
      result.max_lod = 0.0f;
      break;
    case 9729:
      result.max_lod = 0.0f;
      break;
    case 9984:
      result.min_filter = VK_FILTER_NEAREST;
      result.mipmap_mode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
      break;
    case 9985:
      result.mipmap_mode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
      break;
    case 9986:
      result.min_filter = VK_FILTER_NEAREST;
      break;
    case 9987:
      break;
    default:
      report_value("minFilter", min_filter);
      break;
  }

  const auto read_address_mode = [&](const char* field) {
    const int32_t value = ReadInt(ChildNode(sampler, field), 10497);
    switch (value) {
      case 33071:
        return VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
      case 33648:
        return VK_SAMPLER_ADDRESS_MODE_MIRRORED_REPEAT;
      case 10497:
        return VK_SAMPLER_ADDRESS_MODE_REPEAT;
      default:
        report_value(field, value);
        return VK_SAMPLER_ADDRESS_MODE_REPEAT;
    }
  };
  result.address_mode_u = read_address_mode("wrapS");
  result.address_mode_v = read_address_mode("wrapT");
  return result;
}

std::vector<GltfMaterialData> evo_engine::BuildGltfMaterialDataFromGltfNode(
    const YAML::Node& gltf, const std::function<int32_t(int32_t texture_index, bool srgb)>& resolve_texture_index,
    const std::function<bool(int32_t texture_index, bool srgb)>& texture_source_needs_y_flip,
    const std::function<bool(int32_t texture_index, bool srgb)>& texture_source_decodes_srgb,
    const GltfMaterialErrorCallback& report_error) {
  std::vector<GltfMaterialData> result;
  const auto materials = ChildNode(gltf, "materials");
  if (!materials || !materials.IsSequence()) {
    return result;
  }

  result.reserve(materials.size());
  for (size_t material_index = 0; material_index < materials.size(); ++material_index) {
    const auto source_material = materials[material_index];
    GltfMaterialData material_data;
    auto& shade_material = material_data.shade_material;

    const auto pbr_metallic_roughness = ChildNode(source_material, "pbrMetallicRoughness");
    shade_material.pbr_base_color_factor =
        ReadVec4(ChildNode(pbr_metallic_roughness, "baseColorFactor"), shade_material.pbr_base_color_factor);
    shade_material.pbr_metallic_factor =
        ReadFloat(ChildNode(pbr_metallic_roughness, "metallicFactor"), shade_material.pbr_metallic_factor);
    shade_material.pbr_roughness_factor =
        ReadFloat(ChildNode(pbr_metallic_roughness, "roughnessFactor"), shade_material.pbr_roughness_factor);

    const auto alpha_mode = ReadString(ChildNode(source_material, "alphaMode"), "OPAQUE");
    shade_material.alpha_mode = alpha_mode == "MASK"    ? static_cast<int32_t>(GltfAlphaMode::Mask)
                                : alpha_mode == "BLEND" ? static_cast<int32_t>(GltfAlphaMode::Blend)
                                                        : static_cast<int32_t>(GltfAlphaMode::Opaque);
    shade_material.alpha_cutoff = ReadFloat(ChildNode(source_material, "alphaCutoff"), shade_material.alpha_cutoff);
    shade_material.double_sided = ReadBool(ChildNode(source_material, "doubleSided"), false) ? 1 : 0;
    shade_material.emissive_factor =
        ReadVec3(ChildNode(source_material, "emissiveFactor"), shade_material.emissive_factor);
    shade_material.normal_texture_scale =
        ReadFloat(ChildNode(ChildNode(source_material, "normalTexture"), "scale"), shade_material.normal_texture_scale);
    shade_material.occlusion_strength = ReadFloat(ChildNode(ChildNode(source_material, "occlusionTexture"), "strength"),
                                                  shade_material.occlusion_strength);

    const auto extensions = ChildNode(source_material, "extensions");
    const auto raw_unlit = ChildNode(extensions, "KHR_materials_unlit");
    const auto raw_specular_glossiness = ChildNode(extensions, "KHR_materials_pbrSpecularGlossiness");
    const std::string primary_model = raw_unlit && raw_unlit.IsMap() ? "KHR_materials_unlit"
                                      : raw_specular_glossiness && raw_specular_glossiness.IsMap()
                                          ? "KHR_materials_pbrSpecularGlossiness"
                                          : "";
#if MAT_EXT_TRANSMISSION
    const auto transmission = CompatibleMaterialExtension(extensions, "KHR_materials_transmission", primary_model,
                                                          material_index, report_error);
    shade_material.transmission_factor =
        ReadFloat(ChildNode(transmission, "transmissionFactor"), shade_material.transmission_factor);
#endif
#if MAT_EXT_IOR
    const auto ior_extension =
        CompatibleMaterialExtension(extensions, "KHR_materials_ior", primary_model, material_index, report_error);
    const float ior = ReadFloat(ChildNode(ior_extension, "ior"), shade_material.ior);
    shade_material.ior = ior == 0.0f ? 0.0f : std::max(ior, 1.0f);
#endif
#if MAT_EXT_VOLUME
    const auto volume =
        CompatibleMaterialExtension(extensions, "KHR_materials_volume", primary_model, material_index, report_error);
    shade_material.thickness_factor = ReadFloat(ChildNode(volume, "thicknessFactor"), shade_material.thickness_factor);
    shade_material.attenuation_distance =
        ReadFloat(ChildNode(volume, "attenuationDistance"), shade_material.attenuation_distance);
    shade_material.attenuation_color =
        ReadVec3(ChildNode(volume, "attenuationColor"), shade_material.attenuation_color);
#endif
#if MAT_EXT_CLEARCOAT
    const auto clearcoat =
        CompatibleMaterialExtension(extensions, "KHR_materials_clearcoat", primary_model, material_index, report_error);
    shade_material.clearcoat_factor =
        ReadFloat(ChildNode(clearcoat, "clearcoatFactor"), shade_material.clearcoat_factor);
    shade_material.clearcoat_roughness =
        ReadFloat(ChildNode(clearcoat, "clearcoatRoughnessFactor"), shade_material.clearcoat_roughness);
    shade_material.clearcoat_normal_texture_scale =
        ReadFloat(ChildNode(ChildNode(clearcoat, "clearcoatNormalTexture"), "scale"),
                  shade_material.clearcoat_normal_texture_scale);
#endif
#if MAT_EXT_SPECULAR
    const auto specular =
        CompatibleMaterialExtension(extensions, "KHR_materials_specular", primary_model, material_index, report_error);
    shade_material.specular_factor =
        ClampFloat(ReadFloat(ChildNode(specular, "specularFactor"), shade_material.specular_factor), 0.0f, 1.0f);
    shade_material.specular_color_factor = glm::max(
        ReadVec3(ChildNode(specular, "specularColorFactor"), shade_material.specular_color_factor), glm::vec3(0.0f));
#endif
    const float emissive_strength =
        ReadFloat(ChildNode(ChildNode(extensions, "KHR_materials_emissive_strength"), "emissiveStrength"), 1.0f);
    shade_material.emissive_factor *= emissive_strength;
#if MAT_EXT_UNLIT
    const auto unlit = raw_unlit;
    shade_material.unlit = unlit && unlit.IsMap() ? 1 : 0;
#endif
#if MAT_EXT_SHEEN
    const auto sheen =
        CompatibleMaterialExtension(extensions, "KHR_materials_sheen", primary_model, material_index, report_error);
    shade_material.sheen_color_factor =
        ReadVec3(ChildNode(sheen, "sheenColorFactor"), shade_material.sheen_color_factor);
    shade_material.sheen_roughness_factor =
        ReadFloat(ChildNode(sheen, "sheenRoughnessFactor"), shade_material.sheen_roughness_factor);
#endif
#if MAT_EXT_IRIDESCENCE
    const auto iridescence = CompatibleMaterialExtension(extensions, "KHR_materials_iridescence", primary_model,
                                                         material_index, report_error);
    shade_material.iridescence_factor = ClampFloat(
        ReadFloat(ChildNode(iridescence, "iridescenceFactor"), shade_material.iridescence_factor), 0.0f, 1.0f);
    shade_material.iridescence_ior =
        std::max(ReadFloat(ChildNode(iridescence, "iridescenceIor"), shade_material.iridescence_ior), 1.0f);
    shade_material.iridescence_thickness_minimum = std::max(
        ReadFloat(ChildNode(iridescence, "iridescenceThicknessMinimum"), shade_material.iridescence_thickness_minimum),
        0.0f);
    shade_material.iridescence_thickness_maximum = std::max(
        ReadFloat(ChildNode(iridescence, "iridescenceThicknessMaximum"), shade_material.iridescence_thickness_maximum),
        0.0f);
#endif
#if MAT_EXT_ANISOTROPY
    const auto anisotropy = CompatibleMaterialExtension(extensions, "KHR_materials_anisotropy", primary_model,
                                                        material_index, report_error);
    shade_material.anisotropy_strength = ClampFloat(
        ReadFloat(ChildNode(anisotropy, "anisotropyStrength"), shade_material.anisotropy_strength), 0.0f, 1.0f);
    const float anisotropy_rotation = ReadFloat(ChildNode(anisotropy, "anisotropyRotation"), 0.0f);
    shade_material.anisotropy_rotation = glm::vec2(std::cos(anisotropy_rotation), std::sin(anisotropy_rotation));
#endif
#if MAT_EXT_DISPERSION
    const auto dispersion = CompatibleMaterialExtension(extensions, "KHR_materials_dispersion", primary_model,
                                                        material_index, report_error);
    shade_material.dispersion =
        std::max(ReadFloat(ChildNode(dispersion, "dispersion"), shade_material.dispersion), 0.0f);
#endif
#if MAT_EXT_RETROREFLECTION
    const auto khr_retroreflection = CompatibleMaterialExtension(extensions, "KHR_materials_retroreflection",
                                                                 primary_model, material_index, report_error);
    const auto ext_retroreflection = CompatibleMaterialExtension(extensions, "EXT_materials_retroreflection",
                                                                 primary_model, material_index, report_error);
    const auto retroreflection =
        khr_retroreflection && khr_retroreflection.IsMap() ? khr_retroreflection : ext_retroreflection;
    shade_material.retroreflection_factor = ClampFloat(
        ReadFloat(ChildNode(retroreflection, "retroreflectionFactor"), shade_material.retroreflection_factor), 0.0f,
        1.0f);
#endif
#if MAT_EXT_DIFFUSE_TRANSMISSION
    const auto diffuse_transmission = CompatibleMaterialExtension(extensions, "KHR_materials_diffuse_transmission",
                                                                  primary_model, material_index, report_error);
    shade_material.diffuse_transmission_factor = ReadFloat(ChildNode(diffuse_transmission, "diffuseTransmissionFactor"),
                                                           shade_material.diffuse_transmission_factor);
    shade_material.diffuse_transmission_color = ReadVec3(ChildNode(diffuse_transmission, "diffuseTransmissionColor"),
                                                         shade_material.diffuse_transmission_color);
    shade_material.diffuse_transmission_color = ReadVec3(
        ChildNode(diffuse_transmission, "diffuseTransmissionColorFactor"), shade_material.diffuse_transmission_color);
#endif
#if MAT_EXT_VOLUME_SCATTER
    const auto volume_scatter = CompatibleMaterialExtension(extensions, "KHR_materials_volume_scatter", primary_model,
                                                            material_index, report_error);
    shade_material.multiscatter_color_factor =
        ReadVec3(ChildNode(volume_scatter, "multiscatterColorFactor"), shade_material.multiscatter_color_factor);
    shade_material.multiscatter_color_factor =
        ReadVec3(ChildNode(volume_scatter, "multiscatterColor"), shade_material.multiscatter_color_factor);
    shade_material.scatter_anisotropy = ClampFloat(
        ReadFloat(ChildNode(volume_scatter, "scatterAnisotropy"), shade_material.scatter_anisotropy), -0.999f, 0.999f);
#endif
    const auto specular_glossiness = CompatibleMaterialExtension(extensions, "KHR_materials_pbrSpecularGlossiness",
                                                                 primary_model, material_index, report_error);
    if (specular_glossiness && specular_glossiness.IsMap()) {
      const auto converted = gltf_import::ConvertSpecularGlossiness(
          ReadVec4(ChildNode(specular_glossiness, "diffuseFactor"), glm::vec4(1.0f)),
          ReadVec3(ChildNode(specular_glossiness, "specularFactor"), glm::vec3(1.0f)),
          ReadFloat(ChildNode(specular_glossiness, "glossinessFactor"), 1.0f));
      shade_material.pbr_base_color_factor = converted.base_color;
      shade_material.pbr_metallic_factor = converted.metallic;
      shade_material.pbr_roughness_factor = converted.roughness;
      if (ChildNode(specular_glossiness, "diffuseTexture") ||
          ChildNode(specular_glossiness, "specularGlossinessTexture")) {
        ReportMaterialError(report_error, "glTF material " + std::to_string(material_index) +
                                              " converts KHR_materials_pbrSpecularGlossiness factors only because "
                                              "this material-data builder has no source pixel access.");
      }
    }

    const auto read_texture_info = [&](const YAML::Node& node) {
      return ReadTextureNodeInfo(node, material_index, report_error);
    };
    AssignTextureNode(material_data, &GltfShadeMaterial::emissive_texture,
                      read_texture_info(ChildNode(source_material, "emissiveTexture")), resolve_texture_index,
                      texture_source_needs_y_flip, texture_source_decodes_srgb, true);
    AssignTextureNode(material_data, &GltfShadeMaterial::normal_texture,
                      read_texture_info(ChildNode(source_material, "normalTexture")), resolve_texture_index,
                      texture_source_needs_y_flip, texture_source_decodes_srgb, false);
    if (!specular_glossiness || !specular_glossiness.IsMap()) {
      AssignTextureNode(material_data, &GltfShadeMaterial::pbr_base_color_texture,
                        read_texture_info(ChildNode(pbr_metallic_roughness, "baseColorTexture")), resolve_texture_index,
                        texture_source_needs_y_flip, texture_source_decodes_srgb, true);
      AssignTextureNode(material_data, &GltfShadeMaterial::pbr_metallic_roughness_texture,
                        read_texture_info(ChildNode(pbr_metallic_roughness, "metallicRoughnessTexture")),
                        resolve_texture_index, texture_source_needs_y_flip, texture_source_decodes_srgb, false);
    }
    AssignTextureNode(material_data, &GltfShadeMaterial::occlusion_texture,
                      read_texture_info(ChildNode(source_material, "occlusionTexture")), resolve_texture_index,
                      texture_source_needs_y_flip, texture_source_decodes_srgb, false);
#if MAT_EXT_TRANSMISSION
    AssignTextureNode(material_data, &GltfShadeMaterial::transmission_texture,
                      read_texture_info(ChildNode(transmission, "transmissionTexture")), resolve_texture_index,
                      texture_source_needs_y_flip, texture_source_decodes_srgb, false);
#endif
#if MAT_EXT_VOLUME
    AssignTextureNode(material_data, &GltfShadeMaterial::thickness_texture,
                      read_texture_info(ChildNode(volume, "thicknessTexture")), resolve_texture_index,
                      texture_source_needs_y_flip, texture_source_decodes_srgb, false);
#endif
#if MAT_EXT_CLEARCOAT
    AssignTextureNode(material_data, &GltfShadeMaterial::clearcoat_roughness_texture,
                      read_texture_info(ChildNode(clearcoat, "clearcoatRoughnessTexture")), resolve_texture_index,
                      texture_source_needs_y_flip, texture_source_decodes_srgb, false);
    AssignTextureNode(material_data, &GltfShadeMaterial::clearcoat_texture,
                      read_texture_info(ChildNode(clearcoat, "clearcoatTexture")), resolve_texture_index,
                      texture_source_needs_y_flip, texture_source_decodes_srgb, false);
    AssignTextureNode(material_data, &GltfShadeMaterial::clearcoat_normal_texture,
                      read_texture_info(ChildNode(clearcoat, "clearcoatNormalTexture")), resolve_texture_index,
                      texture_source_needs_y_flip, texture_source_decodes_srgb, false);
#endif
#if MAT_EXT_SPECULAR
    AssignTextureNode(material_data, &GltfShadeMaterial::specular_texture,
                      read_texture_info(ChildNode(specular, "specularTexture")), resolve_texture_index,
                      texture_source_needs_y_flip, texture_source_decodes_srgb, false);
    AssignTextureNode(material_data, &GltfShadeMaterial::specular_color_texture,
                      read_texture_info(ChildNode(specular, "specularColorTexture")), resolve_texture_index,
                      texture_source_needs_y_flip, texture_source_decodes_srgb, true);
#endif
#if MAT_EXT_IRIDESCENCE
    AssignTextureNode(material_data, &GltfShadeMaterial::iridescence_texture,
                      read_texture_info(ChildNode(iridescence, "iridescenceTexture")), resolve_texture_index,
                      texture_source_needs_y_flip, texture_source_decodes_srgb, false);
    AssignTextureNode(material_data, &GltfShadeMaterial::iridescence_thickness_texture,
                      read_texture_info(ChildNode(iridescence, "iridescenceThicknessTexture")), resolve_texture_index,
                      texture_source_needs_y_flip, texture_source_decodes_srgb, false);
#endif
#if MAT_EXT_ANISOTROPY
    AssignTextureNode(material_data, &GltfShadeMaterial::anisotropy_texture,
                      read_texture_info(ChildNode(anisotropy, "anisotropyTexture")), resolve_texture_index,
                      texture_source_needs_y_flip, texture_source_decodes_srgb, false);
#endif
#if MAT_EXT_SHEEN
    AssignTextureNode(material_data, &GltfShadeMaterial::sheen_color_texture,
                      read_texture_info(ChildNode(sheen, "sheenColorTexture")), resolve_texture_index,
                      texture_source_needs_y_flip, texture_source_decodes_srgb, true);
    AssignTextureNode(material_data, &GltfShadeMaterial::sheen_roughness_texture,
                      read_texture_info(ChildNode(sheen, "sheenRoughnessTexture")), resolve_texture_index,
                      texture_source_needs_y_flip, texture_source_decodes_srgb, false);
#endif
#if MAT_EXT_DIFFUSE_TRANSMISSION
    AssignTextureNode(material_data, &GltfShadeMaterial::diffuse_transmission_texture,
                      read_texture_info(ChildNode(diffuse_transmission, "diffuseTransmissionTexture")),
                      resolve_texture_index, texture_source_needs_y_flip, texture_source_decodes_srgb, false);
    AssignTextureNode(material_data, &GltfShadeMaterial::diffuse_transmission_color_texture,
                      read_texture_info(ChildNode(diffuse_transmission, "diffuseTransmissionColorTexture")),
                      resolve_texture_index, texture_source_needs_y_flip, texture_source_decodes_srgb, true);
#endif
#if MAT_EXT_RETROREFLECTION
    AssignTextureNode(material_data, &GltfShadeMaterial::retroreflection_texture,
                      read_texture_info(ChildNode(retroreflection, "retroreflectionTexture")), resolve_texture_index,
                      texture_source_needs_y_flip, texture_source_decodes_srgb, false);
#endif

    result.emplace_back(std::move(material_data));
  }
  return result;
}

std::vector<GltfMaterialData> evo_engine::BuildGltfMaterialDataFromGltfNode(
    const YAML::Node& gltf, const std::function<int32_t(int32_t texture_index)>& resolve_texture_index,
    const std::function<bool(int32_t texture_index)>& texture_source_needs_y_flip,
    const std::function<bool(int32_t texture_index)>& texture_source_decodes_srgb,
    const GltfMaterialErrorCallback& report_error) {
  return BuildGltfMaterialDataFromGltfNode(
      gltf,
      [&](const int32_t texture_index, const bool) {
        return resolve_texture_index(texture_index);
      },
      [&](const int32_t texture_index, const bool) {
        return texture_source_needs_y_flip && texture_source_needs_y_flip(texture_index);
      },
      [&](const int32_t texture_index, const bool) {
        return texture_source_decodes_srgb && texture_source_decodes_srgb(texture_index);
      },
      report_error);
}

void GltfMaterialCache::Clear() {
  material_data_.clear();
  shade_materials_.clear();
  texture_infos_.clear();
  texture_infos_.emplace_back();
}

uint32_t GltfMaterialCache::Append(const GltfMaterialData& material_data) {
  const auto material_index = static_cast<uint32_t>(material_data_.size());
  material_data_.emplace_back(material_data);
  AppendFlattened(material_data);
  return material_index;
}

void GltfMaterialCache::AppendFlattened(const GltfMaterialData& material_data) {
  if (texture_infos_.empty()) {
    texture_infos_.emplace_back();
  }

  std::vector<uint16_t> remap(material_data.texture_infos.size(), 0);
  for (size_t i = 1; i < material_data.texture_infos.size(); ++i) {
    const auto& texture_info = material_data.texture_infos[i];
    if (texture_info.index < 0) {
      continue;
    }
    remap[i] = static_cast<uint16_t>(texture_infos_.size());
    texture_infos_.emplace_back(texture_info);
  }

  auto shade_material = material_data.shade_material;
  RemapTextureSlots(shade_material, remap);
  shade_materials_.emplace_back(shade_material);
}

uint32_t GltfMaterialCache::Append(Material& material) {
  return Append(BuildMaterialGltfData(material));
}

void GltfMaterialCache::Update(const uint32_t material_index, const GltfMaterialData& material_data) {
  if (material_index >= material_data_.size()) {
    throw std::out_of_range("glTF material cache update index is out of range.");
  }
  material_data_[material_index] = material_data;
  Rebuild();
}

void GltfMaterialCache::Rebuild() {
  shade_materials_.clear();
  texture_infos_.clear();
  texture_infos_.emplace_back();
  for (const auto& material_data : material_data_) {
    AppendFlattened(material_data);
  }
}

const std::vector<GltfShadeMaterial>& GltfMaterialCache::GetShadeMaterials() const {
  return shade_materials_;
}

const std::vector<GltfTextureInfo>& GltfMaterialCache::GetTextureInfos() const {
  return texture_infos_;
}
