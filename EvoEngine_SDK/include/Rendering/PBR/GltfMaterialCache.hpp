#pragma once

#include "GltfMaterial.hpp"

#include <functional>
#include <string>

#include <yaml-cpp/yaml.h>

namespace evo_engine {

class Material;

GltfTextureInfo MakeGltfTextureInfo(int32_t texture_index, int32_t tex_coord = 0,
                                    const glm::mat3x2& uv_transform = glm::mat3x2(1.0f));
uint16_t AppendGltfTextureInfo(GltfMaterialData& material_data, const GltfTextureInfo& texture_info);
void AssignGltfTextureSlot(GltfMaterialData& material_data, uint16_t GltfShadeMaterial::* slot, int32_t texture_index,
                           int32_t tex_coord = 0, const glm::mat3x2& uv_transform = glm::mat3x2(1.0f));

GltfMaterialData BuildMaterialGltfData(Material& material);

std::string ResolveGltfTextureUri(const YAML::Node& gltf, int32_t texture_index, bool prefer_dds = true);
std::vector<GltfMaterialData> BuildGltfMaterialDataFromGltfNode(
    const YAML::Node& gltf, const std::function<int32_t(int32_t texture_index)>& resolve_texture_index,
    const std::function<bool(int32_t texture_index)>& texture_source_needs_y_flip = {});

class GltfMaterialCache {
 public:
  void Clear();
  uint32_t Append(const GltfMaterialData& material_data);
  uint32_t Append(Material& material);

  [[nodiscard]] const std::vector<GltfShadeMaterial>& GetShadeMaterials() const;
  [[nodiscard]] const std::vector<GltfTextureInfo>& GetTextureInfos() const;

 private:
  std::vector<GltfShadeMaterial> shade_materials_;
  std::vector<GltfTextureInfo> texture_infos_{GltfTextureInfo{}};
};

}  // namespace evo_engine
