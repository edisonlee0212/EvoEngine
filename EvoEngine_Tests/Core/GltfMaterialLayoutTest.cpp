#include "GltfMaterial.hpp"
#include "Vertex.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <type_traits>

namespace {
std::string ReadText(const std::filesystem::path& path) {
  std::ifstream stream(path);
  return std::string(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}
}  // namespace

TEST(GltfMaterialLayout, HostTextureInfoMatchesReferenceAnchors) {
  using evo_engine::GltfTextureInfo;

  EXPECT_TRUE(std::is_standard_layout_v<GltfTextureInfo>);
  EXPECT_EQ(sizeof(GltfTextureInfo), 40);
  EXPECT_EQ(offsetof(GltfTextureInfo, uv_transform), 0);
  EXPECT_EQ(offsetof(GltfTextureInfo, index), 24);
  EXPECT_EQ(offsetof(GltfTextureInfo, tex_coord), 28);
  EXPECT_EQ(offsetof(GltfTextureInfo, color_space), 32);
  EXPECT_EQ(offsetof(GltfTextureInfo, padding), 36);
}

TEST(GltfMaterialLayout, ExtendedVerticesPreserveLegacyPrefixAndAppendSecondaryUv) {
  using evo_engine::SkinnedVertex;
  using evo_engine::Vertex;

  EXPECT_EQ(sizeof(Vertex), 96);
  EXPECT_EQ(offsetof(Vertex, tex_coord_1), 80);
  EXPECT_EQ(sizeof(SkinnedVertex), 160);
  EXPECT_EQ(offsetof(SkinnedVertex, tex_coord_1), 144);
}

TEST(GltfMaterialLayout, HostShadeMaterialMatchesReferenceBaseAnchors) {
  using evo_engine::GltfShadeMaterial;

  EXPECT_TRUE(std::is_standard_layout_v<GltfShadeMaterial>);
  EXPECT_GE(alignof(GltfShadeMaterial), 8);
  EXPECT_EQ(sizeof(GltfShadeMaterial) % 8, 0);
  EXPECT_EQ(offsetof(GltfShadeMaterial, pbr_base_color_factor), 0);
  EXPECT_EQ(offsetof(GltfShadeMaterial, pbr_roughness_factor), 32);
  EXPECT_EQ(offsetof(GltfShadeMaterial, alpha_mode), 40);
  EXPECT_EQ(offsetof(GltfShadeMaterial, occlusion_strength), 48);
  EXPECT_EQ(offsetof(GltfShadeMaterial, double_sided), 52);
  EXPECT_LT(offsetof(GltfShadeMaterial, pbr_base_color_texture), offsetof(GltfShadeMaterial, pad));
}

TEST(GltfMaterialLayout, ShaderIncludeKeepsMaterialLayoutGatesSeparateFromBehaviorGates) {
  const auto shader_source = ReadText(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" /
                                      "DefaultResources" / "Shaders" / "Includes" / "GltfMaterial.glsl");

  EXPECT_NE(shader_source.find("MAT_EXT_SPECULAR_GLOSSINESS"), std::string::npos);
  EXPECT_NE(shader_source.find("MAT_EXT_TEXTURE_TRANSFORM"), std::string::npos);
  EXPECT_NE(shader_source.find("struct GltfTextureInfo"), std::string::npos);
  EXPECT_NE(shader_source.find("int color_space"), std::string::npos);
  EXPECT_NE(shader_source.find("int padding"), std::string::npos);
  EXPECT_NE(shader_source.find("struct GltfShadeMaterial"), std::string::npos);
  EXPECT_NE(shader_source.find("uint16_t pbr_base_color_texture"), std::string::npos);
  EXPECT_EQ(shader_source.find("GLTF_USE_"), std::string::npos);
}
