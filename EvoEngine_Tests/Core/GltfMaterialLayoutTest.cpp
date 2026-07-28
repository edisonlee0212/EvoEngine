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

TEST(GltfMaterialLayout, ExtendedVerticesPreserveLegacyPrefixAndAppendFourUvSets) {
  using evo_engine::SkinnedVertex;
  using evo_engine::Vertex;

  EXPECT_EQ(sizeof(Vertex), 112);
  EXPECT_EQ(offsetof(Vertex, tex_coord_1), 80);
  EXPECT_EQ(offsetof(Vertex, tex_coord_2), 88);
  EXPECT_EQ(offsetof(Vertex, tex_coord_3), 96);
  EXPECT_EQ(sizeof(SkinnedVertex), 176);
  EXPECT_EQ(offsetof(SkinnedVertex, tex_coord_1), 144);
  EXPECT_EQ(offsetof(SkinnedVertex, tex_coord_2), 152);
  EXPECT_EQ(offsetof(SkinnedVertex, tex_coord_3), 160);
}

TEST(GltfMaterialLayout, HostShadeMaterialMatchesReferenceBaseAnchors) {
  using evo_engine::GltfShadeMaterial;

  EXPECT_TRUE(std::is_standard_layout_v<GltfShadeMaterial>);
  EXPECT_GE(alignof(GltfShadeMaterial), 8);
  EXPECT_EQ(sizeof(GltfShadeMaterial), 288);
  EXPECT_EQ(offsetof(GltfShadeMaterial, pbr_base_color_factor), 0);
  EXPECT_EQ(offsetof(GltfShadeMaterial, pbr_roughness_factor), 32);
  EXPECT_EQ(offsetof(GltfShadeMaterial, alpha_mode), 40);
  EXPECT_EQ(offsetof(GltfShadeMaterial, occlusion_strength), 48);
  EXPECT_EQ(offsetof(GltfShadeMaterial, double_sided), 52);
  EXPECT_EQ(offsetof(GltfShadeMaterial, iridescence_factor), 112);
  EXPECT_EQ(offsetof(GltfShadeMaterial, anisotropy_rotation), 128);
  EXPECT_EQ(offsetof(GltfShadeMaterial, anisotropy_strength), 148);
  EXPECT_EQ(offsetof(GltfShadeMaterial, dispersion), 156);
  EXPECT_EQ(offsetof(GltfShadeMaterial, retroreflection_factor), 212);
  EXPECT_EQ(offsetof(GltfShadeMaterial, pbr_base_color_texture), 232);
  EXPECT_EQ(offsetof(GltfShadeMaterial, iridescence_texture), 256);
  EXPECT_EQ(offsetof(GltfShadeMaterial, iridescence_thickness_texture), 258);
  EXPECT_EQ(offsetof(GltfShadeMaterial, anisotropy_texture), 260);
  EXPECT_EQ(offsetof(GltfShadeMaterial, retroreflection_texture), 274);
  EXPECT_EQ(offsetof(GltfShadeMaterial, padding0), 276);
  EXPECT_EQ(offsetof(GltfShadeMaterial, clearcoat_normal_texture_scale), 280);
  EXPECT_EQ(offsetof(GltfShadeMaterial, padding1), 284);
}

TEST(GltfMaterialLayout, ShaderIncludeKeepsMaterialLayoutGatesSeparateFromBehaviorGates) {
  const auto shader_source = ReadText(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" /
                                      "DefaultResources" / "Shaders" / "Includes" / "GltfMaterial.slangh");

  EXPECT_NE(shader_source.find("MAT_EXT_SPECULAR_GLOSSINESS"), std::string::npos);
  EXPECT_NE(shader_source.find("MAT_EXT_TEXTURE_TRANSFORM"), std::string::npos);
  EXPECT_NE(shader_source.find("MAT_EXT_RETROREFLECTION"), std::string::npos);
  EXPECT_NE(shader_source.find("struct GltfTextureInfo"), std::string::npos);
  EXPECT_NE(shader_source.find("int color_space"), std::string::npos);
  EXPECT_NE(shader_source.find("int padding"), std::string::npos);
  EXPECT_NE(shader_source.find("struct GltfShadeMaterial"), std::string::npos);
  EXPECT_NE(shader_source.find("uint16_t pbr_base_color_texture"), std::string::npos);
  EXPECT_NE(shader_source.find("float retroreflection_factor"), std::string::npos);
  EXPECT_NE(shader_source.find("float clearcoat_normal_texture_scale"), std::string::npos);
  EXPECT_NE(shader_source.find("uint16_t retroreflection_texture"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_GLTF_USE_TRANSMISSION"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_GLTF_USE_TEXTURE_TRANSFORM"), std::string::npos);
}
