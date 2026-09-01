#include "GBuffer.hpp"
#include "GltfMaterial.hpp"
#include "Vertex.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
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

TEST(GltfMaterialLayout, VerticesExposeOnlyUv0AndUv1) {
  using evo_engine::SkinnedVertex;
  using evo_engine::Vertex;

  EXPECT_EQ(sizeof(Vertex), 96);
  EXPECT_EQ(offsetof(Vertex, tex_coord_1), 80);
  EXPECT_EQ(offsetof(Vertex, padding), 88);
  EXPECT_EQ(sizeof(SkinnedVertex), 160);
  EXPECT_EQ(offsetof(SkinnedVertex, tex_coord_1), 144);
  EXPECT_EQ(offsetof(SkinnedVertex, padding), 152);
}

TEST(GltfMaterialLayout, HostShadeMaterialMatchesReferenceBaseAnchors) {
  using evo_engine::GltfShadeMaterial;

  EXPECT_TRUE(std::is_standard_layout_v<GltfShadeMaterial>);
  EXPECT_GE(alignof(GltfShadeMaterial), 8);
  EXPECT_EQ(sizeof(GltfShadeMaterial), 248);
  EXPECT_EQ(offsetof(GltfShadeMaterial, pbr_base_color_factor), 0);
  EXPECT_EQ(offsetof(GltfShadeMaterial, pbr_roughness_factor), 32);
  EXPECT_EQ(offsetof(GltfShadeMaterial, alpha_mode), 40);
  EXPECT_EQ(offsetof(GltfShadeMaterial, occlusion_strength), 48);
  EXPECT_EQ(offsetof(GltfShadeMaterial, double_sided), 52);
  EXPECT_EQ(offsetof(GltfShadeMaterial, iridescence_factor), 112);
  EXPECT_EQ(offsetof(GltfShadeMaterial, anisotropy_rotation), 128);
  EXPECT_EQ(offsetof(GltfShadeMaterial, anisotropy_strength), 148);
  EXPECT_EQ(offsetof(GltfShadeMaterial, dispersion), 156);
  EXPECT_EQ(offsetof(GltfShadeMaterial, retroreflection_factor), 176);
  EXPECT_EQ(offsetof(GltfShadeMaterial, pbr_base_color_texture), 196);
  EXPECT_EQ(offsetof(GltfShadeMaterial, iridescence_texture), 220);
  EXPECT_EQ(offsetof(GltfShadeMaterial, iridescence_thickness_texture), 222);
  EXPECT_EQ(offsetof(GltfShadeMaterial, anisotropy_texture), 224);
  EXPECT_EQ(offsetof(GltfShadeMaterial, retroreflection_texture), 234);
  EXPECT_EQ(offsetof(GltfShadeMaterial, nested_priority), 236);
  EXPECT_EQ(offsetof(GltfShadeMaterial, clearcoat_normal_texture_scale), 240);
  EXPECT_EQ(offsetof(GltfShadeMaterial, padding1), 244);
}

TEST(GltfMaterialLayout, NativeShaderModuleUsesTheFixedFullExtensionAbi) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto shader_source = ReadText(shader_root / "Modules" / "EvoEngine" / "GltfMaterial.slang");

  EXPECT_EQ(shader_source.find('#'), std::string::npos);
  EXPECT_EQ(shader_source.find("MAT_EXT_"), std::string::npos);
  EXPECT_EQ(shader_source.find("EE_GLTF_USE_"), std::string::npos);
  EXPECT_NE(shader_source.find("struct GltfTextureInfo"), std::string::npos);
  EXPECT_NE(shader_source.find("float3x2 uv_transform"), std::string::npos);
  EXPECT_NE(shader_source.find("return mul(uv, texture_info.uv_transform)"), std::string::npos);
  EXPECT_NE(shader_source.find("int color_space"), std::string::npos);
  EXPECT_NE(shader_source.find("int padding"), std::string::npos);
  EXPECT_NE(shader_source.find("struct GltfShadeMaterial"), std::string::npos);
  EXPECT_NE(shader_source.find("uint16_t pbr_base_color_texture"), std::string::npos);
  EXPECT_NE(shader_source.find("float retroreflection_factor"), std::string::npos);
  EXPECT_NE(shader_source.find("float clearcoat_normal_texture_scale"), std::string::npos);
  EXPECT_NE(shader_source.find("uint16_t retroreflection_texture"), std::string::npos);
  EXPECT_NE(shader_source.find("uint nested_priority"), std::string::npos);
  EXPECT_EQ(shader_source.find("SPECULAR_GLOSSINESS"), std::string::npos);
  EXPECT_EQ(shader_source.find("pbr_model"), std::string::npos);
  EXPECT_NE(shader_source.find("[[vk::binding(11, 0)]]"), std::string::npos);
  EXPECT_NE(shader_source.find("StructuredBuffer<GltfShadeMaterial, ScalarDataLayout>"), std::string::npos);
  EXPECT_NE(shader_source.find("[[vk::binding(12, 0)]]"), std::string::npos);
  EXPECT_NE(shader_source.find("StructuredBuffer<GltfTextureInfo, ScalarDataLayout>"), std::string::npos);
}

TEST(GltfMaterialLayout, FirstPartyRuntimeContainsNoSpecularGlossinessRepresentation) {
  const auto root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR);
  const std::array search_roots = {root / "EvoEngine_SDK" / "include", root / "EvoEngine_SDK" / "src",
                                   root / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders",
                                   root / "EvoEngine_App" / "src"};
  const std::array allowed_import_files = {std::string("GltfMaterialCache.cpp"), std::string("Prefab.cpp"),
                                           std::string("Application.cpp")};
  const std::array forbidden = {std::string("GltfPbrModel"),
                                std::string("MAT_EXT_SPECULAR_GLOSSINESS"),
                                std::string("EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS"),
                                std::string("EE_GLTF_SCENE_FEATURE_SPECULAR_GLOSSINESS"),
                                std::string("pbr_diffuse_factor"),
                                std::string("pbr_specular_factor"),
                                std::string("pbr_diffuse_texture"),
                                std::string("pbr_specular_glossiness_texture")};

  for (const auto& search_root : search_roots) {
    for (const auto& entry : std::filesystem::recursive_directory_iterator(search_root)) {
      if (!entry.is_regular_file() || std::find(allowed_import_files.begin(), allowed_import_files.end(),
                                                entry.path().filename().string()) != allowed_import_files.end()) {
        continue;
      }
      const auto extension = entry.path().extension().string();
      if (extension != ".cpp" && extension != ".hpp" && extension != ".slang" && extension != ".slangh") {
        continue;
      }
      const auto source = ReadText(entry.path());
      for (const auto& symbol : forbidden) {
        EXPECT_EQ(source.find(symbol), std::string::npos) << entry.path().string() << ": " << symbol;
      }
    }
  }
}
