#include "EvoEngine_SDK_PCH.hpp"
#include "GltfSceneFeatures.hpp"

#include <gtest/gtest.h>

#include <algorithm>

using namespace evo_engine;

namespace {
uint32_t Bit(const GltfSceneFeature feature) {
  return static_cast<uint32_t>(feature);
}

std::vector<GltfTextureInfo> TextureInfos() {
  std::vector<GltfTextureInfo> result(2);
  result[1].index = 7;
  return result;
}
}  // namespace

TEST(GltfSceneFeatures, DefaultMaterialUsesSparseVariant) {
  EXPECT_EQ(DetectGltfSceneFeatures({GltfShadeMaterial{}}, {GltfTextureInfo{}}), 0u);
}

TEST(GltfSceneFeatures, DetectsEveryBehaviorFeature) {
  const auto detect = [](const GltfShadeMaterial& material) {
    return DetectGltfSceneFeatures({material}, TextureInfos());
  };
  GltfShadeMaterial material;
  material.transmission_factor = 0.25f;
  EXPECT_NE(detect(material) & Bit(GltfSceneFeature::Transmission), 0u);
  material = {};
  material.thickness_factor = 0.25f;
  EXPECT_EQ(detect(material) & (Bit(GltfSceneFeature::Transmission) | Bit(GltfSceneFeature::Volume)),
            Bit(GltfSceneFeature::Transmission) | Bit(GltfSceneFeature::Volume));
  material = {};
  material.multiscatter_color_factor = glm::vec3(0.25f);
  EXPECT_EQ(detect(material) & 7u, 7u);
  material = {};
  material.clearcoat_texture = 1;
  EXPECT_NE(detect(material) & Bit(GltfSceneFeature::Clearcoat), 0u);
  material = {};
  material.iridescence_factor = 0.5f;
  EXPECT_NE(detect(material) & Bit(GltfSceneFeature::Iridescence), 0u);
  material = {};
  material.anisotropy_strength = 0.5f;
  EXPECT_NE(detect(material) & Bit(GltfSceneFeature::Anisotropy), 0u);
  material = {};
  material.sheen_color_factor = glm::vec3(0.25f);
  EXPECT_NE(detect(material) & Bit(GltfSceneFeature::Sheen), 0u);
  material = {};
  material.dispersion = 0.25f;
  EXPECT_NE(detect(material) & Bit(GltfSceneFeature::Dispersion), 0u);
  material = {};
  material.diffuse_transmission_texture = 1;
  EXPECT_NE(detect(material) & Bit(GltfSceneFeature::DiffuseTransmission), 0u);
  material = {};
  material.retroreflection_factor = 0.25f;
  EXPECT_NE(detect(material) & Bit(GltfSceneFeature::Retroreflection), 0u);
  material = {};
  material.unlit = 1;
  EXPECT_NE(detect(material) & Bit(GltfSceneFeature::Unlit), 0u);
  material = {};
  material.specular_factor = 0.5f;
  EXPECT_NE(detect(material) & Bit(GltfSceneFeature::Specular), 0u);
  material = {};
  material.ior = 0.0f;
  EXPECT_NE(detect(material) & Bit(GltfSceneFeature::Ior), 0u);
  material = {};
  material.pbr_model = static_cast<int32_t>(GltfPbrModel::SpecularGlossiness);
  EXPECT_NE(detect(material) & Bit(GltfSceneFeature::SpecularGlossiness), 0u);
  material = {};
  material.pbr_base_color_texture = 1;
  auto texture_infos = TextureInfos();
  texture_infos[1].uv_transform[2] = glm::vec2(0.25f, 0.5f);
  EXPECT_NE(DetectGltfSceneFeatures({material}, texture_infos) & Bit(GltfSceneFeature::TextureTransform), 0u);
}

TEST(GltfSceneFeatures, IgnoresInvalidAndUnreferencedTextures) {
  GltfShadeMaterial material;
  material.transmission_texture = 1;
  std::vector<GltfTextureInfo> invalid(2);
  EXPECT_EQ(DetectGltfSceneFeatures({material}, invalid), 0u);

  material = {};
  auto texture_infos = TextureInfos();
  texture_infos[1].uv_transform[2] = glm::vec2(0.25f, 0.5f);
  EXPECT_EQ(DetectGltfSceneFeatures({material}, texture_infos), 0u);
}

TEST(GltfSceneFeatures, DetectionIsMaterialOrderInvariant) {
  GltfShadeMaterial transmission;
  transmission.transmission_factor = 0.5f;
  GltfShadeMaterial sheen;
  sheen.sheen_color_factor = glm::vec3(0.5f);
  const auto forward = DetectGltfSceneFeatures({transmission, sheen}, TextureInfos());
  const auto reverse = DetectGltfSceneFeatures({sheen, transmission}, TextureInfos());
  EXPECT_EQ(forward, reverse);
}

TEST(GltfSceneFeatures, DefineHeaderIsDeterministicAndLayoutSafe) {
  const auto mask = Bit(GltfSceneFeature::VolumeScatter) | Bit(GltfSceneFeature::Unlit);
  const auto header = BuildGltfSceneFeatureDefines(mask);
  EXPECT_EQ(header, BuildGltfSceneFeatureDefines(mask));
  EXPECT_EQ(
      header.find("#define EE_GLTF_COMPILED_FEATURE_MASK " + std::to_string(PromoteGltfSceneFeatures(mask)) + "u\n"),
      0u);
  EXPECT_NE(header.find("#define EE_GLTF_USE_TRANSMISSION 1\n"), std::string::npos);
  EXPECT_NE(header.find("#define EE_GLTF_USE_VOLUME 1\n"), std::string::npos);
  EXPECT_NE(header.find("#define EE_GLTF_USE_VOLUME_SCATTER 1\n"), std::string::npos);
  EXPECT_NE(header.find("#define EE_GLTF_USE_UNLIT 1\n"), std::string::npos);
  EXPECT_NE(header.find("#define EE_GLTF_USE_CLEARCOAT 0\n"), std::string::npos);
  EXPECT_EQ(header.find("#define MAT_EXT_"), std::string::npos);
  EXPECT_LT(header.find("EE_GLTF_USE_TRANSMISSION"), header.find("EE_GLTF_USE_TEXTURE_TRANSFORM"));
}
