#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "GltfMaterialCache.hpp"
#include "Material.hpp"

using namespace evo_engine;

namespace {
constexpr float kEpsilon = 0.0001f;

void ExpectVec3Near(const glm::vec3& actual, const glm::vec3& expected) {
  EXPECT_NEAR(actual.x, expected.x, kEpsilon);
  EXPECT_NEAR(actual.y, expected.y, kEpsilon);
  EXPECT_NEAR(actual.z, expected.z, kEpsilon);
}

void ExpectVec4Near(const glm::vec4& actual, const glm::vec4& expected) {
  EXPECT_NEAR(actual.x, expected.x, kEpsilon);
  EXPECT_NEAR(actual.y, expected.y, kEpsilon);
  EXPECT_NEAR(actual.z, expected.z, kEpsilon);
  EXPECT_NEAR(actual.w, expected.w, kEpsilon);
}
}  // namespace

TEST(GltfMaterialConversion, MetallicRoughnessGltfMaterialMapsFactorsAndTextureInfos) {
  const auto gltf = YAML::Load(R"({
    "materials": [{
      "alphaMode": "MASK",
      "alphaCutoff": 0.42,
      "doubleSided": true,
      "emissiveFactor": [0.1, 0.2, 0.3],
      "normalTexture": {"index": 4, "texCoord": 2, "scale": 0.75},
      "occlusionTexture": {"index": 5, "strength": 0.25},
      "pbrMetallicRoughness": {
        "baseColorFactor": [0.25, 0.5, 0.75, 0.9],
        "metallicFactor": 0.6,
        "roughnessFactor": 0.2,
        "baseColorTexture": {
          "index": 2,
          "texCoord": 0,
          "extensions": {
            "KHR_texture_transform": {
              "offset": [0.1, 0.2],
              "scale": [2.0, 3.0],
              "rotation": 0.0,
              "texCoord": 1
            }
          }
        },
        "metallicRoughnessTexture": {"index": 3}
      }
    }]
  })");

  const auto materials = BuildGltfMaterialDataFromGltfNode(gltf, [](const int32_t texture_index) {
    return 100 + texture_index;
  });

  ASSERT_EQ(materials.size(), 1);
  const auto& material = materials[0].shade_material;
  EXPECT_EQ(material.pbr_model, static_cast<int32_t>(GltfPbrModel::MetallicRoughness));
  EXPECT_EQ(material.alpha_mode, static_cast<int32_t>(GltfAlphaMode::Mask));
  EXPECT_NEAR(material.alpha_cutoff, 0.42f, kEpsilon);
  EXPECT_EQ(material.double_sided, 1);
  ExpectVec4Near(material.pbr_base_color_factor, glm::vec4(0.25f, 0.5f, 0.75f, 0.9f));
  EXPECT_NEAR(material.pbr_metallic_factor, 0.6f, kEpsilon);
  EXPECT_NEAR(material.pbr_roughness_factor, 0.2f, kEpsilon);
  ExpectVec3Near(material.emissive_factor, glm::vec3(0.1f, 0.2f, 0.3f));
  EXPECT_NEAR(material.normal_texture_scale, 0.75f, kEpsilon);
  EXPECT_NEAR(material.occlusion_strength, 0.25f, kEpsilon);

  ASSERT_NE(material.pbr_base_color_texture, 0);
  const auto& base_color = materials[0].texture_infos[material.pbr_base_color_texture];
  EXPECT_EQ(base_color.index, 102);
  EXPECT_EQ(base_color.tex_coord, 1);
  EXPECT_NEAR(base_color.uv_transform[0][0], 2.0f, kEpsilon);
  EXPECT_NEAR(base_color.uv_transform[1][1], 3.0f, kEpsilon);
  EXPECT_NEAR(base_color.uv_transform[2][0], 0.1f, kEpsilon);
  EXPECT_NEAR(base_color.uv_transform[2][1], 0.2f, kEpsilon);

  ASSERT_NE(material.normal_texture, 0);
  EXPECT_EQ(materials[0].texture_infos[material.normal_texture].index, 104);
  EXPECT_EQ(materials[0].texture_infos[material.normal_texture].tex_coord, 1);
}

TEST(GltfMaterialConversion, BistroSpecularGlossinessGltfMaterialSelectsSpecGlossModel) {
  const auto gltf = YAML::Load(R"({
    "materials": [{
      "extensions": {
        "KHR_materials_pbrSpecularGlossiness": {
          "diffuseFactor": [0.8, 0.7, 0.6, 0.5],
          "specularFactor": [0.4, 0.3, 0.2],
          "glossinessFactor": 0.81,
          "diffuseTexture": {"index": 2},
          "specularGlossinessTexture": {"index": 3, "texCoord": 1}
        },
        "KHR_materials_emissive_strength": {
          "emissiveStrength": 2.5
        },
        "KHR_materials_transmission": {
          "transmissionFactor": 0.35,
          "transmissionTexture": {"index": 6}
        },
        "KHR_materials_ior": {
          "ior": 1.33
        },
        "KHR_materials_clearcoat": {
          "clearcoatFactor": 0.45,
          "clearcoatRoughnessFactor": 0.12,
          "clearcoatTexture": {"index": 7}
        },
        "KHR_materials_sheen": {
          "sheenColorFactor": [0.2, 0.3, 0.4],
          "sheenRoughnessFactor": 0.55,
          "sheenColorTexture": {"index": 8}
        }
      },
      "emissiveFactor": [0.1, 0.2, 0.3]
    }]
  })");

  const auto materials = BuildGltfMaterialDataFromGltfNode(gltf, [](const int32_t texture_index) {
    return 200 + texture_index;
  });

  ASSERT_EQ(materials.size(), 1);
  const auto& material = materials[0].shade_material;
  EXPECT_EQ(material.pbr_model, static_cast<int32_t>(GltfPbrModel::SpecularGlossiness));
  ExpectVec4Near(material.pbr_diffuse_factor, glm::vec4(0.8f, 0.7f, 0.6f, 0.5f));
  ExpectVec3Near(material.pbr_specular_factor, glm::vec3(0.4f, 0.3f, 0.2f));
  EXPECT_NEAR(material.pbr_glossiness_factor, 0.81f, kEpsilon);
  ExpectVec3Near(material.emissive_factor, glm::vec3(0.25f, 0.5f, 0.75f));
  EXPECT_NEAR(material.transmission_factor, 0.35f, kEpsilon);
  EXPECT_NEAR(material.ior, 1.33f, kEpsilon);
  EXPECT_NEAR(material.clearcoat_factor, 0.45f, kEpsilon);
  EXPECT_NEAR(material.clearcoat_roughness, 0.12f, kEpsilon);
  ExpectVec3Near(material.sheen_color_factor, glm::vec3(0.2f, 0.3f, 0.4f));
  EXPECT_NEAR(material.sheen_roughness_factor, 0.55f, kEpsilon);

  ASSERT_NE(material.pbr_diffuse_texture, 0);
  ASSERT_NE(material.pbr_specular_glossiness_texture, 0);
  EXPECT_EQ(materials[0].texture_infos[material.pbr_diffuse_texture].index, 202);
  EXPECT_EQ(materials[0].texture_infos[material.pbr_specular_glossiness_texture].index, 203);
  EXPECT_EQ(materials[0].texture_infos[material.pbr_specular_glossiness_texture].tex_coord, 1);
  EXPECT_EQ(materials[0].texture_infos[material.transmission_texture].index, 206);
  EXPECT_EQ(materials[0].texture_infos[material.clearcoat_texture].index, 207);
  EXPECT_EQ(materials[0].texture_infos[material.sheen_color_texture].index, 208);
}

TEST(GltfMaterialConversion, TransparentExtensionsImportDiffuseTransmissionAndVolumeScatter) {
  const auto gltf = YAML::Load(R"({
    "materials": [{
      "extensions": {
        "KHR_materials_diffuse_transmission": {
          "diffuseTransmissionFactor": 0.65,
          "diffuseTransmissionTexture": {"index": 2, "texCoord": 1},
          "diffuseTransmissionColorFactor": [0.7, 0.8, 0.9],
          "diffuseTransmissionColorTexture": {"index": 3}
        },
        "KHR_materials_volume_scatter": {
          "multiscatterColorFactor": [0.2, 0.3, 0.4],
          "multiscatterColor": [0.5, 0.6, 0.7],
          "scatterAnisotropy": 1.25
        }
      }
    }]
  })");

  const auto materials = BuildGltfMaterialDataFromGltfNode(gltf, [](const int32_t texture_index) {
    return 300 + texture_index;
  });

  ASSERT_EQ(materials.size(), 1);
  const auto& material = materials[0].shade_material;
  EXPECT_NEAR(material.diffuse_transmission_factor, 0.65f, kEpsilon);
  ExpectVec3Near(material.diffuse_transmission_color, glm::vec3(0.7f, 0.8f, 0.9f));
  ExpectVec3Near(material.multiscatter_color_factor, glm::vec3(0.5f, 0.6f, 0.7f));
  EXPECT_NEAR(material.scatter_anisotropy, 0.999f, kEpsilon);

  ASSERT_NE(material.diffuse_transmission_texture, 0);
  ASSERT_NE(material.diffuse_transmission_color_texture, 0);
  EXPECT_EQ(materials[0].texture_infos[material.diffuse_transmission_texture].index, 302);
  EXPECT_EQ(materials[0].texture_infos[material.diffuse_transmission_texture].tex_coord, 1);
  EXPECT_EQ(materials[0].texture_infos[material.diffuse_transmission_color_texture].index, 303);

  GltfMaterialCache cache;
  const auto material_index = cache.Append(materials[0]);
  ASSERT_EQ(material_index, 0);
  ASSERT_EQ(cache.GetShadeMaterials().size(), 1);
  const auto& cached_material = cache.GetShadeMaterials()[material_index];
  EXPECT_EQ(cache.GetTextureInfos()[cached_material.diffuse_transmission_texture].index, 302);
  EXPECT_EQ(cache.GetTextureInfos()[cached_material.diffuse_transmission_color_texture].index, 303);
}

TEST(GltfMaterialConversion, MsftTextureDdsOverridesTextureSourceUri) {
  const auto gltf = YAML::Load(R"({
    "textures": [{
      "source": 0,
      "extensions": {
        "MSFT_texture_dds": {"source": 1}
      }
    }],
    "images": [
      {"uri": "textures/base.png"},
      {"uri": "textures/base.dds"}
    ]
  })");

  EXPECT_EQ(ResolveGltfTextureUri(gltf, 0), "textures/base.dds");
  EXPECT_EQ(ResolveGltfTextureUri(gltf, 0, false), "textures/base.png");
}

TEST(GltfMaterialConversion, DdsTextureInfosApplyVerticalFlipAfterTextureTransform) {
  const auto gltf = YAML::Load(R"({
    "materials": [{
      "normalTexture": {"index": 4},
      "pbrMetallicRoughness": {
        "baseColorTexture": {
          "index": 2,
          "texCoord": 0,
          "extensions": {
            "KHR_texture_transform": {
              "offset": [0.1, 0.2],
              "scale": [2.0, 3.0],
              "rotation": 0.0,
              "texCoord": 1
            }
          }
        }
      }
    }]
  })");

  const auto materials = BuildGltfMaterialDataFromGltfNode(
      gltf,
      [](const int32_t texture_index) {
        return 100 + texture_index;
      },
      [](const int32_t texture_index) {
        return texture_index == 2;
      });

  ASSERT_EQ(materials.size(), 1);
  const auto& material = materials[0].shade_material;

  ASSERT_NE(material.pbr_base_color_texture, 0);
  const auto& base_color = materials[0].texture_infos[material.pbr_base_color_texture];
  EXPECT_EQ(base_color.index, 102);
  EXPECT_EQ(base_color.tex_coord, 1);
  EXPECT_NEAR(base_color.uv_transform[0][0], 2.0f, kEpsilon);
  EXPECT_NEAR(base_color.uv_transform[0][1], 0.0f, kEpsilon);
  EXPECT_NEAR(base_color.uv_transform[1][0], 0.0f, kEpsilon);
  EXPECT_NEAR(base_color.uv_transform[1][1], -3.0f, kEpsilon);
  EXPECT_NEAR(base_color.uv_transform[2][0], 0.1f, kEpsilon);
  EXPECT_NEAR(base_color.uv_transform[2][1], 0.8f, kEpsilon);

  ASSERT_NE(material.normal_texture, 0);
  const auto& normal = materials[0].texture_infos[material.normal_texture];
  EXPECT_EQ(normal.index, 104);
  EXPECT_NEAR(normal.uv_transform[0][0], 1.0f, kEpsilon);
  EXPECT_NEAR(normal.uv_transform[1][1], 1.0f, kEpsilon);
  EXPECT_NEAR(normal.uv_transform[2][1], 0.0f, kEpsilon);
}

TEST(GltfMaterialConversion, EmptyGltfMaterialPreservesReferenceMetallicDefault) {
  const auto gltf = YAML::Load(R"({
    "materials": [{}]
  })");

  const auto materials = BuildGltfMaterialDataFromGltfNode(gltf, [](const int32_t texture_index) {
    return texture_index;
  });

  ASSERT_EQ(materials.size(), 1);
  EXPECT_NEAR(materials[0].shade_material.pbr_metallic_factor, 1.0f, kEpsilon);
  EXPECT_NEAR(materials[0].shade_material.pbr_roughness_factor, 1.0f, kEpsilon);
}

TEST(GltfMaterialConversion, NativeMaterialAssetDefaultsToDielectric) {
  Application app;
  Material material;

  const auto material_data = BuildMaterialGltfData(material);

  EXPECT_NEAR(material_data.shade_material.pbr_metallic_factor, 0.0f, kEpsilon);
  EXPECT_NEAR(material_data.shade_material.pbr_roughness_factor, 1.0f, kEpsilon);
}

TEST(GltfMaterialConversion, CanonicalMaterialAssetBuildsGltfMaterialDataDirectly) {
  Application app;
  Material material;
  auto& source = material.material_data;
  auto& source_shade = source.shade_material;
  source_shade.pbr_base_color_factor = glm::vec4(0.2f, 0.4f, 0.6f, 0.75f);
  source_shade.pbr_metallic_factor = 0.7f;
  source_shade.pbr_roughness_factor = 0.8f;
  source_shade.emissive_factor = glm::vec3(0.5f, 0.25f, 0.125f);
  source_shade.transmission_factor = 0.25f;
  source_shade.ior = 1.2f;
  source_shade.clearcoat_factor = 0.3f;
  source_shade.clearcoat_roughness = 0.4f;
  source_shade.specular_factor = 0.9f;
  source_shade.sheen_color_factor = glm::vec3(0.1f);
  source_shade.sheen_roughness_factor = 0.2f;
  source_shade.alpha_mode = static_cast<int32_t>(GltfAlphaMode::Blend);
  source_shade.double_sided = 1;
  source_shade.pbr_base_color_texture = 1;
  source.texture_infos.resize(2);
  source.texture_infos[1].index = 42;
  source.texture_infos[1].tex_coord = 1;

  const auto material_data = BuildMaterialGltfData(material);
  const auto& shade_material = material_data.shade_material;

  EXPECT_EQ(shade_material.pbr_model, static_cast<int32_t>(GltfPbrModel::MetallicRoughness));
  EXPECT_EQ(shade_material.alpha_mode, static_cast<int32_t>(GltfAlphaMode::Blend));
  EXPECT_EQ(shade_material.double_sided, 1);
  ExpectVec4Near(shade_material.pbr_base_color_factor, glm::vec4(0.2f, 0.4f, 0.6f, 0.75f));
  EXPECT_NEAR(shade_material.pbr_metallic_factor, 0.7f, kEpsilon);
  EXPECT_NEAR(shade_material.pbr_roughness_factor, 0.8f, kEpsilon);
  ExpectVec3Near(shade_material.emissive_factor, glm::vec3(0.5f, 0.25f, 0.125f));
  EXPECT_NEAR(shade_material.transmission_factor, 0.25f, kEpsilon);
  EXPECT_NEAR(shade_material.ior, 1.2f, kEpsilon);
  EXPECT_NEAR(shade_material.clearcoat_factor, 0.3f, kEpsilon);
  EXPECT_NEAR(shade_material.clearcoat_roughness, 0.4f, kEpsilon);
  EXPECT_NEAR(shade_material.specular_factor, 0.9f, kEpsilon);
  ExpectVec3Near(shade_material.sheen_color_factor, glm::vec3(0.1f));
  EXPECT_NEAR(shade_material.sheen_roughness_factor, 0.2f, kEpsilon);
  ASSERT_EQ(material_data.texture_infos.size(), 2);
  EXPECT_EQ(material_data.texture_infos[1].index, 42);
  EXPECT_EQ(material_data.texture_infos[1].tex_coord, 1);
  EXPECT_EQ(shade_material.pbr_base_color_texture, 1);
}

TEST(GltfMaterialConversion, CanonicalMaterialAssetSynchronizesDrawSettings) {
  Application app;
  Material material;
  material.material_data.shade_material.alpha_mode = static_cast<int32_t>(GltfAlphaMode::Blend);
  material.material_data.shade_material.double_sided = 0;

  material.MarkDirty();

  EXPECT_TRUE(material.draw_settings.blending);
  EXPECT_EQ(material.draw_settings.cull_mode, VK_CULL_MODE_NONE);

  material.draw_settings.cull_mode = VK_CULL_MODE_BACK_BIT;
  material.material_data.shade_material.double_sided = 1;
  material.MarkDirty();

  EXPECT_TRUE(material.draw_settings.blending);
  EXPECT_EQ(material.draw_settings.cull_mode, VK_CULL_MODE_NONE);
}
