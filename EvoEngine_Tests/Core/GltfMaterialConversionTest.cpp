#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "GltfMaterialCache.hpp"
#include "Material.hpp"
#include "Mesh.hpp"

using namespace evo_engine;

namespace {
constexpr float kEpsilon = 0.0001f;

void ExpectVec2Near(const glm::vec2& actual, const glm::vec2& expected) {
  EXPECT_NEAR(actual.x, expected.x, kEpsilon);
  EXPECT_NEAR(actual.y, expected.y, kEpsilon);
}

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
  EXPECT_EQ(base_color.color_space, static_cast<int32_t>(GltfTextureColorSpace::Srgb));

  ASSERT_NE(material.normal_texture, 0);
  EXPECT_EQ(materials[0].texture_infos[material.normal_texture].index, 104);
  EXPECT_EQ(materials[0].texture_infos[material.normal_texture].tex_coord, 1);
  EXPECT_EQ(materials[0].texture_infos[material.normal_texture].color_space,
            static_cast<int32_t>(GltfTextureColorSpace::Linear));
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
  EXPECT_EQ(materials[0].texture_infos[material.pbr_diffuse_texture].color_space,
            static_cast<int32_t>(GltfTextureColorSpace::Srgb));
  EXPECT_EQ(materials[0].texture_infos[material.pbr_specular_glossiness_texture].color_space,
            static_cast<int32_t>(GltfTextureColorSpace::Srgb));
  EXPECT_EQ(materials[0].texture_infos[material.transmission_texture].index, 206);
  EXPECT_EQ(materials[0].texture_infos[material.clearcoat_texture].index, 207);
  EXPECT_EQ(materials[0].texture_infos[material.sheen_color_texture].index, 208);
  EXPECT_EQ(materials[0].texture_infos[material.sheen_color_texture].color_space,
            static_cast<int32_t>(GltfTextureColorSpace::Srgb));
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
  EXPECT_EQ(materials[0].texture_infos[material.diffuse_transmission_texture].color_space,
            static_cast<int32_t>(GltfTextureColorSpace::Linear));
  EXPECT_EQ(materials[0].texture_infos[material.diffuse_transmission_color_texture].color_space,
            static_cast<int32_t>(GltfTextureColorSpace::Srgb));

  GltfMaterialCache cache;
  const auto material_index = cache.Append(materials[0]);
  ASSERT_EQ(material_index, 0);
  ASSERT_EQ(cache.GetShadeMaterials().size(), 1);
  const auto& cached_material = cache.GetShadeMaterials()[material_index];
  EXPECT_EQ(cache.GetTextureInfos()[cached_material.diffuse_transmission_texture].index, 302);
  EXPECT_EQ(cache.GetTextureInfos()[cached_material.diffuse_transmission_color_texture].index, 303);
}

TEST(GltfMaterialConversion, AdvancedRayExtensionsImportNormativeFactorsTexturesAndRotation) {
  const auto gltf = YAML::Load(R"({
    "materials": [{
      "extensions": {
        "KHR_materials_specular": {
          "specularFactor": 0.0,
          "specularColorFactor": [1.5, -0.25, 0.4]
        },
        "KHR_materials_ior": {"ior": 0.0},
        "KHR_materials_iridescence": {
          "iridescenceFactor": 1.25,
          "iridescenceIor": 0.5,
          "iridescenceThicknessMinimum": 500.0,
          "iridescenceThicknessMaximum": -20.0,
          "iridescenceTexture": {
            "index": 2,
            "extensions": {"KHR_texture_transform": {"texCoord": 1, "offset": [0.1, 0.2]}}
          },
          "iridescenceThicknessTexture": {"index": 3}
        },
        "KHR_materials_anisotropy": {
          "anisotropyStrength": 2.0,
          "anisotropyRotation": 1.5707963267948966,
          "anisotropyTexture": {"index": 4, "texCoord": 1}
        },
        "KHR_materials_dispersion": {"dispersion": 2.5},
        "KHR_materials_retroreflection": {
          "retroreflectionFactor": 1.5,
          "retroreflectionTexture": {"index": 5}
        }
      }
    }, {
      "extensions": {
        "EXT_materials_retroreflection": {
          "retroreflectionFactor": 0.25,
          "retroreflectionTexture": {"index": 6}
        }
      }
    }]
  })");

  const auto materials = BuildGltfMaterialDataFromGltfNode(gltf, [](const int32_t texture_index) {
    return 400 + texture_index;
  });

  ASSERT_EQ(materials.size(), 2);
  const auto& material = materials[0].shade_material;
  EXPECT_FLOAT_EQ(material.specular_factor, 0.0f);
  ExpectVec3Near(material.specular_color_factor, glm::vec3(1.5f, 0.0f, 0.4f));
  EXPECT_FLOAT_EQ(material.ior, 0.0f);
  EXPECT_FLOAT_EQ(material.iridescence_factor, 1.0f);
  EXPECT_FLOAT_EQ(material.iridescence_ior, 1.0f);
  EXPECT_FLOAT_EQ(material.iridescence_thickness_minimum, 500.0f);
  EXPECT_FLOAT_EQ(material.iridescence_thickness_maximum, 0.0f);
  EXPECT_FLOAT_EQ(material.anisotropy_strength, 1.0f);
  ExpectVec2Near(material.anisotropy_rotation, glm::vec2(0.0f, 1.0f));
  EXPECT_FLOAT_EQ(material.dispersion, 2.5f);
  EXPECT_FLOAT_EQ(material.retroreflection_factor, 1.0f);

  ASSERT_NE(material.iridescence_texture, 0);
  ASSERT_NE(material.iridescence_thickness_texture, 0);
  ASSERT_NE(material.anisotropy_texture, 0);
  ASSERT_NE(material.retroreflection_texture, 0);
  const auto& iridescence = materials[0].texture_infos[material.iridescence_texture];
  EXPECT_EQ(iridescence.index, 402);
  EXPECT_EQ(iridescence.tex_coord, 1);
  EXPECT_FLOAT_EQ(iridescence.uv_transform[2][0], 0.1f);
  EXPECT_FLOAT_EQ(iridescence.uv_transform[2][1], 0.2f);
  EXPECT_EQ(iridescence.color_space, static_cast<int32_t>(GltfTextureColorSpace::Linear));
  EXPECT_EQ(materials[0].texture_infos[material.iridescence_thickness_texture].index, 403);
  EXPECT_EQ(materials[0].texture_infos[material.anisotropy_texture].index, 404);
  EXPECT_EQ(materials[0].texture_infos[material.anisotropy_texture].tex_coord, 1);
  EXPECT_EQ(materials[0].texture_infos[material.retroreflection_texture].index, 405);
  EXPECT_EQ(materials[0].texture_infos[material.retroreflection_texture].color_space,
            static_cast<int32_t>(GltfTextureColorSpace::Linear));

  GltfMaterialCache cache;
  EXPECT_EQ(cache.Append(materials[0]), 0);
  const auto& cached = cache.GetShadeMaterials()[0];
  EXPECT_EQ(cache.GetTextureInfos()[cached.iridescence_texture].index, 402);
  EXPECT_EQ(cache.GetTextureInfos()[cached.iridescence_thickness_texture].index, 403);
  EXPECT_EQ(cache.GetTextureInfos()[cached.anisotropy_texture].index, 404);
  EXPECT_EQ(cache.GetTextureInfos()[cached.retroreflection_texture].index, 405);

  const auto& alias_material = materials[1].shade_material;
  EXPECT_FLOAT_EQ(alias_material.specular_factor, 1.0f);
  ExpectVec2Near(alias_material.anisotropy_rotation, glm::vec2(1.0f, 0.0f));
  EXPECT_FLOAT_EQ(alias_material.retroreflection_factor, 0.25f);
  EXPECT_EQ(materials[1].texture_infos[alias_material.retroreflection_texture].index, 406);
}

TEST(GltfMaterialConversion, AdvancedRayExtensionDefaultsAndLowerBoundsMatchKhronos) {
  const auto gltf = YAML::Load(R"({
    "materials": [{}, {
      "extensions": {
        "KHR_materials_ior": {"ior": 0.5},
        "KHR_materials_anisotropy": {"anisotropyStrength": -1.0},
        "KHR_materials_dispersion": {"dispersion": -3.0}
      }
    }]
  })");
  const auto materials = BuildGltfMaterialDataFromGltfNode(gltf, [](const int32_t texture_index) {
    return texture_index;
  });

  ASSERT_EQ(materials.size(), 2);
  for (const auto& material_data : materials) {
    const auto& material = material_data.shade_material;
    EXPECT_FLOAT_EQ(material.specular_factor, 1.0f);
    EXPECT_FLOAT_EQ(material.iridescence_factor, 0.0f);
    EXPECT_FLOAT_EQ(material.iridescence_ior, 1.3f);
    EXPECT_FLOAT_EQ(material.iridescence_thickness_minimum, 100.0f);
    EXPECT_FLOAT_EQ(material.iridescence_thickness_maximum, 400.0f);
    EXPECT_FLOAT_EQ(material.anisotropy_strength, 0.0f);
    ExpectVec2Near(material.anisotropy_rotation, glm::vec2(1.0f, 0.0f));
    EXPECT_FLOAT_EQ(material.dispersion, 0.0f);
    EXPECT_FLOAT_EQ(material.retroreflection_factor, 0.0f);
  }
  EXPECT_FLOAT_EQ(materials[0].shade_material.ior, 1.5f);
  EXPECT_FLOAT_EQ(materials[1].shade_material.ior, 1.0f);
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

TEST(GltfMaterialConversion, TextureInfosApplyRequestedVerticalFlipAfterTextureTransform) {
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

TEST(GltfMaterialConversion, TextureTransformRotationKeepsKhronosOrderBeforeStorageFlip) {
  const auto gltf = YAML::Load(R"({
    "materials": [{
      "pbrMetallicRoughness": {
        "baseColorTexture": {
          "index": 2,
          "extensions": {
            "KHR_texture_transform": {
              "offset": [0.25, 0.4],
              "scale": [2.0, 3.0],
              "rotation": 1.5707963267948966
            }
          }
        },
        "metallicRoughnessTexture": {
          "index": 3,
          "extensions": {
            "KHR_texture_transform": {
              "offset": [0.25, 0.4],
              "scale": [2.0, 3.0],
              "rotation": 1.5707963267948966
            }
          }
        }
      }
    }]
  })");

  const auto materials = BuildGltfMaterialDataFromGltfNode(
      gltf,
      [](const int32_t texture_index) {
        return texture_index;
      },
      [](const int32_t texture_index) {
        return texture_index == 2;
      });

  ASSERT_EQ(materials.size(), 1);
  const auto& shade = materials[0].shade_material;
  const auto& flipped = materials[0].texture_infos[shade.pbr_base_color_texture].uv_transform;
  const auto& unflipped = materials[0].texture_infos[shade.pbr_metallic_roughness_texture].uv_transform;
  EXPECT_NEAR(flipped[0][0], 0.0f, kEpsilon);
  EXPECT_NEAR(flipped[0][1], -2.0f, kEpsilon);
  EXPECT_NEAR(flipped[1][0], -3.0f, kEpsilon);
  EXPECT_NEAR(flipped[1][1], 0.0f, kEpsilon);
  EXPECT_NEAR(flipped[2][0], 0.25f, kEpsilon);
  EXPECT_NEAR(flipped[2][1], 0.6f, kEpsilon);
  EXPECT_NEAR(unflipped[0][0], 0.0f, kEpsilon);
  EXPECT_NEAR(unflipped[0][1], 2.0f, kEpsilon);
  EXPECT_NEAR(unflipped[1][0], -3.0f, kEpsilon);
  EXPECT_NEAR(unflipped[1][1], 0.0f, kEpsilon);
  EXPECT_NEAR(unflipped[2][0], 0.25f, kEpsilon);
  EXPECT_NEAR(unflipped[2][1], 0.4f, kEpsilon);
}

TEST(GltfMaterialConversion, SemanticColorSpaceAvoidsHardwareSrgbDoubleDecode) {
  const auto gltf = YAML::Load(R"({
    "materials": [{
      "normalTexture": {"index": 2},
      "pbrMetallicRoughness": {"baseColorTexture": {"index": 3}}
    }]
  })");

  const auto shader_decoded = BuildGltfMaterialDataFromGltfNode(gltf, [](const int32_t texture_index) {
    return texture_index;
  });
  const auto hardware_decoded = BuildGltfMaterialDataFromGltfNode(
      gltf,
      [](const int32_t texture_index) {
        return texture_index;
      },
      {},
      [](const int32_t texture_index) {
        return texture_index == 3;
      });

  ASSERT_EQ(shader_decoded.size(), 1);
  const auto& shader_shade = shader_decoded[0].shade_material;
  EXPECT_EQ(shader_decoded[0].texture_infos[shader_shade.pbr_base_color_texture].color_space,
            static_cast<int32_t>(GltfTextureColorSpace::Srgb));
  EXPECT_EQ(shader_decoded[0].texture_infos[shader_shade.normal_texture].color_space,
            static_cast<int32_t>(GltfTextureColorSpace::Linear));

  ASSERT_EQ(hardware_decoded.size(), 1);
  const auto& hardware_shade = hardware_decoded[0].shade_material;
  EXPECT_EQ(hardware_decoded[0].texture_infos[hardware_shade.pbr_base_color_texture].color_space,
            static_cast<int32_t>(GltfTextureColorSpace::Linear));
  EXPECT_EQ(hardware_decoded[0].texture_infos[hardware_shade.normal_texture].color_space,
            static_cast<int32_t>(GltfTextureColorSpace::Linear));
}

TEST(GltfMaterialConversion, MissingTangentsUseNormalTexturesSecondaryUvSetBeforeUpload) {
  Application app;
  VertexAttributes attributes;
  attributes.normal = true;
  attributes.tex_coord = true;
  attributes.tex_coord_1 = true;

  std::vector<Vertex> vertices(3);
  vertices[0].position = glm::vec3(0.0f, 0.0f, 0.0f);
  vertices[1].position = glm::vec3(1.0f, 0.0f, 0.0f);
  vertices[2].position = glm::vec3(0.0f, 1.0f, 0.0f);
  for (auto& vertex : vertices) {
    vertex.normal = glm::vec3(0.0f, 0.0f, 1.0f);
  }
  vertices[0].tex_coord = glm::vec2(0.0f, 0.0f);
  vertices[1].tex_coord = glm::vec2(1.0f, 0.0f);
  vertices[2].tex_coord = glm::vec2(0.0f, 1.0f);
  vertices[0].tex_coord_1 = glm::vec2(0.0f, 0.0f);
  vertices[1].tex_coord_1 = glm::vec2(0.0f, 1.0f);
  vertices[2].tex_coord_1 = glm::vec2(1.0f, 0.0f);

  Mesh mesh;
  mesh.OnCreate();
  mesh.SetVertices(attributes, vertices, {glm::uvec3(0, 1, 2)}, true);

  ASSERT_EQ(mesh.PeekVertices().size(), 3);
  EXPECT_NEAR(mesh.PeekVertices()[0].tangent.x, 0.0f, kEpsilon);
  EXPECT_NEAR(mesh.PeekVertices()[0].tangent.y, 1.0f, kEpsilon);
  EXPECT_NEAR(mesh.PeekVertices()[0].tangent.z, 0.0f, kEpsilon);
  EXPECT_NEAR(mesh.PeekVertices()[0].vertex_info3, -1.0f, kEpsilon);
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
  EXPECT_EQ(material.draw_settings.cull_mode, VK_CULL_MODE_BACK_BIT);
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
  EXPECT_EQ(material.draw_settings.cull_mode, VK_CULL_MODE_BACK_BIT);

  material.draw_settings.cull_mode = VK_CULL_MODE_BACK_BIT;
  material.material_data.shade_material.double_sided = 1;
  material.MarkDirty();

  EXPECT_TRUE(material.draw_settings.blending);
  EXPECT_EQ(material.draw_settings.cull_mode, VK_CULL_MODE_NONE);
}

TEST(GltfMaterialConversion, TransmissiveMaterialSynchronizesTransparentPass) {
  Application app;
  Material material;
  auto& shade_material = material.material_data.shade_material;

  shade_material.alpha_mode = static_cast<int32_t>(GltfAlphaMode::Opaque);
  shade_material.transmission_factor = 0.45f;
  material.MarkDirty();

  EXPECT_TRUE(GltfMaterialRequiresTransparentPass(shade_material));
  EXPECT_TRUE(material.draw_settings.blending);

  material.draw_settings.blending = false;
  shade_material.transmission_factor = 0.0f;
  shade_material.diffuse_transmission_factor = 0.35f;
  material.MarkDirty();

  EXPECT_TRUE(GltfMaterialRequiresTransparentPass(shade_material));
  EXPECT_TRUE(material.draw_settings.blending);

  material.draw_settings.blending = true;
  shade_material.diffuse_transmission_factor = 0.0f;
  material.MarkDirty();

  EXPECT_FALSE(GltfMaterialRequiresTransparentPass(shade_material));
  EXPECT_FALSE(material.draw_settings.blending);
}
