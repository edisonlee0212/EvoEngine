#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "AssetManager.hpp"
#include "GltfMaterialCache.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "Platform.hpp"
#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "SkinnedMesh.hpp"

#include <array>
#include <chrono>
#include <fstream>
#include <iterator>

using namespace evo_engine;

namespace {
constexpr float kEpsilon = 0.0001f;
constexpr uint64_t kProjectGltfHandle = 0xE703'0000'0000'0001ull;
constexpr uint64_t kProjectDdsHandle = 0xE703'0000'0000'0002ull;
constexpr uint64_t kProjectBrokenGltfHandle = 0xE703'0000'0000'0003ull;
constexpr uint64_t kProjectBrokenDdsHandle = 0xE703'0000'0000'0004ull;

class TempGltfProject {
 public:
  TempGltfProject() {
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::temp_directory_path() / ("EvoEngineGltfProject_" + std::to_string(suffix));
    std::filesystem::create_directories(ModelsPath());
    std::ofstream(ProjectPath()) << "{}\n";
  }

  ~TempGltfProject() {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] std::filesystem::path ProjectPath() const {
    return root_ / "GltfProject.eveproj";
  }

  [[nodiscard]] std::filesystem::path ModelsPath() const {
    return root_ / "Assets" / "Models";
  }

 private:
  std::filesystem::path root_;
};

void WriteAssetMetadata(const std::filesystem::path& path, const std::string& type_name, const uint64_t handle) {
  std::ofstream metadata(path.string() + ".evefilemeta");
  metadata << "asset_extension_: " << path.extension().string() << '\n';
  metadata << "asset_file_name_: " << path.stem().string() << '\n';
  metadata << "asset_type_name_: " << type_name << '\n';
  metadata << "asset_handle_: " << handle << '\n';
}

void WriteBc7UnormDds(const std::filesystem::path& path) {
  std::array<std::byte, 164> bytes{};
  const auto write_u32 = [&](const size_t offset, const uint32_t value) {
    for (size_t byte = 0; byte < sizeof(value); ++byte) {
      bytes[offset + byte] = static_cast<std::byte>((value >> (byte * 8)) & 0xff);
    }
  };
  write_u32(0, 0x20534444u);
  write_u32(4, 124);
  write_u32(12, 4);
  write_u32(16, 4);
  write_u32(28, 1);
  write_u32(76, 32);
  write_u32(84, 0x30315844u);
  write_u32(128, 98);
  write_u32(132, 3);
  write_u32(140, 1);
  std::ofstream stream(path, std::ios::binary);
  stream.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

void WriteGltfDdsVariant(const std::filesystem::path& source_path, const std::filesystem::path& output_path,
                         const std::string& dds_uri) {
  std::ifstream source_stream(source_path);
  std::string source((std::istreambuf_iterator<char>(source_stream)), std::istreambuf_iterator<char>());
  const auto replace = [&](const std::string& from, const std::string& to) {
    const auto position = source.find(from);
    if (position == std::string::npos) {
      throw std::runtime_error("BoxTextured glTF fixture shape changed.");
    }
    source.replace(position, from.size(), to);
  };
  replace(R"("sampler": 0,
            "source": 0)",
          R"("sampler": 0,
            "source": 0,
            "extensions": {"MSFT_texture_dds": {"source": 1}})");
  replace(R"({
            "uri": "CesiumLogoFlat.png"
        })",
          R"({
            "uri": "CesiumLogoFlat.png"
        },
        {
            "uri": ")" +
              dds_uri + R"("
        })");
  std::ofstream(output_path, std::ios::binary) << source;
}

ApplicationInitializationSettings EmptyProjectSettings() {
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  return settings;
}

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

std::shared_ptr<Material> FindFirstMaterial(const std::shared_ptr<Prefab>& prefab) {
  for (const auto& holder : prefab->private_components) {
    if (const auto renderer = std::dynamic_pointer_cast<MeshRenderer>(holder.private_component)) {
      if (const auto material = renderer->material.Get<Material>()) {
        return material;
      }
    }
  }
  for (const auto& child : prefab->child_prefabs) {
    if (const auto material = FindFirstMaterial(child)) {
      return material;
    }
  }
  return {};
}

std::shared_ptr<Mesh> FindFirstMesh(const std::shared_ptr<Prefab>& prefab) {
  for (const auto& holder : prefab->private_components) {
    if (const auto renderer = std::dynamic_pointer_cast<MeshRenderer>(holder.private_component)) {
      if (const auto mesh = renderer->mesh.Get<Mesh>()) {
        return mesh;
      }
    }
  }
  for (const auto& child : prefab->child_prefabs) {
    if (const auto mesh = FindFirstMesh(child)) {
      return mesh;
    }
  }
  return {};
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
  EXPECT_EQ(materials[0].texture_infos[material.normal_texture].tex_coord, 2);
  EXPECT_EQ(materials[0].texture_infos[material.normal_texture].color_space,
            static_cast<int32_t>(GltfTextureColorSpace::Linear));
}

TEST(GltfMaterialConversion, SupportsFourUvSetsAndDisablesOnlyOutOfRangeBindings) {
  const auto gltf = YAML::Load(R"({
    "materials": [{
      "normalTexture": {"index": 1, "texCoord": 2},
      "occlusionTexture": {"index": 2, "texCoord": 4},
      "pbrMetallicRoughness": {
        "baseColorTexture": {
          "index": 3,
          "texCoord": 1,
          "extensions": {"KHR_texture_transform": {"texCoord": 3}}
        },
        "metallicRoughnessTexture": {"index": 4, "texCoord": 0}
      }
    }]
  })");

  std::vector<std::string> diagnostics;
  std::vector<int32_t> resolved;
  const auto materials = BuildGltfMaterialDataFromGltfNode(
      gltf,
      [&](const int32_t texture_index) {
        resolved.push_back(texture_index);
        return 100 + texture_index;
      },
      {}, {},
      [&](const std::string& message) {
        diagnostics.push_back(message);
      });

  ASSERT_EQ(materials.size(), 1);
  const auto& material = materials[0].shade_material;
  ASSERT_NE(material.normal_texture, 0);
  ASSERT_NE(material.pbr_base_color_texture, 0);
  ASSERT_NE(material.pbr_metallic_roughness_texture, 0);
  EXPECT_EQ(material.occlusion_texture, 0);
  EXPECT_EQ(materials[0].texture_infos[material.normal_texture].tex_coord, 2);
  EXPECT_EQ(materials[0].texture_infos[material.pbr_base_color_texture].tex_coord, 3);
  EXPECT_EQ(materials[0].texture_infos[material.pbr_metallic_roughness_texture].tex_coord, 0);
  EXPECT_EQ(resolved, (std::vector<int32_t>{1, 3, 4}));
  ASSERT_EQ(diagnostics.size(), 1);
  EXPECT_NE(diagnostics[0].find("TEXCOORD_4"), std::string::npos);
  EXPECT_NE(diagnostics[0].find("supported range 0..3"), std::string::npos);
}

TEST(GltfMaterialConversion, GltfSamplerEnumsMapToVulkanWithoutReferenceMipSwap) {
  const auto gltf = YAML::Load(R"({
    "textures": [
      {"sampler": 0}, {"sampler": 1}, {"sampler": 2},
      {"sampler": 3}, {"sampler": 4}, {"sampler": 5}, {"sampler": 6}
    ],
    "samplers": [
      {"magFilter": 9728, "minFilter": 9728, "wrapS": 33071, "wrapT": 33648},
      {"minFilter": 9729}, {"minFilter": 9984}, {"minFilter": 9985},
      {"minFilter": 9986}, {"minFilter": 9987},
      {"magFilter": 7, "minFilter": 8, "wrapS": 9, "wrapT": 10}
    ]
  })");

  const auto nearest = ReadGltfSamplerInfo(gltf, 0);
  EXPECT_EQ(nearest.mag_filter, VK_FILTER_NEAREST);
  EXPECT_EQ(nearest.min_filter, VK_FILTER_NEAREST);
  EXPECT_EQ(nearest.mipmap_mode, VK_SAMPLER_MIPMAP_MODE_NEAREST);
  EXPECT_FLOAT_EQ(nearest.max_lod, 0.0f);
  EXPECT_EQ(nearest.address_mode_u, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE);
  EXPECT_EQ(nearest.address_mode_v, VK_SAMPLER_ADDRESS_MODE_MIRRORED_REPEAT);

  const auto linear_no_mip = ReadGltfSamplerInfo(gltf, 1);
  EXPECT_EQ(linear_no_mip.min_filter, VK_FILTER_LINEAR);
  EXPECT_FLOAT_EQ(linear_no_mip.max_lod, 0.0f);
  EXPECT_EQ(ReadGltfSamplerInfo(gltf, 2).mipmap_mode, VK_SAMPLER_MIPMAP_MODE_NEAREST);
  EXPECT_EQ(ReadGltfSamplerInfo(gltf, 2).min_filter, VK_FILTER_NEAREST);
  EXPECT_EQ(ReadGltfSamplerInfo(gltf, 3).mipmap_mode, VK_SAMPLER_MIPMAP_MODE_NEAREST);
  EXPECT_EQ(ReadGltfSamplerInfo(gltf, 3).min_filter, VK_FILTER_LINEAR);
  EXPECT_EQ(ReadGltfSamplerInfo(gltf, 4).mipmap_mode, VK_SAMPLER_MIPMAP_MODE_LINEAR);
  EXPECT_EQ(ReadGltfSamplerInfo(gltf, 4).min_filter, VK_FILTER_NEAREST);
  EXPECT_EQ(ReadGltfSamplerInfo(gltf, 5).mipmap_mode, VK_SAMPLER_MIPMAP_MODE_LINEAR);
  EXPECT_EQ(ReadGltfSamplerInfo(gltf, 5).min_filter, VK_FILTER_LINEAR);

  std::vector<std::string> diagnostics;
  const auto invalid = ReadGltfSamplerInfo(gltf, 6, [&](const std::string& message) {
    diagnostics.push_back(message);
  });
  EXPECT_EQ(invalid.mag_filter, VK_FILTER_LINEAR);
  EXPECT_EQ(invalid.min_filter, VK_FILTER_LINEAR);
  EXPECT_EQ(invalid.address_mode_u, VK_SAMPLER_ADDRESS_MODE_REPEAT);
  EXPECT_EQ(invalid.address_mode_v, VK_SAMPLER_ADDRESS_MODE_REPEAT);
  EXPECT_EQ(diagnostics.size(), 4);
}

TEST(GltfMaterialConversion, RootReaderParsesExternalAndBinaryContainersWithEmbeddedUris) {
  const std::string json = R"({
    "asset": {"version": "2.0"},
    "buffers": [{"uri": "data:application/octet-stream;base64,AAAA", "byteLength": 3}],
    "images": [{"uri": "data:image/png;base64,iVBORw0KGgo="}],
    "materials": [{"extensions": {"KHR_materials_clearcoat": {"clearcoatFactor": 0.75}}}]
  })";
  const auto root = std::filesystem::temp_directory_path() / "EvoEngine_GltfRootReader_M9";
  std::filesystem::create_directories(root);
  const auto gltf_path = root / "embedded.gltf";
  const auto glb_path = root / "embedded.glb";
  {
    std::ofstream stream(gltf_path, std::ios::binary);
    stream << json;
  }

  std::string padded_json = json;
  while (padded_json.size() % 4 != 0) {
    padded_json.push_back(' ');
  }
  const uint32_t total_length = static_cast<uint32_t>(20 + padded_json.size());
  const uint32_t header[] = {0x46546c67u, 2u, total_length, static_cast<uint32_t>(padded_json.size()), 0x4e4f534au};
  {
    std::ofstream stream(glb_path, std::ios::binary);
    stream.write(reinterpret_cast<const char*>(header), sizeof(header));
    stream.write(padded_json.data(), static_cast<std::streamsize>(padded_json.size()));
  }

  const auto external = ReadGltfRootNode(gltf_path);
  const auto binary = ReadGltfRootNode(glb_path);
  ASSERT_TRUE(external["materials"][0]["extensions"]["KHR_materials_clearcoat"]);
  ASSERT_TRUE(binary["materials"][0]["extensions"]["KHR_materials_clearcoat"]);
  EXPECT_FLOAT_EQ(binary["materials"][0]["extensions"]["KHR_materials_clearcoat"]["clearcoatFactor"].as<float>(),
                  0.75f);
  EXPECT_EQ(external["buffers"][0]["uri"].as<std::string>().find("data:"), 0);
  EXPECT_EQ(binary["images"][0]["uri"].as<std::string>().find("data:image/png"), 0);

  std::error_code error;
  std::filesystem::remove_all(root, error);
}

TEST(GltfMaterialConversion, PrefabImportsExternalEmbeddedAndBinaryGltfTexturesWithSamplers) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  const auto model_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "Extern" / "3rdParty" / "assimp" /
                          "assimp" / "test" / "models" / "glTF2";
  const std::filesystem::path paths[] = {
      model_root / "BoxTextured-glTF" / "BoxTextured.gltf",
      model_root / "BoxTextured-glTF-Embedded" / "BoxTextured.gltf",
      model_root / "BoxTextured-glTF-Binary" / "BoxTextured.glb",
  };

  const auto validate_import = [](const std::filesystem::path& path) {
    ASSERT_TRUE(std::filesystem::exists(path)) << path.string();
    const auto prefab = AssetManager::CreateTemporaryAsset<Prefab>();
    ASSERT_TRUE(prefab->Import(path)) << path.string();
    const auto material = FindFirstMaterial(prefab);
    ASSERT_TRUE(material) << path.string();
    const auto texture_slot = material->material_data.shade_material.pbr_base_color_texture;
    ASSERT_NE(texture_slot, 0) << path.string();
    const auto texture = material->GetTexture(texture_slot);
    ASSERT_TRUE(texture) << path.string();
    EXPECT_FALSE(texture->PeekLocalData().empty()) << path.string();
    EXPECT_TRUE(texture->srgb) << path.string();

    const auto gltf = ReadGltfRootNode(path);
    const auto expected = ReadGltfSamplerInfo(gltf, 0);
    const auto& sampler = texture->GetSamplerSettings();
    EXPECT_EQ(sampler.mag_filter, expected.mag_filter) << path.string();
    EXPECT_EQ(sampler.min_filter, expected.min_filter) << path.string();
    EXPECT_EQ(sampler.mipmap_mode, expected.mipmap_mode) << path.string();
    EXPECT_EQ(sampler.address_mode_u, expected.address_mode_u) << path.string();
    EXPECT_EQ(sampler.address_mode_v, expected.address_mode_v) << path.string();
    EXPECT_FLOAT_EQ(sampler.max_lod, expected.max_lod) << path.string();
  };

  for (const auto& path : paths) {
    validate_import(path);
  }

  const auto uri_root = std::filesystem::temp_directory_path() / "EvoEngine_GltfUriFallback_M9";
  std::filesystem::create_directories(uri_root);
  std::filesystem::copy_file(paths[0].parent_path() / "BoxTextured0.bin", uri_root / "BoxTextured0.bin",
                             std::filesystem::copy_options::overwrite_existing);
  std::filesystem::copy_file(paths[0].parent_path() / "CesiumLogoFlat.png", uri_root / "Cesium Logo Flat.png",
                             std::filesystem::copy_options::overwrite_existing);
  std::ifstream source_stream(paths[0]);
  std::string source((std::istreambuf_iterator<char>(source_stream)), std::istreambuf_iterator<char>());
  const auto replace = [&](const std::string& from, const std::string& to) {
    const auto position = source.find(from);
    ASSERT_NE(position, std::string::npos);
    source.replace(position, from.size(), to);
  };
  replace(R"("sampler": 0,
            "source": 0)",
          R"("sampler": 0,
            "source": 0,
            "extensions": {"MSFT_texture_dds": {"source": 1}})");
  replace(R"({
            "uri": "CesiumLogoFlat.png"
        })",
          R"({
            "uri": "Cesium%20Logo%20Flat.png"
        },
        {
            "uri": "Missing.dds"
        })");
  const auto uri_path = uri_root / "BoxTexturedUriFallback.gltf";
  std::ofstream(uri_path, std::ios::binary) << source;
  validate_import(uri_path);
  std::error_code cleanup_error;
  std::filesystem::remove_all(uri_root, cleanup_error);
}

TEST(GltfMaterialConversion, PrefabImportsAndPreservesDefaultMorphTargets) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  const auto path = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "Extern" / "3rdParty" / "assimp" / "assimp" /
                    "test" / "models" / "glTF2" / "SimpleMorph" / "glTF" / "SimpleMorph.gltf";
  ASSERT_TRUE(std::filesystem::exists(path));
  const auto prefab = AssetManager::CreateTemporaryAsset<Prefab>();
  ASSERT_TRUE(prefab->Import(path));
  const auto mesh = FindFirstMesh(prefab);
  ASSERT_TRUE(mesh);
  ASSERT_EQ(mesh->PeekMorphTargets().size(), 2);
  ASSERT_EQ(mesh->GetDefaultMorphWeights().size(), 2);
  EXPECT_FLOAT_EQ(mesh->GetDefaultMorphWeights()[0], 0.5f);
  EXPECT_FLOAT_EQ(mesh->GetDefaultMorphWeights()[1], 0.5f);
  EXPECT_TRUE(std::any_of(mesh->PeekVertices().begin(), mesh->PeekVertices().end(), [](const Vertex& vertex) {
    return glm::distance(vertex.position, glm::vec3(0.5f, 1.5f, 0.0f)) < kEpsilon;
  }));
  EXPECT_NEAR(mesh->GetBound().max.y, 1.5f, kEpsilon);

  const auto asymmetric = mesh->BuildMorphedVertices({1.0f, 0.0f});
  ASSERT_EQ(asymmetric.size(), mesh->PeekVertices().size());
  EXPECT_FALSE(std::equal(asymmetric.begin(), asymmetric.end(), mesh->PeekVertices().begin(),
                          [](const Vertex& lhs, const Vertex& rhs) {
                            return lhs.position == rhs.position;
                          }));
}

TEST(GltfMaterialConversion, PrefabImportsMorphNormalAndTangentStreams) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  const auto path = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "Extern" / "3rdParty" / "assimp" / "assimp" /
                    "test" / "models" / "glTF2" / "AnimatedMorphCube" / "glTF" / "AnimatedMorphCube.gltf";
  ASSERT_TRUE(std::filesystem::exists(path));
  const auto prefab = AssetManager::CreateTemporaryAsset<Prefab>();
  ASSERT_TRUE(prefab->Import(path));
  const auto mesh = FindFirstMesh(prefab);
  ASSERT_TRUE(mesh);
  ASSERT_EQ(mesh->PeekMorphTargets().size(), 2);
  for (const auto& target : mesh->PeekMorphTargets()) {
    EXPECT_EQ(target.position_deltas.size(), mesh->PeekVertices().size());
    EXPECT_EQ(target.normal_deltas.size(), mesh->PeekVertices().size());
    EXPECT_EQ(target.tangent_deltas.size(), mesh->PeekVertices().size());
  }
  EXPECT_TRUE(std::any_of(mesh->PeekMorphTargets()[1].normal_deltas.begin(),
                          mesh->PeekMorphTargets()[1].normal_deltas.end(), [](const glm::vec3& delta) {
                            return glm::dot(delta, delta) > kEpsilon * kEpsilon;
                          }));
  EXPECT_TRUE(std::all_of(mesh->PeekMorphTargets()[1].tangent_deltas.begin(),
                          mesh->PeekMorphTargets()[1].tangent_deltas.end(), [](const glm::vec3& delta) {
                            return glm::dot(delta, delta) <= kEpsilon * kEpsilon;
                          }));
  const auto morphed = mesh->BuildMorphedVertices({0.0f, 1.0f});
  ASSERT_EQ(morphed.size(), mesh->PeekVertices().size());
  bool normal_changed = false;
  for (size_t index = 0; index < morphed.size(); index++) {
    normal_changed |= glm::distance(morphed[index].normal, mesh->PeekVertices()[index].normal) > kEpsilon;
    EXPECT_NEAR(glm::distance(morphed[index].tangent, mesh->PeekVertices()[index].tangent), 0.0f, kEpsilon);
  }
  EXPECT_TRUE(normal_changed);
}

TEST(GltfMaterialConversion, MorphEvaluationUsesNeutralNormalBasisAndKeepsWeightsAligned) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  Vertex neutral{};
  neutral.position = glm::vec3(0.0f);
  neutral.normal = glm::vec3(1.0f, 0.0f, 0.0f);
  neutral.tangent = glm::vec3(0.0f, 0.0f, 1.0f);
  auto evaluated = neutral;
  evaluated.normal = glm::normalize(glm::vec3(1.0f, 0.5f, 0.0f));
  std::vector vertices = {evaluated, evaluated, evaluated};
  Mesh mesh;
  mesh.OnCreate();
  VertexAttributes attributes{};
  attributes.normal = true;
  attributes.tangent = true;
  mesh.SetVertices(attributes, vertices, {glm::uvec3(0, 1, 2)});

  MorphTarget invalid;
  invalid.name = "invalid";
  invalid.position_deltas = {glm::vec3(0.0f)};
  MorphTarget valid;
  valid.name = "valid";
  valid.normal_deltas = std::vector(3, glm::vec3(0.0f, 1.0f, 0.0f));
  mesh.SetMorphTargets({invalid, valid}, {0.25f, 0.5f}, std::vector(3, neutral));

  ASSERT_EQ(mesh.PeekMorphTargets().size(), 1);
  EXPECT_EQ(mesh.PeekMorphTargets()[0].name, "valid");
  ASSERT_EQ(mesh.GetDefaultMorphWeights().size(), 1);
  EXPECT_FLOAT_EQ(mesh.GetDefaultMorphWeights()[0], 0.5f);
  EXPECT_NEAR(glm::distance(mesh.BuildMorphedVertices({})[0].normal, evaluated.normal), 0.0f, kEpsilon);
  EXPECT_EQ(mesh.BuildMorphedVertices({0.0f})[0].normal, neutral.normal);
  EXPECT_NEAR(glm::distance(mesh.BuildMorphedVertices({1.0f})[0].normal, glm::normalize(glm::vec3(1.0f, 1.0f, 0.0f))),
              0.0f, kEpsilon);

  mesh.SetMorphTargets({invalid}, {0.25f}, std::vector(3, neutral));
  EXPECT_TRUE(mesh.PeekMorphTargets().empty());
  EXPECT_TRUE(mesh.GetDefaultMorphWeights().empty());
  EXPECT_TRUE(mesh.PeekMorphBaseVertices().empty());

  mesh.SetVertices(attributes, vertices, {glm::uvec3(0, 1, 2)});
  EXPECT_TRUE(mesh.PeekMorphTargets().empty());
  EXPECT_TRUE(mesh.GetDefaultMorphWeights().empty());
  EXPECT_TRUE(mesh.PeekMorphBaseVertices().empty());
}

TEST(GltfMaterialConversion, ProjectGltfTextureReuseAndManagedFailureFallback) {
  TempGltfProject project;
  const auto model_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "Extern" / "3rdParty" / "assimp" /
                          "assimp" / "test" / "models" / "glTF2" / "BoxTextured-glTF";
  std::filesystem::copy_file(model_root / "BoxTextured0.bin", project.ModelsPath() / "BoxTextured0.bin");
  std::filesystem::copy_file(model_root / "CesiumLogoFlat.png", project.ModelsPath() / "CesiumLogoFlat.png");
  WriteBc7UnormDds(project.ModelsPath() / "Texture.dds");

  const auto gltf_path = project.ModelsPath() / "Model.gltf";
  WriteGltfDdsVariant(model_root / "BoxTextured.gltf", gltf_path, "Texture.dds");
  WriteAssetMetadata(gltf_path, "Prefab", kProjectGltfHandle);
  WriteAssetMetadata(project.ModelsPath() / "Texture.dds", "Texture2D", kProjectDdsHandle);
  std::ofstream(project.ModelsPath() / "Broken.dds", std::ios::binary) << "invalid";
  const auto broken_gltf_path = project.ModelsPath() / "Broken.gltf";
  WriteGltfDdsVariant(model_root / "BoxTextured.gltf", broken_gltf_path, "Broken.dds");
  WriteAssetMetadata(broken_gltf_path, "Prefab", kProjectBrokenGltfHandle);
  WriteAssetMetadata(project.ModelsPath() / "Broken.dds", "Texture2D", kProjectBrokenDdsHandle);

  Application app;
  ApplicationContextScope scope(app);
  ASSERT_TRUE(app.PushLayer<RenderLayer>("Render Layer"));
  auto settings = EmptyProjectSettings();
  settings.allow_empty_project = false;
  settings.project_path = project.ProjectPath();
  settings.application_mode = ApplicationMode::Headless;
  settings.load_default_resources = true;
  settings.graphics_settings.use_mesh_shader = false;
  settings.graphics_settings.use_ray_tracing = false;
  app.Initialize(settings);

  const auto fallback_prefab =
      std::dynamic_pointer_cast<Prefab>(ProjectManager::GetOrCreateAsset(std::filesystem::path("Models/Broken.gltf")));
  ASSERT_TRUE(fallback_prefab);
  const auto fallback_material = FindFirstMaterial(fallback_prefab);
  ASSERT_TRUE(fallback_material);
  const auto fallback_texture =
      fallback_material->GetTexture(fallback_material->material_data.shade_material.pbr_base_color_texture);
  ASSERT_TRUE(fallback_texture);
  EXPECT_NE(fallback_texture->GetVkImageView(), VK_NULL_HANDLE);
  EXPECT_NE(fallback_texture->GetVkSampler(), VK_NULL_HANDLE);
  EXPECT_EQ(fallback_texture->RefTexture2DStorage().GetFormat(), VK_FORMAT_R8G8B8A8_SRGB);
  EXPECT_TRUE(fallback_texture->SamplesLinearSrgb());

  if (!Platform::GetSelectedPhysicalDevice()->features.textureCompressionBC) {
    GTEST_SKIP() << "BC texture compression is unavailable on the selected test device.";
  }

  const auto prefab =
      std::dynamic_pointer_cast<Prefab>(ProjectManager::GetOrCreateAsset(std::filesystem::path("Models/Model.gltf")));
  ASSERT_TRUE(prefab);
  const auto material = FindFirstMaterial(prefab);
  ASSERT_TRUE(material);
  const auto texture = material->GetTexture(material->material_data.shade_material.pbr_base_color_texture);
  const auto source_texture =
      std::dynamic_pointer_cast<Texture2D>(ProjectManager::GetOrCreateAsset("Models/Texture.dds"));
  ASSERT_TRUE(texture);
  ASSERT_TRUE(source_texture);
  EXPECT_NE(texture->GetTextureStorageIndex(), source_texture->GetTextureStorageIndex());
  EXPECT_EQ(texture->GetVkImage(), source_texture->GetVkImage());
  EXPECT_NE(texture->GetVkImageView(), VK_NULL_HANDLE);
  EXPECT_NE(texture->GetVkSampler(), VK_NULL_HANDLE);
  EXPECT_NE(texture->GetVkImageView(), source_texture->GetVkImageView());
  EXPECT_EQ(source_texture->RefTexture2DStorage().GetFormat(), VK_FORMAT_BC7_UNORM_BLOCK);
  EXPECT_EQ(texture->RefTexture2DStorage().GetFormat(), VK_FORMAT_BC7_SRGB_BLOCK);
  EXPECT_TRUE(texture->SamplesLinearSrgb());
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

  ASSERT_NE(material.pbr_diffuse_texture, 0);
  ASSERT_NE(material.pbr_specular_glossiness_texture, 0);
  EXPECT_EQ(materials[0].texture_infos[material.pbr_diffuse_texture].index, 202);
  EXPECT_EQ(materials[0].texture_infos[material.pbr_specular_glossiness_texture].index, 203);
  EXPECT_EQ(materials[0].texture_infos[material.pbr_specular_glossiness_texture].tex_coord, 1);
  EXPECT_EQ(materials[0].texture_infos[material.pbr_diffuse_texture].color_space,
            static_cast<int32_t>(GltfTextureColorSpace::Srgb));
  EXPECT_EQ(materials[0].texture_infos[material.pbr_specular_glossiness_texture].color_space,
            static_cast<int32_t>(GltfTextureColorSpace::Srgb));
}

TEST(GltfMaterialConversion, ForbiddenShadingModelExtensionCombinationsAreDiagnosedAndRejected) {
  const auto gltf = YAML::Load(R"({
    "materials": [{
      "extensions": {
        "KHR_materials_pbrSpecularGlossiness": {"specularFactor": [0.2, 0.3, 0.4]},
        "KHR_materials_specular": {"specularFactor": 0.25},
        "KHR_materials_transmission": {"transmissionFactor": 0.75, "transmissionTexture": {"index": 2}},
        "KHR_materials_clearcoat": {"clearcoatFactor": 0.8, "clearcoatNormalTexture": {"index": 3}},
        "KHR_materials_iridescence": {"iridescenceFactor": 0.6, "iridescenceTexture": {"index": 4}}
      }
    }, {
      "extensions": {
        "KHR_materials_unlit": {},
        "KHR_materials_pbrSpecularGlossiness": {"glossinessFactor": 0.2},
        "KHR_materials_specular": {"specularFactor": 0.1},
        "KHR_materials_transmission": {"transmissionFactor": 0.9},
        "KHR_materials_clearcoat": {"clearcoatFactor": 0.7},
        "KHR_materials_iridescence": {"iridescenceFactor": 0.5}
      }
    }]
  })");

  std::vector<std::string> diagnostics;
  const auto materials = BuildGltfMaterialDataFromGltfNode(
      gltf,
      [](const int32_t texture_index) {
        return texture_index;
      },
      {}, {},
      [&](const std::string& message) {
        diagnostics.push_back(message);
      });

  ASSERT_EQ(materials.size(), 2);
  EXPECT_EQ(materials[0].shade_material.pbr_model, static_cast<int32_t>(GltfPbrModel::SpecularGlossiness));
  ExpectVec3Near(materials[0].shade_material.pbr_specular_factor, glm::vec3(0.2f, 0.3f, 0.4f));
  EXPECT_FLOAT_EQ(materials[0].shade_material.specular_factor, 1.0f);
  EXPECT_FLOAT_EQ(materials[0].shade_material.transmission_factor, 0.0f);
  EXPECT_FLOAT_EQ(materials[0].shade_material.clearcoat_factor, 0.0f);
  EXPECT_FLOAT_EQ(materials[0].shade_material.iridescence_factor, 0.0f);
  EXPECT_EQ(materials[0].shade_material.transmission_texture, 0);
  EXPECT_EQ(materials[0].shade_material.clearcoat_normal_texture, 0);
  EXPECT_EQ(materials[0].shade_material.iridescence_texture, 0);

  EXPECT_EQ(materials[1].shade_material.unlit, 1);
  EXPECT_EQ(materials[1].shade_material.pbr_model, static_cast<int32_t>(GltfPbrModel::MetallicRoughness));
  EXPECT_FLOAT_EQ(materials[1].shade_material.transmission_factor, 0.0f);
  EXPECT_FLOAT_EQ(materials[1].shade_material.clearcoat_factor, 0.0f);
  EXPECT_FLOAT_EQ(materials[1].shade_material.iridescence_factor, 0.0f);

  ASSERT_EQ(diagnostics.size(), 9);
  std::string combined_diagnostics;
  for (const auto& diagnostic : diagnostics) {
    combined_diagnostics += diagnostic + '\n';
  }
  EXPECT_NE(combined_diagnostics.find("glTF material 0 rejects KHR_materials_specular"), std::string::npos);
  EXPECT_NE(combined_diagnostics.find("glTF material 1 rejects KHR_materials_pbrSpecularGlossiness"),
            std::string::npos);
  EXPECT_NE(combined_diagnostics.find("incompatible with KHR_materials_unlit"), std::string::npos);
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
        "KHR_materials_clearcoat": {
          "clearcoatFactor": 0.7,
          "clearcoatNormalTexture": {"index": 7, "texCoord": 1, "scale": 0.35}
        },
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
  EXPECT_FLOAT_EQ(material.clearcoat_factor, 0.7f);
  EXPECT_FLOAT_EQ(material.clearcoat_normal_texture_scale, 0.35f);
  EXPECT_FLOAT_EQ(material.retroreflection_factor, 1.0f);

  ASSERT_NE(material.iridescence_texture, 0);
  ASSERT_NE(material.iridescence_thickness_texture, 0);
  ASSERT_NE(material.anisotropy_texture, 0);
  ASSERT_NE(material.retroreflection_texture, 0);
  ASSERT_NE(material.clearcoat_normal_texture, 0);
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
  EXPECT_EQ(materials[0].texture_infos[material.clearcoat_normal_texture].index, 407);
  EXPECT_EQ(materials[0].texture_infos[material.clearcoat_normal_texture].tex_coord, 1);

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

TEST(GltfMaterialConversion, TextureSourceFlipCanDifferByColorSpace) {
  const auto gltf = YAML::Load(R"({
    "materials": [{
      "normalTexture": {"index": 2},
      "pbrMetallicRoughness": {"baseColorTexture": {"index": 2}}
    }]
  })");

  const auto materials = BuildGltfMaterialDataFromGltfNode(
      gltf,
      [](const int32_t texture_index, const bool) {
        return texture_index;
      },
      [](const int32_t texture_index, const bool srgb) {
        return texture_index == 2 && srgb;
      });

  ASSERT_EQ(materials.size(), 1);
  const auto& shade = materials[0].shade_material;
  const auto& base_color = materials[0].texture_infos[shade.pbr_base_color_texture].uv_transform;
  const auto& normal = materials[0].texture_infos[shade.normal_texture].uv_transform;
  EXPECT_NEAR(base_color[1][1], -1.0f, kEpsilon);
  EXPECT_NEAR(base_color[2][1], 1.0f, kEpsilon);
  EXPECT_NEAR(normal[1][1], 1.0f, kEpsilon);
  EXPECT_NEAR(normal[2][1], 0.0f, kEpsilon);
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
  mesh.SetVertices(attributes, vertices, {glm::uvec3(0, 1, 2)}, 1);

  ASSERT_EQ(mesh.PeekVertices().size(), 3);
  EXPECT_NEAR(mesh.PeekVertices()[0].tangent.x, 0.0f, kEpsilon);
  EXPECT_NEAR(mesh.PeekVertices()[0].tangent.y, 1.0f, kEpsilon);
  EXPECT_NEAR(mesh.PeekVertices()[0].tangent.z, 0.0f, kEpsilon);
  EXPECT_NEAR(mesh.PeekVertices()[0].vertex_info3, -1.0f, kEpsilon);
}

TEST(GltfMaterialConversion, MissingTangentsUseNormalTexturesFourthUvSet) {
  Application app;
  VertexAttributes attributes;
  attributes.normal = true;
  attributes.tex_coord = true;
  attributes.tex_coord_3 = true;

  std::vector<Vertex> vertices(3);
  vertices[0].position = glm::vec3(0.0f, 0.0f, 0.0f);
  vertices[1].position = glm::vec3(1.0f, 0.0f, 0.0f);
  vertices[2].position = glm::vec3(0.0f, 1.0f, 0.0f);
  for (auto& vertex : vertices) {
    vertex.normal = glm::vec3(0.0f, 0.0f, 1.0f);
    vertex.tex_coord = glm::vec2(0.0f);
  }
  vertices[0].tex_coord_3 = glm::vec2(0.0f, 0.0f);
  vertices[1].tex_coord_3 = glm::vec2(0.0f, 1.0f);
  vertices[2].tex_coord_3 = glm::vec2(1.0f, 0.0f);

  Mesh mesh;
  mesh.OnCreate();
  mesh.SetVertices(attributes, vertices, {glm::uvec3(0, 1, 2)}, 3);

  ASSERT_EQ(mesh.PeekVertices().size(), 3);
  EXPECT_NEAR(mesh.PeekVertices()[0].tangent.x, 0.0f, kEpsilon);
  EXPECT_NEAR(mesh.PeekVertices()[0].tangent.y, 1.0f, kEpsilon);
  EXPECT_NEAR(mesh.PeekVertices()[0].vertex_info3, -1.0f, kEpsilon);
}

TEST(GltfMaterialConversion, MikkTangentsSplitMirroredChartsAndPreserveSkinnedData) {
  Application app;
  VertexAttributes attributes;
  attributes.normal = true;
  attributes.tex_coord = true;

  std::vector<Vertex> vertices(4);
  vertices[0].position = glm::vec3(0.0f, 0.0f, 0.0f);
  vertices[1].position = glm::vec3(1.0f, 0.0f, 0.0f);
  vertices[2].position = glm::vec3(0.0f, 1.0f, 0.0f);
  vertices[3].position = glm::vec3(1.0f, 1.0f, 0.0f);
  for (auto& vertex : vertices) {
    vertex.normal = glm::vec3(0.0f, 0.0f, 1.0f);
  }
  vertices[0].tex_coord = glm::vec2(0.0f, 0.0f);
  vertices[1].tex_coord = glm::vec2(1.0f, 0.0f);
  vertices[2].tex_coord = glm::vec2(0.0f, 1.0f);
  vertices[3].tex_coord = glm::vec2(0.0f, 0.0f);
  const std::vector<glm::uvec3> triangles{{0, 1, 2}, {2, 1, 3}};

  Mesh mesh;
  mesh.OnCreate();
  std::vector<uint32_t> source_vertex_indices;
  mesh.SetVertices(attributes, vertices, triangles, 0, &source_vertex_indices);
  ASSERT_EQ(mesh.PeekVertices().size(), 6);
  ASSERT_EQ(source_vertex_indices.size(), mesh.PeekVertices().size());
  MorphTarget morph_target;
  morph_target.position_deltas = {glm::vec3(0.0f), glm::vec3(1.0f), glm::vec3(2.0f), glm::vec3(3.0f)};
  std::vector morph_targets = {morph_target};
  RemapMorphTargets(morph_targets, source_vertex_indices);
  ASSERT_EQ(morph_targets[0].position_deltas.size(), source_vertex_indices.size());
  for (size_t index = 0; index < source_vertex_indices.size(); index++) {
    EXPECT_EQ(morph_targets[0].position_deltas[index], morph_target.position_deltas[source_vertex_indices[index]]);
  }
  const auto& split_triangles = mesh.PeekTriangles();
  const float first_sign = mesh.PeekVertices()[split_triangles[0].x].vertex_info3;
  const float second_sign = mesh.PeekVertices()[split_triangles[1].x].vertex_info3;
  EXPECT_EQ(first_sign, mesh.PeekVertices()[split_triangles[0].y].vertex_info3);
  EXPECT_EQ(first_sign, mesh.PeekVertices()[split_triangles[0].z].vertex_info3);
  EXPECT_EQ(second_sign, mesh.PeekVertices()[split_triangles[1].y].vertex_info3);
  EXPECT_EQ(second_sign, mesh.PeekVertices()[split_triangles[1].z].vertex_info3);
  EXPECT_EQ(first_sign, -second_sign);

  SkinnedVertexAttributes skinned_attributes;
  skinned_attributes.normal = true;
  skinned_attributes.tex_coord = true;
  std::vector<SkinnedVertex> skinned_vertices(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    skinned_vertices[i].position = vertices[i].position;
    skinned_vertices[i].normal = vertices[i].normal;
    skinned_vertices[i].tex_coord = vertices[i].tex_coord;
    skinned_vertices[i].bond_id = glm::ivec4(7);
    skinned_vertices[i].weight = glm::vec4(0.25f);
    skinned_vertices[i].vertex_info4 = glm::vec2(3.0f, 4.0f);
  }
  SkinnedMesh skinned_mesh;
  skinned_mesh.OnCreate();
  skinned_mesh.SetVertices(skinned_attributes, skinned_vertices, triangles);
  ASSERT_EQ(skinned_mesh.PeekSkinnedVertices().size(), 6);
  for (const auto& vertex : skinned_mesh.PeekSkinnedVertices()) {
    EXPECT_EQ(vertex.bond_id, glm::ivec4(7));
    EXPECT_EQ(vertex.weight, glm::vec4(0.25f));
    EXPECT_EQ(vertex.vertex_info4, glm::vec2(3.0f, 4.0f));
  }

  VertexAttributes authored_attributes = attributes;
  authored_attributes.tangent = true;
  for (auto& vertex : vertices) {
    vertex.tangent = glm::vec3(1.0f, 0.0f, 0.0f);
  }
  Mesh mutable_mesh;
  mutable_mesh.OnCreate();
  mutable_mesh.SetVertices(authored_attributes, vertices, triangles);
  mutable_mesh.SetMorphTargets({morph_target}, {0.0f}, vertices);
  ASSERT_EQ(mutable_mesh.PeekMorphTargets().size(), 1);
  mutable_mesh.RecalculateTangent();
  EXPECT_TRUE(mutable_mesh.PeekMorphTargets().empty());
  EXPECT_TRUE(mutable_mesh.GetDefaultMorphWeights().empty());
  EXPECT_TRUE(mutable_mesh.PeekMorphBaseVertices().empty());
  EXPECT_EQ(mutable_mesh.BuildMorphedVertices({}).size(), mutable_mesh.PeekVertices().size());
  for (const auto& triangle : mutable_mesh.PeekTriangles()) {
    EXPECT_LT(triangle.x, mutable_mesh.PeekVertices().size());
    EXPECT_LT(triangle.y, mutable_mesh.PeekVertices().size());
    EXPECT_LT(triangle.z, mutable_mesh.PeekVertices().size());
  }

  SkinnedVertexAttributes authored_skinned_attributes = skinned_attributes;
  authored_skinned_attributes.tangent = true;
  for (auto& vertex : skinned_vertices) {
    vertex.tangent = glm::vec3(1.0f, 0.0f, 0.0f);
  }
  SkinnedMesh mutable_skinned_mesh;
  mutable_skinned_mesh.OnCreate();
  mutable_skinned_mesh.SetVertices(authored_skinned_attributes, skinned_vertices, triangles);
  mutable_skinned_mesh.SetMorphTargets({morph_target}, {0.0f}, skinned_vertices);
  ASSERT_EQ(mutable_skinned_mesh.PeekMorphTargets().size(), 1);
  mutable_skinned_mesh.RecalculateTangent();
  EXPECT_TRUE(mutable_skinned_mesh.PeekMorphTargets().empty());
  EXPECT_TRUE(mutable_skinned_mesh.GetDefaultMorphWeights().empty());
  EXPECT_TRUE(mutable_skinned_mesh.PeekMorphBaseVertices().empty());
  EXPECT_EQ(mutable_skinned_mesh.BuildMorphedVertices({}).size(), mutable_skinned_mesh.PeekSkinnedVertices().size());
  for (const auto& triangle : mutable_skinned_mesh.PeekTriangles()) {
    EXPECT_LT(triangle.x, mutable_skinned_mesh.PeekSkinnedVertices().size());
    EXPECT_LT(triangle.y, mutable_skinned_mesh.PeekSkinnedVertices().size());
    EXPECT_LT(triangle.z, mutable_skinned_mesh.PeekSkinnedVertices().size());
  }
}

TEST(GltfMaterialConversion, AuthoredTangentsRemainByteStableAndUnsplit) {
  Application app;
  VertexAttributes attributes;
  attributes.normal = true;
  attributes.tangent = true;
  attributes.tex_coord = true;
  std::vector<Vertex> vertices(3);
  vertices[0].position = glm::vec3(0.0f, 0.0f, 0.0f);
  vertices[1].position = glm::vec3(1.0f, 0.0f, 0.0f);
  vertices[2].position = glm::vec3(0.0f, 1.0f, 0.0f);
  for (size_t i = 0; i < vertices.size(); ++i) {
    vertices[i].normal = glm::vec3(0.0f, 0.0f, 1.0f);
    vertices[i].tangent = glm::vec3(0.25f + static_cast<float>(i), 0.5f, 0.75f);
    vertices[i].vertex_info3 = i == 1 ? -1.0f : 1.0f;
  }
  Mesh mesh;
  mesh.OnCreate();
  mesh.SetVertices(attributes, vertices, {glm::uvec3(0, 1, 2)}, 3);
  ASSERT_EQ(mesh.PeekVertices().size(), vertices.size());
  EXPECT_EQ(std::memcmp(mesh.PeekVertices().data(), vertices.data(), vertices.size() * sizeof(Vertex)), 0);
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
  source_shade.nested_priority = 3u;
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
  EXPECT_EQ(shade_material.nested_priority, 3u);
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
