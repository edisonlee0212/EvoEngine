#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Animation.hpp"
#include "AnimationPlayer.hpp"
#include "Animator.hpp"
#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "AssetManager.hpp"
#include "AssetRef.hpp"
#include "AssetThumbnailProvider.hpp"
#include "Camera.hpp"
#include "EnvironmentalMap.hpp"
#include "FileManager.hpp"
#include "GaussianSplat.hpp"
#include "GaussianSplatRenderer.hpp"
#include "IAsset.hpp"
#include "IPrivateComponent.hpp"
#include "ISerializable.hpp"
#include "ISystem.hpp"
#include "InspectorRegistry.hpp"
#include "Json.hpp"
#include "Lights.hpp"
#include "LodGroup.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "OffscreenPreviewRenderer.hpp"
#include "Particles.hpp"
#include "PlayerController.hpp"
#include "PointCloud.hpp"
#include "PointCloudScanner.hpp"
#include "PostProcessingStack.hpp"
#include "Prefab.hpp"
#include "ProceduralNoise.hpp"
#include "ProjectManager.hpp"
#include "RenderInstanceStorage.hpp"
#include "Scene.hpp"
#include "Serialization.hpp"
#include "Shader.hpp"
#include "SkinnedMesh.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "Strands.hpp"
#include "StrandsRenderer.hpp"
#include "Texture2D.hpp"
#include "UnknownPrivateComponent.hpp"
#include "WayPoints.hpp"

#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <thread>

using namespace evo_engine;

namespace {
class TestSerializable final : public ISerializable {
 public:
  mutable int serialize_count = 0;
  int deserialize_count = 0;
  int value = 0;

  void Serialize(YAML::Emitter& out) const {
    ++serialize_count;
    out << YAML::Key << "value" << YAML::Value << value;
  }

  void Deserialize(const YAML::Node& in) {
    ++deserialize_count;
    if (in["value"]) {
      value = in["value"].as<int>();
    }
  }
};

class TestAsset final : public IAsset {
 public:
  mutable int serialize_count = 0;
  int deserialize_count = 0;
  int collect_asset_ref_count = 0;
  int value = 0;

  void Serialize(YAML::Emitter& out) const {
    ++serialize_count;
    out << YAML::Key << "value" << YAML::Value << value;
  }

  void Deserialize(const YAML::Node& in) {
    ++deserialize_count;
    if (in["value"]) {
      value = in["value"].as<int>();
    }
  }

  void CollectAssetRef(std::vector<AssetRef>&) {
    ++collect_asset_ref_count;
  }
};

class TestAssetIoPayload final : public StagedAssetLoadPayload {
 public:
  int value = 0;
};

class TestAssetIo final : public IAsset {
 public:
  mutable int concrete_save_count = 0;
  int concrete_load_count = 0;
  mutable int concrete_supports_staged_loading_count = 0;
  mutable int concrete_load_staged_payload_count = 0;
  int concrete_apply_staged_payload_count = 0;
  int concrete_generate_thumbnail_count = 0;
  mutable int serialize_count = 0;
  int deserialize_count = 0;
  int value = 0;

  bool SaveConcrete(const std::filesystem::path& path) const {
    ++concrete_save_count;
    return path.filename() == "concrete.eveasset";
  }

  bool LoadConcrete(const std::filesystem::path& path) {
    ++concrete_load_count;
    return path.filename() == "concrete.eveasset";
  }

  [[nodiscard]] bool SupportsConcreteStagedLoading(const std::filesystem::path&) const {
    ++concrete_supports_staged_loading_count;
    return true;
  }

  [[nodiscard]] std::shared_ptr<StagedAssetLoadPayload> LoadConcreteStagedPayload(const std::filesystem::path&) const {
    ++concrete_load_staged_payload_count;
    auto payload = std::make_shared<TestAssetIoPayload>();
    payload->value = 41;
    return payload;
  }

  bool ApplyConcreteStagedPayload(const std::filesystem::path&,
                                  const std::shared_ptr<StagedAssetLoadPayload>& payload) {
    ++concrete_apply_staged_payload_count;
    const auto typed_payload = std::dynamic_pointer_cast<TestAssetIoPayload>(payload);
    if (!typed_payload) {
      return false;
    }
    value = typed_payload->value;
    return true;
  }

  std::shared_ptr<Texture2D> GenerateConcreteThumbnail() {
    ++concrete_generate_thumbnail_count;
    return {};
  }

  void Serialize(YAML::Emitter& out) const {
    ++serialize_count;
    out << YAML::Key << "value" << YAML::Value << value;
  }

  void Deserialize(const YAML::Node& in) {
    ++deserialize_count;
    if (in["value"]) {
      value = in["value"].as<int>();
    }
  }
};

void RegisterTestAssetIoSerializationHandler() {
  ASSERT_TRUE(Serialization::RegisterSerializationHandler<TestAssetIo>(
      [](YAML::Emitter& out, const TestAssetIo& asset) {
        asset.Serialize(out);
      },
      [](const YAML::Node& in, TestAssetIo& asset) {
        asset.Deserialize(in);
      },
      {}, "TestAssetIo"));
}

class TestPrivateComponent final : public IPrivateComponent {
 public:
  mutable int serialize_count = 0;
  int deserialize_count = 0;
  int collect_asset_ref_count = 0;
  int relink_count = 0;
  int value = 0;

  void Serialize(YAML::Emitter& out) const {
    ++serialize_count;
    out << YAML::Key << "value" << YAML::Value << value;
  }

  void Deserialize(const YAML::Node& in) {
    ++deserialize_count;
    if (in["value"]) {
      value = in["value"].as<int>();
    }
  }

  void CollectAssetRef(std::vector<AssetRef>&) {
    ++collect_asset_ref_count;
  }

  void Relink(const std::unordered_map<Handle, Handle>&, const std::shared_ptr<Scene>&) {
    ++relink_count;
  }
};

class TestSystem final : public ISystem {
 public:
  mutable int serialize_count = 0;
  int deserialize_count = 0;
  int collect_asset_ref_count = 0;
  int value = 0;

  void Serialize(YAML::Emitter& out) const {
    ++serialize_count;
    out << YAML::Key << "value" << YAML::Value << value;
  }

  void Deserialize(const YAML::Node& in) {
    ++deserialize_count;
    if (in["value"]) {
      value = in["value"].as<int>();
    }
  }

  void CollectAssetRef(std::vector<AssetRef>&) {
    ++collect_asset_ref_count;
  }
};

class OffsetBase {
 public:
  virtual ~OffsetBase() = default;

  int offset_base_value = 0;
};

class OffsetAsset final : public OffsetBase, public IAsset {
 public:
  int handler_value = 0;
  mutable int serialize_count = 0;
  int deserialize_count = 0;

  void Serialize(YAML::Emitter&) const {
    ++serialize_count;
  }

  void Deserialize(const YAML::Node&) {
    ++deserialize_count;
  }
};

class RoutedSceneComponent final : public IPrivateComponent {
 public:
  int value = 0;
};

class RoutedSceneSystem final : public ISystem {
 public:
  int value = 0;
};

class TestablePrefab final : public Prefab {
 public:
  [[nodiscard]] bool LoadFrom(const std::filesystem::path& path) {
    return LoadInternal(path);
  }

  [[nodiscard]] bool SaveTo(const std::filesystem::path& path) const {
    return SaveInternal(path);
  }
};

void RegisterTestablePrefabHandlers() {
  auto serialize = [](YAML::Emitter& out, const TestablePrefab& prefab) {
    out << YAML::Key << "in" << YAML::Value << prefab.instance_name;
    out << YAML::Key << "e" << YAML::Value << prefab.IsPrefabEnabled();
    out << YAML::Key << "eh" << YAML::Value << prefab.entity_handle.GetValue();

    if (!prefab.data_components.empty()) {
      out << YAML::Key << "dc" << YAML::BeginSeq;
      for (const auto& component : prefab.data_components) {
        out << YAML::BeginMap;
        component.Serialize(out);
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
    }

    if (!prefab.private_components.empty()) {
      out << YAML::Key << "pc" << YAML::BeginSeq;
      for (const auto& component : prefab.private_components) {
        out << YAML::BeginMap;
        component.Serialize(out);
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
    }

    if (!prefab.child_prefabs.empty()) {
      out << YAML::Key << "c" << YAML::BeginSeq;
      for (const auto& child : prefab.child_prefabs) {
        out << YAML::BeginMap;
        out << YAML::Key << "h" << child->GetHandle().GetValue();
        Serialization::SerializeObject(out, *child);
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
    }
  };

  auto deserialize = [](const YAML::Node& in, TestablePrefab& prefab) {
    prefab.instance_name = in["in"].as<std::string>();
    prefab.SetPrefabEnabled(in["e"].as<bool>());
    prefab.entity_handle = Handle(in["eh"].as<uint64_t>());
    if (in["dc"]) {
      for (const auto& i : in["dc"]) {
        DataComponentHolder holder;
        if (holder.Deserialize(i)) {
          prefab.data_components.push_back(holder);
        }
      }
    }
    std::vector<std::pair<int, std::shared_ptr<IAsset>>> local_assets;
    if (const auto in_local_assets = in["LocalAssets"]) {
      int index = 0;
      for (const auto& i : in_local_assets) {
        if (const auto type_name = i["TypeName"].as<std::string>(); Serialization::HasSerializableType(type_name)) {
          auto asset = AssetManager::CreateTemporaryAsset(type_name, Handle(i["Handle"].as<uint64_t>()));
          local_assets.emplace_back(index, asset);
        }
        index++;
      }

      for (const auto& i : local_assets) {
        Serialization::DeserializeObject(in_local_assets[i.first], *i.second);
      }
    }
    if (in["pc"]) {
      for (const auto& i : in["pc"]) {
        PrivateComponentHolder holder;
        holder.Deserialize(i);
        prefab.private_components.push_back(holder);
      }
    }

    if (in["c"]) {
      for (const auto& i : in["c"]) {
        auto child = std::dynamic_pointer_cast<Prefab>(
            AssetManager::CreateTemporaryAsset("Prefab", Handle(i["h"].as<uint64_t>())));
        if (!child) {
          continue;
        }
        Serialization::DeserializeObject(i, *child);
        prefab.child_prefabs.push_back(child);
      }
    }
  };

  Serialization::RegisterSerializationHandler<TestablePrefab>(serialize, deserialize, {}, "TestablePrefab");
}

class TempProject {
 public:
  TempProject() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::temp_directory_path() / ("EvoEngineSerializationRegistryTest_" + std::to_string(now));
    std::filesystem::create_directories(AssetsPath());
  }

  ~TempProject() {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] std::filesystem::path ProjectPath() const {
    return root_ / "SerializationRegistryTest.eveproj";
  }

  [[nodiscard]] std::filesystem::path AssetsPath() const {
    return root_ / "Assets";
  }

  [[nodiscard]] std::filesystem::path RootPath() const {
    return root_;
  }

 private:
  std::filesystem::path root_;
};

ApplicationInitializationSettings EmptyProjectSettings() {
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  return settings;
}

ApplicationInitializationSettings ProjectSettings(const TempProject& project) {
  ApplicationInitializationSettings settings = EmptyProjectSettings();
  settings.allow_empty_project = false;
  settings.project_path = project.ProjectPath();
  settings.load_project_assets = true;
  return settings;
}

std::shared_ptr<Texture2D> CreateSinglePixelTexture(const glm::vec4& color) {
  auto texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  texture->SetRgbaChannelData({color}, glm::uvec2(1, 1));
  return texture;
}

void BeginMap(YAML::Emitter& out) {
  out << YAML::BeginMap;
}

void WriteFloat(std::ofstream& stream, const float value) {
  stream.write(reinterpret_cast<const char*>(&value), sizeof(float));
}

std::filesystem::path WriteGaussianSplatPlyFixture(const std::filesystem::path& directory, const bool include_rest) {
  const auto path = directory / (include_rest ? "GaussianSplatFixtureWithRest.ply" : "GaussianSplatFixture.ply");
  std::ofstream stream(path, std::ios::binary);
  stream << "ply\n";
  stream << "format binary_little_endian 1.0\n";
  stream << "element vertex 2\n";
  const std::vector<std::string> properties = {"x",      "y",      "z",       "scale_0", "scale_1", "scale_2", "f_dc_0",
                                               "f_dc_1", "f_dc_2", "opacity", "rot_0",   "rot_1",   "rot_2",   "rot_3"};
  for (const auto& property : properties) {
    stream << "property float " << property << "\n";
  }
  if (include_rest) {
    stream << "property float f_rest_0\n";
    stream << "property float f_rest_1\n";
    stream << "property float f_rest_2\n";
  }
  stream << "end_header\n";

  const std::array<std::vector<float>, 2> rows = {
      std::vector<float>{-1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 1.0f, 0.0f, 0.0f, 0.0f, 0.01f,
                         0.02f, 0.03f},
      std::vector<float>{4.0f, -5.0f, 6.0f, 0.8f, 0.9f, 1.0f, 1.1f, 1.2f, 1.3f, 1.4f, 0.0f, 1.0f, 0.0f, 0.0f, 0.04f,
                         0.05f, 0.06f}};
  for (const auto& row : rows) {
    const auto value_count = include_rest ? row.size() : properties.size();
    for (size_t i = 0; i < value_count; ++i) {
      WriteFloat(stream, row[i]);
    }
  }
  return path;
}

std::filesystem::path WriteInvalidGaussianSplatPlyFixture(const std::filesystem::path& directory) {
  const auto path = directory / "InvalidGaussianSplatFixture.ply";
  std::ofstream stream(path, std::ios::binary);
  stream << "ply\n";
  stream << "format binary_little_endian 1.0\n";
  stream << "element vertex 1\n";
  stream << "property float x\n";
  stream << "property float y\n";
  stream << "property float z\n";
  stream << "end_header\n";
  WriteFloat(stream, 0.0f);
  WriteFloat(stream, 0.0f);
  WriteFloat(stream, 0.0f);
  return path;
}

GaussianSplat CreateGaussianSplatFixture() {
  GaussianSplat gaussian_splat;
  gaussian_splat.positions = {glm::vec3(-1.0f, 2.0f, 3.0f), glm::vec3(4.0f, -5.0f, 6.0f)};
  gaussian_splat.scales = {glm::vec3(0.1f, 0.2f, 0.3f), glm::vec3(0.8f, 0.9f, 1.0f)};
  gaussian_splat.rotations = {glm::vec4(1.0f, 0.0f, 0.0f, 0.0f), glm::vec4(0.0f, 1.0f, 0.0f, 0.0f)};
  gaussian_splat.opacities = {1.25f, 0.25f};
  gaussian_splat.colors = {glm::vec3(1.2f, -0.2f, 0.5f), glm::vec3(0.25f, 0.5f, 0.75f)};
  gaussian_splat.spherical_harmonics_rest = {0.01f, 0.02f, 0.03f, 0.04f};
  gaussian_splat.spherical_harmonics_rest_float_count = 2;
  gaussian_splat.RecalculateBoundingBox();
  return gaussian_splat;
}

std::vector<uint8_t> ReadBinaryFile(const std::filesystem::path& path) {
  std::ifstream stream(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

std::vector<Handle> ReadMaterialTextureHandles(const std::string& saved_text) {
  std::vector<Handle> texture_handles;
  std::istringstream stream(saved_text);
  std::string line;
  bool reading_texture_ref = false;
  while (std::getline(stream, line)) {
    if (line.find("texture:") != std::string::npos) {
      reading_texture_ref = true;
      continue;
    }
    if (!reading_texture_ref) {
      continue;
    }
    const auto handle_key = line.find("asset_handle_:");
    if (handle_key == std::string::npos) {
      continue;
    }
    const auto handle_value = std::stoull(line.substr(handle_key + std::string("asset_handle_:").size()));
    if (handle_value != 0) {
      texture_handles.emplace_back(handle_value);
    }
    reading_texture_ref = false;
  }
  return texture_handles;
}
}  // namespace

TEST(SerializationRegistry, MissingSerializationHandlersDoNotCallConcreteSerialization) {
  Application app;
  ApplicationContextScope scope(app);

  TestSerializable serializable;
  serializable.value = 3;
  YAML::Emitter serializable_out;
  BeginMap(serializable_out);
  Serialization::SerializeObject(serializable_out, serializable);
  serializable_out << YAML::EndMap;
  EXPECT_EQ(serializable.serialize_count, 0);
  Serialization::DeserializeObject(YAML::Load("{value: 7}"), serializable);
  EXPECT_EQ(serializable.deserialize_count, 0);
  EXPECT_EQ(serializable.value, 3);

  TestAsset asset;
  asset.value = 4;
  YAML::Emitter asset_out;
  BeginMap(asset_out);
  Serialization::SerializeObject(asset_out, static_cast<IAsset&>(asset));
  asset_out << YAML::EndMap;
  EXPECT_EQ(asset.serialize_count, 0);
  Serialization::DeserializeObject(YAML::Load("{value: 8}"), static_cast<IAsset&>(asset));
  EXPECT_EQ(asset.deserialize_count, 0);
  EXPECT_EQ(asset.value, 4);

  TestPrivateComponent component;
  component.value = 5;
  YAML::Emitter component_out;
  BeginMap(component_out);
  Serialization::SerializeObject(component_out, static_cast<IPrivateComponent&>(component));
  component_out << YAML::EndMap;
  EXPECT_EQ(component.serialize_count, 0);
  Serialization::DeserializeObject(YAML::Load("{value: 9}"), static_cast<IPrivateComponent&>(component));
  EXPECT_EQ(component.deserialize_count, 0);
  EXPECT_EQ(component.value, 5);

  TestSystem system;
  system.value = 6;
  YAML::Emitter system_out;
  BeginMap(system_out);
  Serialization::SerializeObject(system_out, static_cast<ISystem&>(system));
  system_out << YAML::EndMap;
  EXPECT_EQ(system.serialize_count, 0);
  Serialization::DeserializeObject(YAML::Load("{value: 10}"), static_cast<ISystem&>(system));
  EXPECT_EQ(system.deserialize_count, 0);
  EXPECT_EQ(system.value, 6);
}

TEST(SerializationRegistry, RegisteredTypesDoNotInstallDefaultSerializationHandlers) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  app.RegisterAsset<TestAsset>("TestAsset", {".evetestasset"});
  app.RegisterPrivateComponent<TestPrivateComponent>("TestPrivateComponent");
  app.RegisterSystem<TestSystem>("TestSystem");

  EXPECT_EQ(Serialization::FindSerializationHandler(typeid(TestAsset).hash_code()), nullptr);
  EXPECT_EQ(Serialization::FindSerializationHandler(typeid(TestPrivateComponent).hash_code()), nullptr);
  EXPECT_EQ(Serialization::FindSerializationHandler(typeid(TestSystem).hash_code()), nullptr);

  TestAsset asset;
  asset.value = 8;
  YAML::Emitter asset_out;
  BeginMap(asset_out);
  Serialization::SerializeObject(asset_out, static_cast<IAsset&>(asset));
  asset_out << YAML::EndMap;
  EXPECT_EQ(asset.serialize_count, 0);
  Serialization::DeserializeObject(YAML::Load("{value: 18}"), static_cast<IAsset&>(asset));
  EXPECT_EQ(asset.deserialize_count, 0);
  EXPECT_EQ(asset.value, 8);

  TestPrivateComponent component;
  component.value = 9;
  YAML::Emitter component_out;
  BeginMap(component_out);
  Serialization::SerializeObject(component_out, static_cast<IPrivateComponent&>(component));
  component_out << YAML::EndMap;
  EXPECT_EQ(component.serialize_count, 0);
  Serialization::DeserializeObject(YAML::Load("{value: 19}"), static_cast<IPrivateComponent&>(component));
  EXPECT_EQ(component.deserialize_count, 0);
  EXPECT_EQ(component.value, 9);

  TestSystem system;
  system.value = 10;
  YAML::Emitter system_out;
  BeginMap(system_out);
  Serialization::SerializeObject(system_out, static_cast<ISystem&>(system));
  system_out << YAML::EndMap;
  EXPECT_EQ(system.serialize_count, 0);
  Serialization::DeserializeObject(YAML::Load("{value: 20}"), static_cast<ISystem&>(system));
  EXPECT_EQ(system.deserialize_count, 0);
  EXPECT_EQ(system.value, 10);
}

TEST(SerializationRegistry, BuiltInAnimationAndPostProcessingTypesInstallSerializationHandlers) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());

  ASSERT_NE(Serialization::FindSerializationHandler(typeid(AssetRef).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(PrivateComponentRef).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(AnimationPlayer).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(Camera).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(Animator).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(MeshRenderer).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(GaussianSplatRenderer).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(StrandsRenderer).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(SkinnedMeshRenderer).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(Particles).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(SpotLight).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(PointLight).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(DirectionalLight).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(PointCloudScanner).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(PlayerController).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(LodGroup).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(UnknownPrivateComponent).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(UnknownAsset).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(UnknownSystem).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(PostProcessingStack).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(Material).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(EnvironmentalMap).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(Shader).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(procedural_noise::ProceduralNoise2D).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(procedural_noise::ProceduralNoise3D).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(procedural_noise::ProceduralNoise4D).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(PointCloud).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(GaussianSplat).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(Texture2D).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(Animation).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(ParticleInfoList).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(Mesh).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(SkinnedMesh).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(Strands).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(Prefab).hash_code()), nullptr);
  ASSERT_NE(Serialization::FindSerializationHandler(typeid(Scene).hash_code()), nullptr);

  AnimationPlayer player;
  player.auto_play = false;
  player.auto_play_speed = 12.5f;

  YAML::Emitter player_out;
  BeginMap(player_out);
  Serialization::SerializeObject(player_out, static_cast<IPrivateComponent&>(player));
  player_out << YAML::EndMap;

  const auto player_node = YAML::Load(player_out.c_str());
  EXPECT_FALSE(player_node["auto_play"].as<bool>());
  EXPECT_FLOAT_EQ(player_node["auto_play_speed"].as<float>(), 12.5f);

  AnimationPlayer restored_player;
  Serialization::DeserializeObject(YAML::Load("{auto_play: false, auto_play_speed: 4.25}"),
                                   static_cast<IPrivateComponent&>(restored_player));
  EXPECT_FALSE(restored_player.auto_play);
  EXPECT_FLOAT_EQ(restored_player.auto_play_speed, 4.25f);

  Camera camera;
  camera.Resize({1280, 720});
  camera.camera_settings.fov = 72.0f;
  camera.camera_settings.gamma = 2.0f;
  YAML::Emitter camera_out;
  BeginMap(camera_out);
  Serialization::SerializeObject(camera_out, static_cast<IPrivateComponent&>(camera));
  camera_out << YAML::EndMap;
  const auto camera_node = YAML::Load(camera_out.c_str());
  EXPECT_EQ(camera_node["x"].as<uint32_t>(), 1280);
  EXPECT_EQ(camera_node["y"].as<uint32_t>(), 720);
  EXPECT_FLOAT_EQ(camera_node["fov"].as<float>(), 72.0f);

  Camera restored_camera;
  restored_camera.SetRendered();
  restored_camera.SetRequireRendering(true);
  Serialization::DeserializeObject(camera_node, static_cast<IPrivateComponent&>(restored_camera));
  EXPECT_EQ(restored_camera.GetSize().x, 1280);
  EXPECT_EQ(restored_camera.GetSize().y, 720);
  EXPECT_FLOAT_EQ(restored_camera.camera_settings.gamma, 2.0f);
  EXPECT_FALSE(restored_camera.Rendered());

  Animator animator;
  animator.RefTransformChain() = {glm::mat4(1.0f)};
  animator.RefOffsetMatrices() = {glm::mat4(2.0f)};
  animator.RefBoneNames() = {"Root"};
  YAML::Emitter animator_out;
  BeginMap(animator_out);
  Serialization::SerializeObject(animator_out, static_cast<IPrivateComponent&>(animator));
  animator_out << YAML::EndMap;
  const auto animator_node = YAML::Load(animator_out.c_str());
  EXPECT_TRUE(animator_node["transform_chain_"]);
  ASSERT_EQ(animator_node["names_"].size(), 1);
  EXPECT_EQ(animator_node["names_"][0]["Name"].as<std::string>(), "Root");

  Animator restored_animator;
  Serialization::DeserializeObject(animator_node, static_cast<IPrivateComponent&>(restored_animator));
  EXPECT_EQ(restored_animator.PeekTransformChain().size(), 1);
  EXPECT_EQ(restored_animator.PeekOffsetMatrices().size(), 1);
  ASSERT_EQ(restored_animator.PeekBoneNames().size(), 1);
  EXPECT_EQ(restored_animator.PeekBoneNames()[0], "Root");

  EnvironmentalMap environmental_map;
  YAML::Emitter environmental_map_out;
  BeginMap(environmental_map_out);
  Serialization::SerializeObject(environmental_map_out, static_cast<IAsset&>(environmental_map));
  environmental_map_out << YAML::EndMap;
  const auto environmental_map_node = YAML::Load(environmental_map_out.c_str());
  EXPECT_TRUE(environmental_map_node["light_probe"]);
  EXPECT_TRUE(environmental_map_node["reflection_probe"]);
  EXPECT_TRUE(environmental_map_node["environment_pdf_texture"]);

  EnvironmentalMap restored_environmental_map;
  Serialization::DeserializeObject(YAML::Load("{}"), static_cast<IAsset&>(restored_environmental_map));
  EXPECT_EQ(restored_environmental_map.light_probe.GetAssetHandle().GetValue(), 0);
  EXPECT_EQ(restored_environmental_map.reflection_probe.GetAssetHandle().GetValue(), 0);
  EXPECT_EQ(restored_environmental_map.environment_pdf_texture.GetAssetHandle().GetValue(), 0);

  Prefab prefab;
  prefab.instance_name = "Parent";
  prefab.SetPrefabEnabled(false);
  prefab.entity_handle = Handle(123);
  auto child_prefab = AssetManager::CreateTemporaryAsset<Prefab>();
  child_prefab->instance_name = "Child";
  child_prefab->entity_handle = Handle(456);
  const auto child_handle = child_prefab->GetHandle();
  prefab.child_prefabs.push_back(child_prefab);
  YAML::Emitter prefab_out;
  BeginMap(prefab_out);
  Serialization::SerializeObject(prefab_out, static_cast<IAsset&>(prefab));
  prefab_out << YAML::EndMap;
  const auto prefab_node = YAML::Load(prefab_out.c_str());
  EXPECT_EQ(prefab_node["in"].as<std::string>(), "Parent");
  EXPECT_FALSE(prefab_node["e"].as<bool>());
  ASSERT_EQ(prefab_node["c"].size(), 1);
  EXPECT_EQ(prefab_node["c"][0]["h"].as<uint64_t>(), child_handle.GetValue());

  Prefab restored_prefab;
  Serialization::DeserializeObject(prefab_node, static_cast<IAsset&>(restored_prefab));
  EXPECT_EQ(restored_prefab.instance_name, "Parent");
  EXPECT_FALSE(restored_prefab.IsPrefabEnabled());
  EXPECT_EQ(restored_prefab.entity_handle.GetValue(), 123);
  ASSERT_EQ(restored_prefab.child_prefabs.size(), 1);
  EXPECT_EQ(restored_prefab.child_prefabs[0]->instance_name, "Child");
  EXPECT_EQ(restored_prefab.child_prefabs[0]->entity_handle.GetValue(), 456);
  EXPECT_EQ(restored_prefab.child_prefabs[0]->GetHandle().GetValue(), child_handle.GetValue());

  PostProcessingStack stack;
  stack.enable_screen_space_ambient_occlusion = false;
  stack.enable_bloom = false;
  stack.enable_screen_space_reflection = true;
  stack.enable_tone_mapping = false;
  stack.screen_space_ambient_occlusion = std::make_shared<ScreenSpaceAmbientOcclusion>();
  stack.screen_space_ambient_occlusion->kernel_size = 16;
  stack.screen_space_ambient_occlusion->radius = 0.35f;
  stack.bloom = std::make_shared<Bloom>();
  stack.bloom->filter_radius = 0.02f;
  stack.bloom->bloom_chain_length = 4;
  stack.screen_space_reflection = std::make_shared<ScreenSpaceReflection>();
  stack.screen_space_reflection->max_iteration_count = 96;
  stack.screen_space_reflection->blur = false;
  stack.tone_mapping = std::make_shared<ToneMapping>();
  stack.tone_mapping->method = ToneMapping::ToneMapMethod::Filmic;
  stack.tone_mapping->exposure = 1.5f;
  stack.tone_mapping->brightness = 2.2f;
  stack.tone_mapping->contrast = 1.1f;
  stack.tone_mapping->saturation = 0.9f;
  stack.tone_mapping->auto_exposure = true;

  YAML::Emitter stack_out;
  BeginMap(stack_out);
  Serialization::SerializeObject(stack_out, static_cast<IAsset&>(stack));
  stack_out << YAML::EndMap;

  const auto stack_node = YAML::Load(stack_out.c_str());
  EXPECT_FALSE(stack_node["enable_screen_space_ambient_occlusion"].as<bool>());
  EXPECT_FALSE(stack_node["enable_bloom"].as<bool>());
  EXPECT_TRUE(stack_node["enable_screen_space_reflection"].as<bool>());
  EXPECT_FALSE(stack_node["enable_tone_mapping"].as<bool>());
  EXPECT_EQ(stack_node["screen_space_ambient_occlusion"]["kernel_size"].as<int>(), 16);
  EXPECT_FLOAT_EQ(stack_node["bloom"]["filter_radius"].as<float>(), 0.02f);
  EXPECT_EQ(stack_node["screen_space_reflection"]["max_iteration_count"].as<int>(), 96);
  EXPECT_EQ(stack_node["tone_mapping"]["method"].as<int>(), static_cast<int>(ToneMapping::ToneMapMethod::Filmic));
  EXPECT_FLOAT_EQ(stack_node["tone_mapping"]["brightness"].as<float>(), 2.2f);
  EXPECT_FLOAT_EQ(stack_node["tone_mapping"]["contrast"].as<float>(), 1.1f);
  EXPECT_FLOAT_EQ(stack_node["tone_mapping"]["saturation"].as<float>(), 0.9f);
  EXPECT_TRUE(stack_node["tone_mapping"]["auto_exposure"].as<bool>());

  PostProcessingStack restored_stack;
  Serialization::DeserializeObject(YAML::Load(R"(
enable_screen_space_ambient_occlusion: false
enable_bloom: true
enable_screen_space_reflection: false
enable_tone_mapping: true
screen_space_ambient_occlusion:
  avoid_distance: 3.5
  kernel_size: 24
  radius: 0.45
  bias: 0.04
  factor: 1.25
  intensity: 2.5
bloom:
  filter_radius: 0.03
  bloom_chain_length: 5
screen_space_reflection:
  max_distance: 50.0
  distance_confidence: 0.6
  max_iteration_count: 72
  initial_steps: 12
  thickness: 0.75
  blur: false
tone_mapping:
  method: 0
  exposure: 1.75
  brightness: 2.4
  contrast: 1.2
  saturation: 0.8
  vignette: 0.1
  auto_exposure: true
  auto_exposure_speed: 4.0
  ev_min_value: -6.0
  ev_max_value: 9.0
  enable_center_metering: true
  center_metering_size: 0.4
  average_mode: 0
  dither: false
)"),
                                   static_cast<IAsset&>(restored_stack));
  EXPECT_FALSE(restored_stack.enable_screen_space_ambient_occlusion);
  EXPECT_TRUE(restored_stack.enable_bloom);
  EXPECT_FALSE(restored_stack.enable_screen_space_reflection);
  EXPECT_TRUE(restored_stack.enable_tone_mapping);
  ASSERT_TRUE(restored_stack.screen_space_ambient_occlusion);
  EXPECT_FLOAT_EQ(restored_stack.screen_space_ambient_occlusion->avoid_distance, 3.5f);
  EXPECT_EQ(restored_stack.screen_space_ambient_occlusion->kernel_size, 24);
  ASSERT_TRUE(restored_stack.bloom);
  EXPECT_FLOAT_EQ(restored_stack.bloom->filter_radius, 0.03f);
  EXPECT_EQ(restored_stack.bloom->bloom_chain_length, 5);
  ASSERT_TRUE(restored_stack.screen_space_reflection);
  EXPECT_FALSE(restored_stack.screen_space_reflection->blur);
  EXPECT_EQ(restored_stack.screen_space_reflection->initial_steps, 12);
  ASSERT_TRUE(restored_stack.tone_mapping);
  EXPECT_EQ(restored_stack.tone_mapping->method, ToneMapping::ToneMapMethod::Filmic);
  EXPECT_FLOAT_EQ(restored_stack.tone_mapping->exposure, 1.75f);
  EXPECT_FLOAT_EQ(restored_stack.tone_mapping->brightness, 2.4f);
  EXPECT_FLOAT_EQ(restored_stack.tone_mapping->contrast, 1.2f);
  EXPECT_FLOAT_EQ(restored_stack.tone_mapping->saturation, 0.8f);
  EXPECT_FLOAT_EQ(restored_stack.tone_mapping->vignette, 0.1f);
  EXPECT_TRUE(restored_stack.tone_mapping->auto_exposure);
  EXPECT_FLOAT_EQ(restored_stack.tone_mapping->auto_exposure_speed, 4.0f);
  EXPECT_FLOAT_EQ(restored_stack.tone_mapping->ev_min_value, -6.0f);
  EXPECT_FLOAT_EQ(restored_stack.tone_mapping->ev_max_value, 9.0f);
  EXPECT_TRUE(restored_stack.tone_mapping->enable_center_metering);
  EXPECT_FLOAT_EQ(restored_stack.tone_mapping->center_metering_size, 0.4f);
  EXPECT_EQ(restored_stack.tone_mapping->average_mode, 0);
  EXPECT_FALSE(restored_stack.tone_mapping->dither);

  Shader shader;
  shader.RefShaderCode() = "void main() {}";
  shader.RefShaderType() = static_cast<unsigned>(ShaderType::Fragment);
  YAML::Emitter shader_out;
  BeginMap(shader_out);
  Serialization::SerializeObject(shader_out, static_cast<IAsset&>(shader));
  shader_out << YAML::EndMap;
  const auto shader_node = YAML::Load(shader_out.c_str());
  EXPECT_EQ(shader_node["shader_code"].as<std::string>(), "void main() {}");
  EXPECT_EQ(shader_node["shader_type"].as<unsigned>(), static_cast<unsigned>(ShaderType::Fragment));

  Shader restored_shader;
  Serialization::DeserializeObject(YAML::Load("{shader_code: 'void compute() {}', shader_type: 7}"),
                                   static_cast<IAsset&>(restored_shader));
  EXPECT_EQ(restored_shader.PeekShaderCode(), "void compute() {}");
  EXPECT_EQ(restored_shader.RefShaderType(), 7);

  procedural_noise::ProceduralNoise2D noise;
  YAML::Emitter noise_out;
  BeginMap(noise_out);
  Serialization::SerializeObject(noise_out, static_cast<IAsset&>(noise));
  noise_out << YAML::EndMap;
  const auto noise_node = YAML::Load(noise_out.c_str());
  EXPECT_TRUE(noise_node["node_graph"]);
  procedural_noise::ProceduralNoise2D restored_noise;
  Serialization::DeserializeObject(noise_node, static_cast<IAsset&>(restored_noise));

  PointCloud point_cloud;
  point_cloud.offset = glm::dvec3(1.0, 2.0, 3.0);
  point_cloud.point_size = 0.25f;
  point_cloud.compress_factor = 0.5f;
  point_cloud.positions = {glm::dvec3(-1.0, 2.0, 3.0), glm::dvec3(4.0, 5.0, 6.0)};
  point_cloud.RecalculateBoundingBox();
  YAML::Emitter point_cloud_out;
  BeginMap(point_cloud_out);
  Serialization::SerializeObject(point_cloud_out, static_cast<IAsset&>(point_cloud));
  point_cloud_out << YAML::EndMap;
  const auto point_cloud_node = YAML::Load(point_cloud_out.c_str());
  EXPECT_FLOAT_EQ(point_cloud_node["point_size"].as<float>(), 0.25f);
  EXPECT_EQ(point_cloud_node["positions"].as<YAML::Binary>().size(), 2 * sizeof(glm::dvec3));

  PointCloud restored_point_cloud;
  Serialization::DeserializeObject(point_cloud_node, static_cast<IAsset&>(restored_point_cloud));
  EXPECT_EQ(restored_point_cloud.positions.size(), 2);
  EXPECT_DOUBLE_EQ(restored_point_cloud.GetMinBound().x, -1.0);
  EXPECT_DOUBLE_EQ(restored_point_cloud.GetMaxBound().z, 6.0);

  GaussianSplat gaussian_splat;
  gaussian_splat.positions = {glm::vec3(-1.0f, 2.0f, 3.0f), glm::vec3(4.0f, 5.0f, 6.0f)};
  gaussian_splat.scales = {glm::vec3(0.1f, 0.2f, 0.3f), glm::vec3(0.4f, 0.5f, 0.6f)};
  gaussian_splat.rotations = {glm::vec4(1.0f, 0.0f, 0.0f, 0.0f), glm::vec4(0.0f, 1.0f, 0.0f, 0.0f)};
  gaussian_splat.opacities = {0.7f, 0.8f};
  gaussian_splat.colors = {glm::vec3(0.9f, 1.0f, 1.1f), glm::vec3(1.2f, 1.3f, 1.4f)};
  gaussian_splat.spherical_harmonics_rest = {0.01f, 0.02f, 0.03f, 0.04f};
  gaussian_splat.spherical_harmonics_rest_float_count = 2;
  gaussian_splat.RecalculateBoundingBox();
  YAML::Emitter gaussian_splat_out;
  BeginMap(gaussian_splat_out);
  Serialization::SerializeObject(gaussian_splat_out, static_cast<IAsset&>(gaussian_splat));
  gaussian_splat_out << YAML::EndMap;
  const auto gaussian_splat_node = YAML::Load(gaussian_splat_out.c_str());
  EXPECT_EQ(gaussian_splat_node["positions"].as<YAML::Binary>().size(), 2 * sizeof(glm::vec3));
  EXPECT_EQ(gaussian_splat_node["spherical_harmonics_rest_float_count"].as<uint32_t>(), 2u);

  GaussianSplat restored_gaussian_splat;
  Serialization::DeserializeObject(gaussian_splat_node, static_cast<IAsset&>(restored_gaussian_splat));
  EXPECT_EQ(restored_gaussian_splat.GetSplatCount(), 2);
  EXPECT_FLOAT_EQ(restored_gaussian_splat.GetMinBound().x, -1.0f);
  EXPECT_FLOAT_EQ(restored_gaussian_splat.GetMaxBound().z, 6.0f);
  EXPECT_FLOAT_EQ(restored_gaussian_splat.scales[1].y, 0.5f);
  EXPECT_FLOAT_EQ(restored_gaussian_splat.opacities[1], 0.8f);
  EXPECT_EQ(restored_gaussian_splat.spherical_harmonics_rest.size(), 4);

  Texture2D texture;
  texture.SetRgbaChannelData({glm::vec4(1.0f, 0.5f, 0.25f, 1.0f)}, glm::uvec2(1, 1));
  YAML::Emitter texture_out;
  BeginMap(texture_out);
  Serialization::SerializeObject(texture_out, static_cast<IAsset&>(texture));
  texture_out << YAML::EndMap;
  const auto texture_node = YAML::Load(texture_out.c_str());
  EXPECT_EQ(texture_node["resolution"].as<glm::uvec2>().x, 1);
  EXPECT_EQ(texture_node["resolution"].as<glm::uvec2>().y, 1);
  EXPECT_TRUE(texture_node["pixels"]);

  Texture2D restored_texture;
  Serialization::DeserializeObject(texture_node, static_cast<IAsset&>(restored_texture));
  EXPECT_EQ(restored_texture.GetResolution().x, 1);
  EXPECT_EQ(restored_texture.GetResolution().y, 1);
  ASSERT_EQ(restored_texture.PeekLocalData().size(), 1);
  EXPECT_GT(restored_texture.PeekLocalData()[0].r, 0.99f);

  Animation animation;
  animation.bone_size = 1;
  animation.animation_length["Idle"] = 2.5f;
  animation.root_bone = std::make_shared<Bone>();
  animation.root_bone->name = "Root";
  animation.root_bone->index = 0;
  YAML::Emitter animation_out;
  BeginMap(animation_out);
  Serialization::SerializeObject(animation_out, static_cast<IAsset&>(animation));
  animation_out << YAML::EndMap;
  const auto animation_node = YAML::Load(animation_out.c_str());
  EXPECT_EQ(animation_node["bone_size"].as<size_t>(), 1);
  EXPECT_EQ(animation_node["animation_length"][0]["Name"].as<std::string>(), "Idle");

  Animation restored_animation;
  Serialization::DeserializeObject(animation_node, static_cast<IAsset&>(restored_animation));
  EXPECT_EQ(restored_animation.bone_size, 1);
  EXPECT_FLOAT_EQ(restored_animation.animation_length["Idle"], 2.5f);
  ASSERT_TRUE(restored_animation.root_bone);
  EXPECT_EQ(restored_animation.root_bone->name, "Root");

  const auto particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  ParticleInfo particle_info;
  particle_info.instance_color = glm::vec4(0.25f, 0.5f, 0.75f, 1.0f);
  particle_info_list->SetParticleInfos({particle_info});
  YAML::Emitter particle_info_list_out;
  BeginMap(particle_info_list_out);
  Serialization::SerializeObject(particle_info_list_out, static_cast<IAsset&>(*particle_info_list));
  particle_info_list_out << YAML::EndMap;
  const auto particle_info_list_node = YAML::Load(particle_info_list_out.c_str());
  EXPECT_TRUE(particle_info_list_node["particle_infos"]);

  const auto restored_particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  Serialization::DeserializeObject(particle_info_list_node, static_cast<IAsset&>(*restored_particle_info_list));
  ASSERT_EQ(restored_particle_info_list->PeekParticleInfoList().size(), 1);
  EXPECT_FLOAT_EQ(restored_particle_info_list->PeekParticleInfoList()[0].instance_color.z, 0.75f);

  VertexAttributes vertex_attributes;
  vertex_attributes.normal = true;
  vertex_attributes.tangent = true;
  vertex_attributes.tex_coord = true;
  vertex_attributes.color = true;
  std::vector<Vertex> vertices(3);
  vertices[0].position = glm::vec3(0.0f, 0.0f, 0.0f);
  vertices[1].position = glm::vec3(1.0f, 0.0f, 0.0f);
  vertices[2].position = glm::vec3(0.0f, 1.0f, 0.0f);
  const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
  mesh->SetVertices(vertex_attributes, vertices, {glm::uvec3(0, 1, 2)});
  YAML::Emitter mesh_out;
  BeginMap(mesh_out);
  Serialization::SerializeObject(mesh_out, static_cast<IAsset&>(*mesh));
  mesh_out << YAML::EndMap;
  const auto mesh_node = YAML::Load(mesh_out.c_str());
  EXPECT_TRUE(mesh_node["vertices_"]);
  EXPECT_TRUE(mesh_node["triangles_"]);

  const auto restored_mesh = AssetManager::CreateTemporaryAsset<Mesh>();
  Serialization::DeserializeObject(mesh_node, static_cast<IAsset&>(*restored_mesh));
  EXPECT_EQ(restored_mesh->PeekVertices().size(), 3);
  EXPECT_EQ(restored_mesh->PeekTriangles().size(), 1);
  EXPECT_FLOAT_EQ(restored_mesh->PeekVertices()[1].position.x, 1.0f);

  SkinnedVertexAttributes skinned_vertex_attributes;
  skinned_vertex_attributes.normal = true;
  skinned_vertex_attributes.tangent = true;
  skinned_vertex_attributes.tex_coord = true;
  skinned_vertex_attributes.color = true;
  std::vector<SkinnedVertex> skinned_vertices(3);
  skinned_vertices[0].position = glm::vec3(0.0f, 0.0f, 0.0f);
  skinned_vertices[1].position = glm::vec3(1.0f, 0.0f, 0.0f);
  skinned_vertices[2].position = glm::vec3(0.0f, 1.0f, 0.0f);
  const auto skinned_mesh = AssetManager::CreateTemporaryAsset<SkinnedMesh>();
  skinned_mesh->bone_animator_indices = {2, 5};
  skinned_mesh->SetVertices(skinned_vertex_attributes, skinned_vertices, {glm::uvec3(0, 1, 2)});
  YAML::Emitter skinned_mesh_out;
  BeginMap(skinned_mesh_out);
  Serialization::SerializeObject(skinned_mesh_out, static_cast<IAsset&>(*skinned_mesh));
  skinned_mesh_out << YAML::EndMap;
  const auto skinned_mesh_node = YAML::Load(skinned_mesh_out.c_str());
  EXPECT_TRUE(skinned_mesh_node["bone_animator_indices"]);
  EXPECT_TRUE(skinned_mesh_node["skinned_vertices_"]);

  const auto restored_skinned_mesh = AssetManager::CreateTemporaryAsset<SkinnedMesh>();
  Serialization::DeserializeObject(skinned_mesh_node, static_cast<IAsset&>(*restored_skinned_mesh));
  EXPECT_EQ(restored_skinned_mesh->bone_animator_indices.size(), 2);
  EXPECT_EQ(restored_skinned_mesh->PeekSkinnedVertices().size(), 3);
  EXPECT_EQ(restored_skinned_mesh->PeekTriangles().size(), 1);
  EXPECT_EQ(restored_skinned_mesh->bone_animator_indices[1], 5);

  StrandPointAttributes strand_point_attributes;
  strand_point_attributes.normal = true;
  strand_point_attributes.tex_coord = true;
  strand_point_attributes.color = true;
  std::vector<StrandPoint> strand_points(4);
  strand_points[0].position = glm::vec3(0.0f, 0.0f, 0.0f);
  strand_points[1].position = glm::vec3(0.25f, 0.0f, 0.0f);
  strand_points[2].position = glm::vec3(0.75f, 0.0f, 0.0f);
  strand_points[3].position = glm::vec3(1.0f, 0.0f, 0.0f);
  const auto strands = AssetManager::CreateTemporaryAsset<Strands>();
  strands->SetSegments(strand_point_attributes, {0}, strand_points);
  YAML::Emitter strands_out;
  BeginMap(strands_out);
  Serialization::SerializeObject(strands_out, static_cast<IAsset&>(*strands));
  strands_out << YAML::EndMap;
  const auto strands_node = YAML::Load(strands_out.c_str());
  EXPECT_TRUE(strands_node["segment_raw_indices_"]);
  EXPECT_TRUE(strands_node["strand_points_"]);

  const auto restored_strands = AssetManager::CreateTemporaryAsset<Strands>();
  Serialization::DeserializeObject(strands_node, static_cast<IAsset&>(*restored_strands));
  EXPECT_EQ(restored_strands->PeekSegments().size(), 1);
  EXPECT_EQ(restored_strands->PeekStrandPoints().size(), 4);
  EXPECT_FLOAT_EQ(restored_strands->PeekStrandPoints()[3].position.x, 1.0f);

  SkinnedMeshRenderer skinned_mesh_renderer;
  skinned_mesh_renderer.cast_shadow = false;
  skinned_mesh_renderer.SetRagDollState(true);
  skinned_mesh_renderer.rag_doll_freeze = true;
  skinned_mesh_renderer.RefRagDollTransformChain() = {glm::mat4(3.0f)};
  YAML::Emitter skinned_mesh_renderer_out;
  BeginMap(skinned_mesh_renderer_out);
  Serialization::SerializeObject(skinned_mesh_renderer_out, static_cast<IPrivateComponent&>(skinned_mesh_renderer));
  skinned_mesh_renderer_out << YAML::EndMap;
  const auto skinned_mesh_renderer_node = YAML::Load(skinned_mesh_renderer_out.c_str());
  EXPECT_FALSE(skinned_mesh_renderer_node["cast_shadow"].as<bool>());
  EXPECT_TRUE(skinned_mesh_renderer_node["rag_doll_"].as<bool>());
  EXPECT_TRUE(skinned_mesh_renderer_node["rag_doll_transform_chain_"]);

  SkinnedMeshRenderer restored_skinned_mesh_renderer;
  Serialization::DeserializeObject(skinned_mesh_renderer_node,
                                   static_cast<IPrivateComponent&>(restored_skinned_mesh_renderer));
  EXPECT_FALSE(restored_skinned_mesh_renderer.cast_shadow);
  EXPECT_TRUE(restored_skinned_mesh_renderer.RagDoll());
  EXPECT_TRUE(restored_skinned_mesh_renderer.rag_doll_freeze);
  EXPECT_EQ(restored_skinned_mesh_renderer.PeekRagDollTransformChain().size(), 1);
}

TEST(SerializationRegistry, GaussianSplatLoadsStandardPlyFields) {
  Application app;
  ApplicationContextScope scope(app);
  TempProject project;
  const auto path = WriteGaussianSplatPlyFixture(project.RootPath(), true);

  GaussianSplat gaussian_splat;
  ASSERT_TRUE(gaussian_splat.LoadPly(path));
  ASSERT_EQ(gaussian_splat.GetSplatCount(), 2);
  EXPECT_FLOAT_EQ(gaussian_splat.positions[0].x, -1.0f);
  EXPECT_FLOAT_EQ(gaussian_splat.positions[1].y, -5.0f);
  EXPECT_FLOAT_EQ(gaussian_splat.scales[0].z, 0.3f);
  EXPECT_FLOAT_EQ(gaussian_splat.colors[1].x, 1.1f);
  EXPECT_FLOAT_EQ(gaussian_splat.opacities[1], 1.4f);
  EXPECT_FLOAT_EQ(gaussian_splat.rotations[0].x, 1.0f);
  EXPECT_EQ(gaussian_splat.spherical_harmonics_rest_float_count, 3u);
  ASSERT_EQ(gaussian_splat.spherical_harmonics_rest.size(), 6);
  EXPECT_FLOAT_EQ(gaussian_splat.spherical_harmonics_rest[4], 0.05f);
  EXPECT_FLOAT_EQ(gaussian_splat.GetMinBound().y, -5.0f);
  EXPECT_FLOAT_EQ(gaussian_splat.GetMaxBound().z, 6.0f);
}

TEST(SerializationRegistry, GaussianSplatRejectsPlyWithoutRequiredFields) {
  Application app;
  ApplicationContextScope scope(app);
  TempProject project;
  const auto path = WriteInvalidGaussianSplatPlyFixture(project.RootPath());

  GaussianSplat gaussian_splat;
  EXPECT_FALSE(gaussian_splat.LoadPly(path));
  EXPECT_TRUE(gaussian_splat.Empty());
}

TEST(SerializationRegistry, GaussianSplatSavesPlyRoundTripPreservesRestFields) {
  Application app;
  ApplicationContextScope scope(app);
  TempProject project;
  auto gaussian_splat = CreateGaussianSplatFixture();
  const auto path = project.RootPath() / "roundtrip.ply";

  ASSERT_TRUE(gaussian_splat.SavePly(path));
  GaussianSplat restored;
  ASSERT_TRUE(restored.LoadPly(path));
  ASSERT_EQ(restored.GetSplatCount(), 2);
  EXPECT_FLOAT_EQ(restored.positions[1].z, 6.0f);
  EXPECT_FLOAT_EQ(restored.scales[0].y, 0.2f);
  EXPECT_FLOAT_EQ(restored.colors[0].x, 1.2f);
  EXPECT_FLOAT_EQ(restored.opacities[0], 1.25f);
  EXPECT_FLOAT_EQ(restored.rotations[1].y, 1.0f);
  EXPECT_EQ(restored.spherical_harmonics_rest_float_count, 2u);
  ASSERT_EQ(restored.spherical_harmonics_rest.size(), 4);
  EXPECT_FLOAT_EQ(restored.spherical_harmonics_rest[3], 0.04f);
}

TEST(SerializationRegistry, GaussianSplatSavesAndLoadsStandardSplatRows) {
  Application app;
  ApplicationContextScope scope(app);
  TempProject project;
  auto gaussian_splat = CreateGaussianSplatFixture();
  const auto path = project.RootPath() / "fixture.splat";

  testing::internal::CaptureStderr();
  ASSERT_TRUE(gaussian_splat.SaveSplat(path));
  const auto warning = testing::internal::GetCapturedStderr();
  EXPECT_NE(warning.find("drops spherical_harmonics_rest"), std::string::npos);

  const auto bytes = ReadBinaryFile(path);
  ASSERT_EQ(bytes.size(), 64);
  float first_x = 0.0f;
  float first_scale_z = 0.0f;
  std::memcpy(&first_x, bytes.data(), sizeof(float));
  std::memcpy(&first_scale_z, bytes.data() + 20, sizeof(float));
  EXPECT_FLOAT_EQ(first_x, -1.0f);
  EXPECT_FLOAT_EQ(first_scale_z, 0.3f);
  EXPECT_EQ(bytes[24], 255);
  EXPECT_EQ(bytes[25], 0);
  EXPECT_EQ(bytes[26], 128);
  EXPECT_EQ(bytes[27], 255);
  EXPECT_EQ(bytes[28], 255);
  EXPECT_EQ(bytes[29], 128);
  EXPECT_EQ(bytes[30], 128);
  EXPECT_EQ(bytes[31], 128);

  GaussianSplat restored;
  ASSERT_TRUE(restored.LoadSplat(path));
  ASSERT_EQ(restored.GetSplatCount(), 2);
  EXPECT_FLOAT_EQ(restored.positions[0].x, -1.0f);
  EXPECT_FLOAT_EQ(restored.scales[1].z, 1.0f);
  EXPECT_FLOAT_EQ(restored.colors[0].x, 1.0f);
  EXPECT_FLOAT_EQ(restored.colors[0].y, 0.0f);
  EXPECT_NEAR(restored.colors[0].z, 128.0f / 255.0f, 0.0001f);
  EXPECT_FLOAT_EQ(restored.opacities[0], 1.0f);
  EXPECT_EQ(restored.spherical_harmonics_rest_float_count, 0u);
  EXPECT_TRUE(restored.spherical_harmonics_rest.empty());
}

TEST(SerializationRegistry, GaussianSplatRejectsInvalidSplatByteSize) {
  Application app;
  ApplicationContextScope scope(app);
  TempProject project;
  const auto path = project.RootPath() / "invalid.splat";
  std::ofstream stream(path, std::ios::binary);
  stream << "not a splat";
  stream.close();

  GaussianSplat gaussian_splat;
  EXPECT_FALSE(gaussian_splat.LoadSplat(path));
}

TEST(SerializationRegistry, GaussianSplatSavesAndLoadsUncompressedKSplat) {
  Application app;
  ApplicationContextScope scope(app);
  TempProject project;
  auto gaussian_splat = CreateGaussianSplatFixture();
  const auto path = project.RootPath() / "fixture.ksplat";

  testing::internal::CaptureStderr();
  ASSERT_TRUE(gaussian_splat.SaveKSplat(path));
  const auto warning = testing::internal::GetCapturedStderr();
  EXPECT_NE(warning.find("drops spherical_harmonics_rest"), std::string::npos);
  EXPECT_EQ(std::filesystem::file_size(path), 4096u + 1024u + 88u);

  GaussianSplat restored;
  ASSERT_TRUE(restored.LoadKSplat(path));
  ASSERT_EQ(restored.GetSplatCount(), 2);
  EXPECT_FLOAT_EQ(restored.positions[0].x, -1.0f);
  EXPECT_FLOAT_EQ(restored.scales[1].z, 1.0f);
  EXPECT_FLOAT_EQ(restored.rotations[1].y, 1.0f);
  EXPECT_FLOAT_EQ(restored.colors[0].x, 1.0f);
  EXPECT_FLOAT_EQ(restored.colors[0].y, 0.0f);
  EXPECT_NEAR(restored.colors[0].z, 128.0f / 255.0f, 0.0001f);
  EXPECT_FLOAT_EQ(restored.opacities[0], 1.0f);
  EXPECT_EQ(restored.spherical_harmonics_rest_float_count, 0u);
  EXPECT_TRUE(restored.spherical_harmonics_rest.empty());
}

TEST(SerializationRegistry, GaussianSplatRejectsUnsupportedKSplatVersionsAndCompression) {
  Application app;
  ApplicationContextScope scope(app);
  TempProject project;
  auto gaussian_splat = CreateGaussianSplatFixture();
  const auto version_path = project.RootPath() / "unsupported-version.ksplat";
  const auto compression_path = project.RootPath() / "unsupported-compression.ksplat";
  ASSERT_TRUE(gaussian_splat.SaveKSplat(version_path));
  std::filesystem::copy_file(version_path, compression_path);

  {
    std::fstream stream(version_path, std::ios::binary | std::ios::in | std::ios::out);
    const uint8_t version = 1;
    stream.write(reinterpret_cast<const char*>(&version), sizeof(version));
  }
  {
    std::fstream stream(compression_path, std::ios::binary | std::ios::in | std::ios::out);
    const uint16_t compression_level = 1;
    stream.seekp(20);
    stream.write(reinterpret_cast<const char*>(&compression_level), sizeof(compression_level));
  }

  GaussianSplat restored;
  EXPECT_FALSE(restored.LoadKSplat(version_path));
  EXPECT_FALSE(restored.LoadKSplat(compression_path));
}

TEST(SerializationRegistry, GaussianSplatRegisteredExtensionsIncludeInterchangeFormats) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());

  const auto& extensions = Serialization::PeekAssetExtensions("GaussianSplat");
  EXPECT_NE(std::find(extensions.begin(), extensions.end(), ".evegaussiansplat"), extensions.end());
  EXPECT_NE(std::find(extensions.begin(), extensions.end(), ".ply"), extensions.end());
  EXPECT_NE(std::find(extensions.begin(), extensions.end(), ".splat"), extensions.end());
  EXPECT_NE(std::find(extensions.begin(), extensions.end(), ".ksplat"), extensions.end());
}

TEST(SerializationRegistry, GaussianSplatBuildsGpuDataWithoutRepackingStaticAsset) {
  Application app;
  ApplicationContextScope scope(app);

  GaussianSplat gaussian_splat;
  gaussian_splat.positions = {glm::vec3(-1.0f, 2.0f, 3.0f), glm::vec3(4.0f, -5.0f, 6.0f)};
  gaussian_splat.scales = {glm::vec3(0.1f, 0.2f, 0.3f), glm::vec3(0.4f, 0.5f, 0.6f)};
  gaussian_splat.rotations = {glm::vec4(1.0f, 0.0f, 0.0f, 0.0f), glm::vec4(0.0f, 1.0f, 0.0f, 0.0f)};
  gaussian_splat.opacities = {0.7f, 0.8f};
  gaussian_splat.colors = {glm::vec3(0.9f, 1.0f, 1.1f), glm::vec3(1.2f, 1.3f, 1.4f)};
  gaussian_splat.spherical_harmonics_rest = {0.01f, 0.02f, 0.03f, 0.04f};
  gaussian_splat.spherical_harmonics_rest_float_count = 2;
  gaussian_splat.RecalculateBoundingBox();

  const auto& gpu_data = gaussian_splat.EnsureGpuData();
  ASSERT_EQ(gpu_data.size(), 2);
  EXPECT_FLOAT_EQ(gpu_data[0].position_opacity.x, -1.0f);
  EXPECT_FLOAT_EQ(gpu_data[0].position_opacity.w, 0.7f);
  EXPECT_FLOAT_EQ(gpu_data[1].scale_reserved.y, 0.5f);
  EXPECT_FLOAT_EQ(gpu_data[1].rotation.y, 1.0f);
  EXPECT_FLOAT_EQ(gpu_data[1].color_rest_offset.z, 1.4f);
  EXPECT_FLOAT_EQ(gpu_data[1].color_rest_offset.w, 2.0f);

  const auto revision = gaussian_splat.GetGpuDataRevision();
  (void)gaussian_splat.EnsureGpuData();
  EXPECT_EQ(gaussian_splat.GetGpuDataRevision(), revision);
  EXPECT_EQ(gaussian_splat.GetGpuDataBuffer(), nullptr);

  gaussian_splat.positions[0].z = 9.0f;
  gaussian_splat.RecalculateBoundingBox();
  const auto& updated_gpu_data = gaussian_splat.EnsureGpuData();
  EXPECT_GT(gaussian_splat.GetGpuDataRevision(), revision);
  EXPECT_FLOAT_EQ(updated_gpu_data[0].position_opacity.z, 9.0f);
}

TEST(SerializationRegistry, GaussianSplatReportsAvailableSphericalHarmonicsDegree) {
  Application app;
  ApplicationContextScope scope(app);

  GaussianSplat gaussian_splat;
  gaussian_splat.positions.resize(2);
  EXPECT_EQ(gaussian_splat.GetSphericalHarmonicsRestFloatCount(), 0u);
  EXPECT_EQ(gaussian_splat.GetSphericalHarmonicsDegree(), 0u);

  gaussian_splat.spherical_harmonics_rest_float_count = 9;
  gaussian_splat.spherical_harmonics_rest.assign(18, 0.0f);
  EXPECT_EQ(gaussian_splat.GetSphericalHarmonicsRestFloatCount(), 9u);
  EXPECT_EQ(gaussian_splat.GetSphericalHarmonicsDegree(), 1u);

  gaussian_splat.spherical_harmonics_rest_float_count = 24;
  gaussian_splat.spherical_harmonics_rest.assign(48, 0.0f);
  EXPECT_EQ(gaussian_splat.GetSphericalHarmonicsDegree(), 2u);

  gaussian_splat.spherical_harmonics_rest_float_count = 45;
  gaussian_splat.spherical_harmonics_rest.assign(90, 0.0f);
  EXPECT_EQ(gaussian_splat.GetSphericalHarmonicsDegree(), 3u);

  gaussian_splat.spherical_harmonics_rest.resize(10);
  EXPECT_EQ(gaussian_splat.GetSphericalHarmonicsDegree(), 0u);
}

TEST(SerializationRegistry, GaussianSplatSortCacheRefreshesForCameraTransform) {
  Application app;
  ApplicationContextScope scope(app);

  GaussianSplat gaussian_splat;
  gaussian_splat.positions = {glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, 0.0f, -5.0f), glm::vec3(0.0f, 0.0f, -3.0f)};
  gaussian_splat.RecalculateBoundingBox();

  const auto& cache = gaussian_splat.EnsureSortedIndices(Handle(101), Handle(301), glm::mat4(1.0f), glm::mat4(1.0f));
  EXPECT_EQ(cache.indices, (std::vector<uint32_t>{1, 2, 0}));
  ASSERT_EQ(cache.depths.size(), 3);
  EXPECT_FLOAT_EQ(cache.depths[0], 5.0f);
  EXPECT_FLOAT_EQ(cache.depths[2], 1.0f);
  const auto generation = cache.generation;

  const auto& unchanged_cache =
      gaussian_splat.EnsureSortedIndices(Handle(101), Handle(301), glm::mat4(1.0f), glm::mat4(1.0f));
  EXPECT_EQ(unchanged_cache.generation, generation);

  const auto rotated_model = glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
  const auto wait_for_sort = [&](const Handle camera_handle, const Handle renderer_handle, const glm::mat4& model,
                                 const std::vector<uint32_t>& expected_indices) {
    for (size_t attempt = 0; attempt < 200; ++attempt) {
      const auto& current = gaussian_splat.EnsureSortedIndices(camera_handle, renderer_handle, model, glm::mat4(1.0f));
      if (current.indices == expected_indices) {
        return current.generation;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const auto& current = gaussian_splat.EnsureSortedIndices(camera_handle, renderer_handle, model, glm::mat4(1.0f));
    EXPECT_EQ(current.indices, expected_indices);
    return current.generation;
  };

  const auto& stale_rotated_cache =
      gaussian_splat.EnsureSortedIndices(Handle(101), Handle(301), rotated_model, glm::mat4(1.0f));
  EXPECT_EQ(stale_rotated_cache.indices, (std::vector<uint32_t>{1, 2, 0}));
  EXPECT_EQ(stale_rotated_cache.generation, generation);
  const auto rotated_generation = wait_for_sort(Handle(101), Handle(301), rotated_model, {0, 2, 1});
  EXPECT_GT(rotated_generation, generation);

  const auto& other_camera_cache =
      gaussian_splat.EnsureSortedIndices(Handle(202), Handle(301), glm::mat4(1.0f), glm::mat4(1.0f));
  EXPECT_EQ(other_camera_cache.indices, (std::vector<uint32_t>{1, 2, 0}));
  EXPECT_EQ(other_camera_cache.generation, 1);

  const auto& shared_camera_other_renderer =
      gaussian_splat.EnsureSortedIndices(Handle(101), Handle(302), glm::mat4(1.0f), glm::mat4(1.0f));
  EXPECT_EQ(shared_camera_other_renderer.indices, (std::vector<uint32_t>{1, 2, 0}));
  EXPECT_EQ(shared_camera_other_renderer.generation, 1);

  const auto& restored_first_renderer =
      gaussian_splat.EnsureSortedIndices(Handle(101), Handle(301), rotated_model, glm::mat4(1.0f));
  EXPECT_EQ(restored_first_renderer.indices, (std::vector<uint32_t>{0, 2, 1}));
  EXPECT_EQ(restored_first_renderer.generation, rotated_generation);
}

TEST(SerializationRegistry, GaussianSplatRendererPreservesSettingsAndAssetRef) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());

  const auto gaussian_splat = AssetManager::CreateTemporaryAsset<GaussianSplat>();
  ASSERT_TRUE(gaussian_splat);

  GaussianSplatRenderer renderer;
  renderer.gaussian_splat.Set<GaussianSplat>(gaussian_splat);
  renderer.opacity_scale = 0.5f;
  renderer.sh_degree = 2;
  renderer.sort_mode = GaussianSplatSortMode::GpuRadix;
  renderer.depth_mode = GaussianSplatDepthMode::Always;
  renderer.raster_mode = GaussianSplatRasterMode::Mesh;

  YAML::Emitter out;
  BeginMap(out);
  Serialization::SerializeObject(out, static_cast<IPrivateComponent&>(renderer));
  out << YAML::EndMap;
  const auto node = YAML::Load(out.c_str());

  GaussianSplatRenderer restored;
  Serialization::DeserializeObject(node, static_cast<IPrivateComponent&>(restored));
  EXPECT_EQ(restored.gaussian_splat.GetAssetHandle(), gaussian_splat->GetHandle());
  EXPECT_FLOAT_EQ(restored.opacity_scale, 0.5f);
  EXPECT_EQ(restored.sh_degree, 2);
  EXPECT_EQ(restored.sort_mode, GaussianSplatSortMode::GpuRadix);
  EXPECT_EQ(restored.depth_mode, GaussianSplatDepthMode::Always);
  EXPECT_EQ(restored.raster_mode, GaussianSplatRasterMode::Mesh);

  std::vector<AssetRef> asset_refs;
  Serialization::CollectAssetRefs(static_cast<IPrivateComponent&>(renderer), asset_refs);
  ASSERT_EQ(asset_refs.size(), 1);
  EXPECT_EQ(asset_refs.front().GetAssetHandle(), gaussian_splat->GetHandle());
}

TEST(SerializationRegistry, GaussianSplatRendererSceneCollectionSkipsInvalidRenderers) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());

  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  ASSERT_TRUE(scene);
  app.Attach(scene);
  for (const auto& camera_entity : scene->GetPrivateComponentOwnersList<Camera>()) {
    const auto camera = scene->GetOrSetPrivateComponent<Camera>(camera_entity).lock();
    ASSERT_TRUE(camera);
    camera->SetEnabled(false);
  }

  const auto gaussian_splat = AssetManager::CreateTemporaryAsset<GaussianSplat>();
  ASSERT_TRUE(gaussian_splat);
  gaussian_splat->positions = {glm::vec3(-1.0f, 2.0f, 3.0f), glm::vec3(4.0f, -5.0f, 6.0f)};
  gaussian_splat->RecalculateBoundingBox();

  const auto entity = scene->CreateEntity("Gaussian");
  const auto renderer = scene->GetOrSetPrivateComponent<GaussianSplatRenderer>(entity).lock();
  ASSERT_TRUE(renderer);
  renderer->gaussian_splat.Set<GaussianSplat>(gaussian_splat);

  RenderInstanceStorage storage;
  Bound world_bound;
  storage.BuildFromScene({}, scene, world_bound, false);
  ASSERT_EQ(storage.GetInstanceInfoBlocks().size(), 1);
  EXPECT_EQ(storage.GetRenderInstanceIndex(renderer->GetHandle()), 0);
  EXPECT_EQ(storage.GetInstanceRendererHandle(0), renderer->GetHandle());
  EXPECT_EQ(storage.GetInstanceEntityHandle(0), scene->GetEntityHandle(entity));

  renderer->SetEnabled(false);
  storage.Clear();
  storage.BuildFromScene({}, scene, world_bound, false);
  EXPECT_TRUE(storage.GetInstanceInfoBlocks().empty());

  renderer->SetEnabled(true);
  renderer->gaussian_splat.Clear();
  storage.Clear();
  storage.BuildFromScene({}, scene, world_bound, false);
  EXPECT_TRUE(storage.GetInstanceInfoBlocks().empty());
}

TEST(SerializationRegistry, UsesExactRegisteredHandlerBeforeDefaultHandler) {
  Application app;
  ApplicationContextScope scope(app);

  int serialize_handler_count = 0;
  int deserialize_handler_count = 0;
  ASSERT_TRUE(Serialization::RegisterSerializationHandler<TestAsset>(
      [&](YAML::Emitter& out, const TestAsset& asset) {
        ++serialize_handler_count;
        out << YAML::Key << "value" << YAML::Value << asset.value + 1;
      },
      [&](const YAML::Node& in, TestAsset& asset) {
        ++deserialize_handler_count;
        asset.value = in["value"].as<int>();
      },
      "test-owner", "TestAsset", 3));

  const auto* info = Serialization::FindSerializationHandler(typeid(TestAsset).hash_code());
  ASSERT_NE(info, nullptr);
  EXPECT_EQ(info->type_id, typeid(TestAsset).hash_code());
  EXPECT_EQ(info->type_name, "TestAsset");
  EXPECT_EQ(info->owner_name, "test-owner");
  EXPECT_EQ(info->version, 3);

  TestAsset asset;
  asset.value = 10;
  YAML::Emitter out;
  BeginMap(out);
  Serialization::SerializeObject(out, static_cast<IAsset&>(asset));
  out << YAML::EndMap;
  EXPECT_EQ(serialize_handler_count, 1);
  EXPECT_EQ(asset.serialize_count, 0);

  Serialization::DeserializeObject(YAML::Load("{value: 12}"), static_cast<IAsset&>(asset));
  EXPECT_EQ(deserialize_handler_count, 1);
  EXPECT_EQ(asset.deserialize_count, 0);
  EXPECT_EQ(asset.value, 12);
}

TEST(SerializationRegistry, ExactRegisteredHandlerReceivesMostDerivedObject) {
  Application app;
  ApplicationContextScope scope(app);

  ASSERT_TRUE(Serialization::RegisterSerializationHandler<OffsetAsset>(
      [](YAML::Emitter&, const OffsetAsset& asset) {
        EXPECT_EQ(asset.offset_base_value, 0);
      },
      [](const YAML::Node&, OffsetAsset& asset) {
        asset.handler_value = 42;
      }));

  OffsetAsset asset;
  YAML::Emitter out;
  BeginMap(out);
  Serialization::SerializeObject(out, static_cast<IAsset&>(asset));
  out << YAML::EndMap;
  EXPECT_EQ(asset.serialize_count, 0);

  Serialization::DeserializeObject(YAML::Load("{}"), static_cast<IAsset&>(asset));
  EXPECT_EQ(asset.deserialize_count, 0);
  EXPECT_EQ(asset.handler_value, 42);
}

TEST(SerializationRegistry, BaseCategoryHandlerDoesNotHandleDerivedTypeWithoutExactRegistration) {
  Application app;
  ApplicationContextScope scope(app);

  int serialize_handler_count = 0;
  int deserialize_handler_count = 0;
  ASSERT_TRUE(Serialization::RegisterSerializationHandler<IAsset>(
      [&](YAML::Emitter& out, const IAsset&) {
        ++serialize_handler_count;
        out << YAML::Key << "base" << YAML::Value << true;
      },
      [&](const YAML::Node&, IAsset&) {
        ++deserialize_handler_count;
      }));

  TestAsset asset;
  YAML::Emitter out;
  BeginMap(out);
  Serialization::SerializeObject(out, static_cast<IAsset&>(asset));
  out << YAML::EndMap;
  EXPECT_EQ(serialize_handler_count, 0);
  EXPECT_EQ(asset.serialize_count, 0);

  Serialization::DeserializeObject(YAML::Load("{}"), static_cast<IAsset&>(asset));
  EXPECT_EQ(deserialize_handler_count, 0);
  EXPECT_EQ(asset.deserialize_count, 0);
}

TEST(SerializationRegistry, MissingHandlerDirectionDoesNotCallConcreteSerialization) {
  Application app;
  ApplicationContextScope scope(app);

  ASSERT_TRUE(Serialization::RegisterSerializationHandler<TestAsset>(
      [](YAML::Emitter& out, const TestAsset&) {
        out << YAML::Key << "handled" << YAML::Value << true;
      },
      {}));

  TestAsset asset;
  YAML::Emitter out;
  BeginMap(out);
  Serialization::SerializeObject(out, static_cast<IAsset&>(asset));
  out << YAML::EndMap;
  EXPECT_EQ(asset.serialize_count, 0);

  Serialization::DeserializeObject(YAML::Load("{value: 14}"), static_cast<IAsset&>(asset));
  EXPECT_EQ(asset.deserialize_count, 0);
  EXPECT_EQ(asset.value, 0);

  EXPECT_EQ(Serialization::FindSerializationHandler(typeid(TestSystem).hash_code()), nullptr);
}

TEST(SerializationRegistry, UnregistersOwnerHandlers) {
  Application app;
  ApplicationContextScope scope(app);

  ASSERT_TRUE(Serialization::RegisterSerializationHandler<TestAsset>(
      [](YAML::Emitter&, const TestAsset&) {
      },
      [](const YAML::Node&, TestAsset&) {
      },
      "owner-a"));
  ASSERT_TRUE(Serialization::RegisterSerializationHandler<TestPrivateComponent>(
      [](YAML::Emitter&, const TestPrivateComponent&) {
      },
      [](const YAML::Node&, TestPrivateComponent&) {
      },
      "owner-a"));
  ASSERT_TRUE(Serialization::RegisterSerializationHandler<TestSystem>(
      [](YAML::Emitter&, const TestSystem&) {
      },
      [](const YAML::Node&, TestSystem&) {
      },
      "owner-b"));

  EXPECT_EQ(Serialization::UnregisterSerializationHandlersByOwner("owner-a"), 2);
  EXPECT_FALSE(Serialization::HasSerializationHandler<TestAsset>());
  EXPECT_FALSE(Serialization::HasSerializationHandler<TestPrivateComponent>());
  EXPECT_TRUE(Serialization::HasSerializationHandler<TestSystem>());

  TestAsset asset;
  YAML::Emitter out;
  BeginMap(out);
  Serialization::SerializeObject(out, static_cast<IAsset&>(asset));
  out << YAML::EndMap;
  EXPECT_EQ(asset.serialize_count, 0);
}

TEST(SerializationRegistry, MissingSupportHandlersDoNotCallConcreteSupportMethods) {
  Application app;
  ApplicationContextScope scope(app);

  std::vector<AssetRef> asset_refs;

  TestAsset asset;
  Serialization::CollectAssetRefs(static_cast<IAsset&>(asset), asset_refs);
  EXPECT_EQ(asset.collect_asset_ref_count, 0);

  TestPrivateComponent component;
  Serialization::CollectAssetRefs(static_cast<IPrivateComponent&>(component), asset_refs);
  EXPECT_EQ(component.collect_asset_ref_count, 0);
  Serialization::RelinkObject(static_cast<IPrivateComponent&>(component), {}, {});
  EXPECT_EQ(component.relink_count, 0);

  TestSystem system;
  Serialization::CollectAssetRefs(static_cast<ISystem&>(system), asset_refs);
  EXPECT_EQ(system.collect_asset_ref_count, 0);
}

TEST(SerializationRegistry, RegisteredTypesInstallDefaultSupportHandlers) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  app.RegisterAsset<TestAsset>("TestAsset", {".evetestasset"});
  app.RegisterPrivateComponent<TestPrivateComponent>("TestPrivateComponent");
  app.RegisterSystem<TestSystem>("TestSystem");

  std::vector<AssetRef> asset_refs;

  TestAsset asset;
  Serialization::CollectAssetRefs(static_cast<IAsset&>(asset), asset_refs);
  EXPECT_EQ(asset.collect_asset_ref_count, 1);

  TestPrivateComponent component;
  Serialization::CollectAssetRefs(static_cast<IPrivateComponent&>(component), asset_refs);
  EXPECT_EQ(component.collect_asset_ref_count, 1);
  Serialization::RelinkObject(static_cast<IPrivateComponent&>(component), {}, {});
  EXPECT_EQ(component.relink_count, 1);

  TestSystem system;
  Serialization::CollectAssetRefs(static_cast<ISystem&>(system), asset_refs);
  EXPECT_EQ(system.collect_asset_ref_count, 1);
}

TEST(SerializationRegistry, MissingAssetIoHandlersDoNotCallConcreteAssetIoMethods) {
  Application app;
  ApplicationContextScope scope(app);

  TestAssetIo asset;
  const auto path = std::filesystem::path("concrete.eveasset");

  EXPECT_FALSE(Serialization::SaveAsset(asset, path));
  EXPECT_FALSE(Serialization::LoadAsset(asset, path));
  EXPECT_FALSE(Serialization::SupportsStagedAssetLoading(asset, path));
  EXPECT_FALSE(Serialization::LoadStagedAssetPayload(asset, path));
  EXPECT_FALSE(Serialization::ApplyStagedAssetPayload(asset, path, std::make_shared<TestAssetIoPayload>()));

  EXPECT_EQ(asset.concrete_save_count, 0);
  EXPECT_EQ(asset.concrete_load_count, 0);
  EXPECT_EQ(asset.concrete_supports_staged_loading_count, 0);
  EXPECT_EQ(asset.concrete_load_staged_payload_count, 0);
  EXPECT_EQ(asset.concrete_apply_staged_payload_count, 0);
  EXPECT_EQ(asset.value, 0);
}

TEST(SerializationRegistry, DefaultAssetIoHandlersUseGenericYamlAndDefaultPreview) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  app.RegisterAsset<TestAssetIo>("TestAssetIo", {".evetestassetio"});
  RegisterTestAssetIoSerializationHandler();

  TempProject project;
  TestAssetIo asset;
  const auto path = project.RootPath() / "generic.evetestassetio";

  asset.value = 17;
  EXPECT_TRUE(Serialization::SaveAsset(asset, path));
  asset.value = 0;
  EXPECT_TRUE(Serialization::LoadAsset(asset, path));
  EXPECT_EQ(asset.value, 17);
  EXPECT_FALSE(Serialization::SupportsStagedAssetLoading(asset, path));
  const auto payload = Serialization::LoadStagedAssetPayload(asset, path);
  ASSERT_TRUE(payload);
  asset.value = 0;
  EXPECT_TRUE(Serialization::ApplyStagedAssetPayload(asset, path, payload));
  EXPECT_EQ(asset.value, 17);

  EXPECT_EQ(asset.concrete_save_count, 0);
  EXPECT_EQ(asset.concrete_load_count, 0);
  EXPECT_EQ(asset.concrete_supports_staged_loading_count, 0);
  EXPECT_EQ(asset.concrete_load_staged_payload_count, 0);
  EXPECT_EQ(asset.concrete_apply_staged_payload_count, 0);
  EXPECT_EQ(asset.serialize_count, 1);
  EXPECT_EQ(asset.deserialize_count, 2);

  const auto preview_asset = AssetManager::CreateTemporaryAsset<TestAssetIo>();
  ASSERT_TRUE(preview_asset);
  OffscreenPreviewSettings settings;
  EXPECT_EQ(Serialization::GenerateAssetThumbnail(preview_asset, settings), nullptr);
  EXPECT_EQ(preview_asset->concrete_generate_thumbnail_count, 0);
}

TEST(SerializationRegistry, UsesExactAssetIoHandlersBeforeDefaultHandlers) {
  Application app;
  ApplicationContextScope scope(app);

  int save_count = 0;
  int load_count = 0;
  int supports_count = 0;
  int load_payload_count = 0;
  int apply_payload_count = 0;
  ASSERT_TRUE(Serialization::RegisterAssetIoHandler<TestAssetIo>(
      [&](const TestAssetIo&, const std::filesystem::path& path) {
        ++save_count;
        return path.filename() == "handled.eveasset";
      },
      [&](TestAssetIo& asset, const std::filesystem::path& path) {
        ++load_count;
        asset.value = 12;
        return path.filename() == "handled.eveasset";
      },
      [&](const TestAssetIo&, const std::filesystem::path&) {
        ++supports_count;
        return true;
      },
      [&](const TestAssetIo&, const std::filesystem::path&) {
        ++load_payload_count;
        auto payload = std::make_shared<TestAssetIoPayload>();
        payload->value = 42;
        return payload;
      },
      [&](TestAssetIo& asset, const std::filesystem::path&, const std::shared_ptr<StagedAssetLoadPayload>& payload) {
        ++apply_payload_count;
        const auto typed_payload = std::dynamic_pointer_cast<TestAssetIoPayload>(payload);
        if (!typed_payload) {
          return false;
        }
        asset.value = typed_payload->value;
        return true;
      },
      "test-owner", "TestAssetIo", 7));

  const auto* info = Serialization::FindAssetIoHandler(typeid(TestAssetIo).hash_code());
  ASSERT_NE(info, nullptr);
  EXPECT_EQ(info->type_id, typeid(TestAssetIo).hash_code());
  EXPECT_EQ(info->type_name, "TestAssetIo");
  EXPECT_EQ(info->owner_name, "test-owner");
  EXPECT_EQ(info->version, 7);

  TestAssetIo asset;
  const auto path = std::filesystem::path("handled.eveasset");

  EXPECT_TRUE(Serialization::SaveAsset(asset, path));
  EXPECT_TRUE(Serialization::LoadAsset(asset, path));
  EXPECT_TRUE(Serialization::SupportsStagedAssetLoading(asset, path));
  const auto payload = Serialization::LoadStagedAssetPayload(asset, path);
  ASSERT_TRUE(payload);
  EXPECT_TRUE(Serialization::ApplyStagedAssetPayload(asset, path, payload));

  EXPECT_EQ(save_count, 1);
  EXPECT_EQ(load_count, 1);
  EXPECT_EQ(supports_count, 1);
  EXPECT_EQ(load_payload_count, 1);
  EXPECT_EQ(apply_payload_count, 1);
  EXPECT_EQ(asset.concrete_save_count, 0);
  EXPECT_EQ(asset.concrete_load_count, 0);
  EXPECT_EQ(asset.concrete_supports_staged_loading_count, 0);
  EXPECT_EQ(asset.concrete_load_staged_payload_count, 0);
  EXPECT_EQ(asset.concrete_apply_staged_payload_count, 0);
  EXPECT_EQ(asset.value, 42);
}

TEST(SerializationRegistry, ExactAssetIoHandlersOverrideDefaultOperations) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  app.RegisterAsset<TestAssetIo>("TestAssetIo", {".evetestassetio"});
  RegisterTestAssetIoSerializationHandler();

  TempProject project;
  const auto default_path = project.RootPath() / "default.evetestassetio";
  TestAssetIo asset;
  asset.value = 22;
  ASSERT_TRUE(Serialization::SaveAsset(asset, default_path));

  int save_count = 0;
  int supports_count = 0;
  ASSERT_TRUE(Serialization::RegisterAssetIoHandler<TestAssetIo>(
      [&](const TestAssetIo&, const std::filesystem::path& path) {
        ++save_count;
        return path.filename() == "exact.evetestassetio";
      },
      {},
      [&](const TestAssetIo&, const std::filesystem::path&) {
        ++supports_count;
        return true;
      },
      {}, {}, "test-owner", "TestAssetIo", 3));

  const auto exact_path = project.RootPath() / "exact.evetestassetio";
  EXPECT_TRUE(Serialization::SaveAsset(asset, exact_path));
  asset.value = 0;
  EXPECT_TRUE(Serialization::LoadAsset(asset, default_path));
  EXPECT_EQ(asset.value, 22);
  EXPECT_TRUE(Serialization::SupportsStagedAssetLoading(asset, default_path));
  const auto payload = Serialization::LoadStagedAssetPayload(asset, default_path);
  ASSERT_TRUE(payload);
  asset.value = 0;
  EXPECT_TRUE(Serialization::ApplyStagedAssetPayload(asset, default_path, payload));
  EXPECT_EQ(asset.value, 22);

  EXPECT_EQ(save_count, 1);
  EXPECT_EQ(supports_count, 1);
  EXPECT_EQ(asset.concrete_save_count, 0);
  EXPECT_EQ(asset.concrete_load_count, 0);
  EXPECT_EQ(asset.concrete_supports_staged_loading_count, 0);
  EXPECT_EQ(asset.concrete_load_staged_payload_count, 0);
  EXPECT_EQ(asset.concrete_apply_staged_payload_count, 0);
  EXPECT_EQ(asset.serialize_count, 1);
  EXPECT_EQ(asset.deserialize_count, 2);
}

TEST(SerializationRegistry, BaseAssetIoHandlerDoesNotHandleDerivedTypeWithoutExactRegistration) {
  Application app;
  ApplicationContextScope scope(app);

  int save_count = 0;
  int load_count = 0;
  int supports_count = 0;
  int load_payload_count = 0;
  int apply_payload_count = 0;
  ASSERT_TRUE(Serialization::RegisterAssetIoHandler<IAsset>(
      [&](const IAsset&, const std::filesystem::path&) {
        ++save_count;
        return true;
      },
      [&](IAsset& asset, const std::filesystem::path&) {
        ++load_count;
        dynamic_cast<TestAssetIo&>(asset).value = 13;
        return true;
      },
      [&](const IAsset&, const std::filesystem::path&) {
        ++supports_count;
        return true;
      },
      [&](const IAsset&, const std::filesystem::path&) {
        ++load_payload_count;
        auto payload = std::make_shared<TestAssetIoPayload>();
        payload->value = 43;
        return payload;
      },
      [&](IAsset& asset, const std::filesystem::path&, const std::shared_ptr<StagedAssetLoadPayload>& payload) {
        ++apply_payload_count;
        dynamic_cast<TestAssetIo&>(asset).value = std::dynamic_pointer_cast<TestAssetIoPayload>(payload)->value;
        return true;
      }));

  TestAssetIo asset;
  const auto path = std::filesystem::path("handled.eveasset");

  EXPECT_FALSE(Serialization::SaveAsset(asset, path));
  EXPECT_FALSE(Serialization::LoadAsset(asset, path));
  EXPECT_FALSE(Serialization::SupportsStagedAssetLoading(asset, path));
  const auto payload = Serialization::LoadStagedAssetPayload(asset, path);
  EXPECT_FALSE(payload);
  EXPECT_FALSE(Serialization::ApplyStagedAssetPayload(asset, path, std::make_shared<TestAssetIoPayload>()));

  EXPECT_EQ(save_count, 0);
  EXPECT_EQ(load_count, 0);
  EXPECT_EQ(supports_count, 0);
  EXPECT_EQ(load_payload_count, 0);
  EXPECT_EQ(apply_payload_count, 0);
  EXPECT_EQ(asset.concrete_save_count, 0);
  EXPECT_EQ(asset.concrete_load_count, 0);
  EXPECT_EQ(asset.concrete_supports_staged_loading_count, 0);
  EXPECT_EQ(asset.concrete_load_staged_payload_count, 0);
  EXPECT_EQ(asset.concrete_apply_staged_payload_count, 0);
  EXPECT_EQ(asset.value, 0);
}

TEST(SerializationRegistry, MissingAssetPreviewHandlersDoNotCallConcreteThumbnailMethods) {
  Application app;
  ApplicationContextScope scope(app);

  const auto asset = std::make_shared<TestAssetIo>();
  OffscreenPreviewSettings settings;
  settings.camera_zoom = 1.25f;

  EXPECT_EQ(Serialization::GenerateAssetThumbnail(asset, settings), nullptr);
  EXPECT_EQ(asset->concrete_generate_thumbnail_count, 0);
}

TEST(SerializationRegistry, UsesExactAssetPreviewHandlerBeforeDefaultHandler) {
  Application app;
  ApplicationContextScope scope(app);

  int preview_count = 0;
  const auto expected_thumbnail = std::make_shared<Texture2D>();
  ASSERT_TRUE(Serialization::RegisterAssetPreviewHandler<TestAssetIo>(
      [&](const std::shared_ptr<TestAssetIo>& asset, const OffscreenPreviewSettings& settings) {
        ++preview_count;
        asset->value = static_cast<int>(settings.camera_zoom * 10.0f);
        return expected_thumbnail;
      },
      "test-owner", "TestAssetIo", 9));

  const auto* info = Serialization::FindAssetPreviewHandler(typeid(TestAssetIo).hash_code());
  ASSERT_NE(info, nullptr);
  EXPECT_EQ(info->type_id, typeid(TestAssetIo).hash_code());
  EXPECT_EQ(info->type_name, "TestAssetIo");
  EXPECT_EQ(info->owner_name, "test-owner");
  EXPECT_EQ(info->version, 9);

  const auto asset = std::make_shared<TestAssetIo>();
  OffscreenPreviewSettings settings;
  settings.camera_zoom = 1.7f;

  EXPECT_EQ(Serialization::GenerateAssetThumbnail(asset, settings), expected_thumbnail);
  EXPECT_EQ(preview_count, 1);
  EXPECT_EQ(asset->concrete_generate_thumbnail_count, 0);
  EXPECT_EQ(asset->value, 17);
}

TEST(SerializationRegistry, AssetThumbnailProviderUsesPreviewRegistry) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  app.RegisterAsset<TestAssetIo>("TestAssetIo", {".evetestassetio"});

  const auto expected_thumbnail = std::make_shared<Texture2D>();
  ASSERT_TRUE(Serialization::RegisterAssetPreviewHandler<TestAssetIo>(
      [&](const std::shared_ptr<TestAssetIo>& asset, const OffscreenPreviewSettings& settings) {
        asset->value = static_cast<int>(settings.camera_zoom * 10.0f);
        return expected_thumbnail;
      },
      "test-owner", "TestAssetIo", 1));

  const auto asset = AssetManager::CreateTemporaryAsset<TestAssetIo>();
  ASSERT_TRUE(asset);
  OffscreenPreviewSettings settings;
  settings.camera_zoom = 2.0f;

  EXPECT_TRUE(AssetThumbnailProvider::SupportsGeneratedThumbnail("TestAssetIo"));
  EXPECT_EQ(AssetThumbnailProvider::GenerateThumbnail(asset, settings), expected_thumbnail);
  EXPECT_EQ(asset->concrete_generate_thumbnail_count, 0);
  EXPECT_EQ(asset->value, 20);
}

TEST(SerializationRegistry, UsesExactSupportHandlersBeforeDefaultHandler) {
  Application app;
  ApplicationContextScope scope(app);

  int collect_count = 0;
  int relink_count = 0;
  int clone_count = 0;
  ASSERT_TRUE(Serialization::RegisterSerializationSupportHandler<TestPrivateComponent>(
      [&](TestPrivateComponent& component, std::vector<AssetRef>&) {
        ++collect_count;
        component.value = 21;
      },
      [&](TestPrivateComponent& component, const std::unordered_map<Handle, Handle>&, const std::shared_ptr<Scene>&) {
        ++relink_count;
        component.value = 22;
      },
      [&](const std::shared_ptr<TestPrivateComponent>& target, const std::shared_ptr<TestPrivateComponent>& source) {
        ++clone_count;
        ASSERT_TRUE(target);
        ASSERT_TRUE(source);
        target->value = source->value + 1;
      },
      "test-owner", "TestPrivateComponentSupport", 5));

  const auto* info = Serialization::FindSerializationSupportHandler(typeid(TestPrivateComponent).hash_code());
  ASSERT_NE(info, nullptr);
  EXPECT_EQ(info->type_id, typeid(TestPrivateComponent).hash_code());
  EXPECT_EQ(info->type_name, "TestPrivateComponentSupport");
  EXPECT_EQ(info->owner_name, "test-owner");
  EXPECT_EQ(info->version, 5);

  TestPrivateComponent component;
  std::vector<AssetRef> asset_refs;
  Serialization::CollectAssetRefs(static_cast<IPrivateComponent&>(component), asset_refs);
  EXPECT_EQ(collect_count, 1);
  EXPECT_EQ(component.collect_asset_ref_count, 0);
  EXPECT_EQ(component.value, 21);

  Serialization::RelinkObject(static_cast<IPrivateComponent&>(component), {}, {});
  EXPECT_EQ(relink_count, 1);
  EXPECT_EQ(component.relink_count, 0);
  EXPECT_EQ(component.value, 22);

  const auto source = std::make_shared<TestPrivateComponent>();
  const auto target = std::make_shared<TestPrivateComponent>();
  source->value = 30;
  Serialization::ClonePrivateComponent(std::static_pointer_cast<IPrivateComponent>(target),
                                       std::static_pointer_cast<IPrivateComponent>(source));
  EXPECT_EQ(clone_count, 1);
  EXPECT_EQ(target->value, 31);
}

TEST(SerializationRegistry, RegisteredTypesInstallDefaultCloneSupportHandlers) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  app.RegisterPrivateComponent<RoutedSceneComponent>("RoutedSceneComponent");
  app.RegisterSystem<RoutedSceneSystem>("RoutedSceneSystem");

  auto source_component = std::make_shared<RoutedSceneComponent>();
  auto target_component = std::make_shared<RoutedSceneComponent>();
  source_component->value = 17;
  Serialization::ClonePrivateComponent(std::static_pointer_cast<IPrivateComponent>(target_component),
                                       std::static_pointer_cast<IPrivateComponent>(source_component));
  EXPECT_EQ(target_component->value, 17);

  auto source_system = std::make_shared<RoutedSceneSystem>();
  auto target_system = std::make_shared<RoutedSceneSystem>();
  source_system->value = 23;
  Serialization::CloneSystem(std::static_pointer_cast<ISystem>(target_system),
                             std::static_pointer_cast<ISystem>(source_system));
  EXPECT_EQ(target_system->value, 23);
}

TEST(SerializationRegistry, AssetRefKeepsUnresolvedSerializedHandle) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());

  constexpr uint64_t kMissingTextureHandle = 0xE701'0000'0001'0001ull;
  AssetRef asset_ref;
  const auto before = AssetManager::GetAssetLoadSnapshot();
  asset_ref.Deserialize(
      YAML::Load("{asset_handle_: " + std::to_string(kMissingTextureHandle) + ", type_name_: Texture2D}"));
  const auto after_deserialize = AssetManager::GetAssetLoadSnapshot();

  EXPECT_EQ(asset_ref.GetAssetHandle().GetValue(), kMissingTextureHandle);
  EXPECT_EQ(asset_ref.Get<Texture2D>(), nullptr);
  const auto after_get = AssetManager::GetAssetLoadSnapshot();
  EXPECT_EQ(asset_ref.GetAssetHandle().GetValue(), kMissingTextureHandle);
  EXPECT_EQ(after_deserialize.total, before.total);
  EXPECT_EQ(after_get.total, before.total);
  EXPECT_FALSE(after_get.Active());
}

TEST(SerializationRegistry, MaterialRoundTripKeepsTransparentExtensionFields) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());

  const auto material = AssetManager::CreateTemporaryAsset<Material>();
  ASSERT_TRUE(material);
  auto& shade_material = material->material_data.shade_material;
  shade_material.alpha_mode = static_cast<int32_t>(GltfAlphaMode::Blend);
  shade_material.alpha_cutoff = 0.37f;
  shade_material.transmission_factor = 0.42f;
  shade_material.attenuation_color = glm::vec3(0.2f, 0.4f, 0.8f);
  shade_material.attenuation_distance = 12.0f;
  shade_material.thickness_factor = 0.75f;
  shade_material.diffuse_transmission_color = glm::vec3(0.7f, 0.8f, 0.9f);
  shade_material.diffuse_transmission_factor = 0.65f;
  shade_material.multiscatter_color_factor = glm::vec3(0.3f, 0.4f, 0.5f);
  shade_material.scatter_anisotropy = -0.25f;
  shade_material.transmission_texture = 3;
  shade_material.thickness_texture = 4;
  shade_material.diffuse_transmission_texture = 5;
  shade_material.diffuse_transmission_color_texture = 6;

  YAML::Emitter out;
  BeginMap(out);
  Serialization::SerializeObject(out, static_cast<IAsset&>(*material));
  out << YAML::EndMap;

  const auto restored = AssetManager::CreateTemporaryAsset<Material>();
  ASSERT_TRUE(restored);
  Serialization::DeserializeObject(YAML::Load(out.c_str()), static_cast<IAsset&>(*restored));
  const auto& restored_material = restored->material_data.shade_material;
  EXPECT_EQ(restored_material.alpha_mode, static_cast<int32_t>(GltfAlphaMode::Blend));
  EXPECT_FLOAT_EQ(restored_material.alpha_cutoff, 0.37f);
  EXPECT_FLOAT_EQ(restored_material.transmission_factor, 0.42f);
  EXPECT_EQ(restored_material.attenuation_color, glm::vec3(0.2f, 0.4f, 0.8f));
  EXPECT_FLOAT_EQ(restored_material.attenuation_distance, 12.0f);
  EXPECT_FLOAT_EQ(restored_material.thickness_factor, 0.75f);
  EXPECT_EQ(restored_material.diffuse_transmission_color, glm::vec3(0.7f, 0.8f, 0.9f));
  EXPECT_FLOAT_EQ(restored_material.diffuse_transmission_factor, 0.65f);
  EXPECT_EQ(restored_material.multiscatter_color_factor, glm::vec3(0.3f, 0.4f, 0.5f));
  EXPECT_FLOAT_EQ(restored_material.scatter_anisotropy, -0.25f);
  EXPECT_EQ(restored_material.transmission_texture, 3);
  EXPECT_EQ(restored_material.thickness_texture, 4);
  EXPECT_EQ(restored_material.diffuse_transmission_texture, 5);
  EXPECT_EQ(restored_material.diffuse_transmission_color_texture, 6);
}

TEST(SerializationRegistry, PrefabMeshRendererMaterialTextureRefsAreCollectedAndLoaded) {
  const auto prefab_path = std::filesystem::temp_directory_path() / "EvoEngine_PrefabMaterialTextureRefs.eveprefab";
  std::filesystem::remove(prefab_path);
  Handle material_handle = 0;
  std::array<Handle, 5> texture_handles{};

  {
    SCOPED_TRACE("save prefab");
    Application app;
    ApplicationContextScope scope(app);
    app.Initialize(EmptyProjectSettings());
    RegisterTestablePrefabHandlers();

    const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
    ASSERT_TRUE(scene);
    app.Attach(scene);

    const auto entity = scene->CreateEntity("Textured Prefab Source");
    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
    ASSERT_TRUE(mesh_renderer);

    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    const auto albedo = CreateSinglePixelTexture(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
    const auto normal = CreateSinglePixelTexture(glm::vec4(0.5f, 0.5f, 1.0f, 1.0f));
    const auto metallic_roughness = CreateSinglePixelTexture(glm::vec4(1.0f, 0.7f, 0.0f, 1.0f));
    const auto emissive = CreateSinglePixelTexture(glm::vec4(0.2f, 0.3f, 0.4f, 1.0f));
    const auto ao = CreateSinglePixelTexture(glm::vec4(1.0f));
    ASSERT_TRUE(material);
    ASSERT_TRUE(albedo);
    ASSERT_TRUE(normal);
    ASSERT_TRUE(metallic_roughness);
    ASSERT_TRUE(emissive);
    ASSERT_TRUE(ao);

    material->SetTexture(&GltfShadeMaterial::pbr_base_color_texture, albedo);
    material->SetTexture(&GltfShadeMaterial::normal_texture, normal);
    material->SetTexture(&GltfShadeMaterial::pbr_metallic_roughness_texture, metallic_roughness);
    material->SetTexture(&GltfShadeMaterial::emissive_texture, emissive);
    material->SetTexture(&GltfShadeMaterial::occlusion_texture, ao);
    mesh_renderer->material = material;

    TestablePrefab prefab;
    prefab.FromEntity(entity);

    std::unordered_map<Handle, std::shared_ptr<IAsset>> local_assets;
    prefab.CollectAssets(local_assets);
    EXPECT_EQ(local_assets.count(material->GetHandle()), 1);
    EXPECT_EQ(local_assets.count(albedo->GetHandle()), 1);
    EXPECT_EQ(local_assets.count(normal->GetHandle()), 1);
    EXPECT_EQ(local_assets.count(metallic_roughness->GetHandle()), 1);
    EXPECT_EQ(local_assets.count(emissive->GetHandle()), 1);
    EXPECT_EQ(local_assets.count(ao->GetHandle()), 1);

    material_handle = material->GetHandle();
    texture_handles = {albedo->GetHandle(), normal->GetHandle(), metallic_roughness->GetHandle(), emissive->GetHandle(),
                       ao->GetHandle()};
    ASSERT_TRUE(prefab.SaveTo(prefab_path));
  }

  {
    SCOPED_TRACE("validate saved local texture payloads");
    std::ifstream file(prefab_path);
    const std::string saved_text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    EXPECT_NE(saved_text.find("TypeName: Texture2D"), std::string::npos);
    EXPECT_NE(saved_text.find("Handle: " + std::to_string(texture_handles[0].GetValue())), std::string::npos);
  }

  SCOPED_TRACE("load prefab after asset-manager restart");
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  RegisterTestablePrefabHandlers();
  TestablePrefab loaded_prefab;
  ASSERT_TRUE(loaded_prefab.LoadFrom(prefab_path));
  std::filesystem::remove(prefab_path);

  SCOPED_TRACE("resolve loaded refs");
  const auto loaded_mesh_renderer = loaded_prefab.GetPrivateComponent<MeshRenderer>();
  ASSERT_TRUE(loaded_mesh_renderer);
  const auto loaded_material = loaded_mesh_renderer->material.Get<Material>();
  ASSERT_TRUE(loaded_material);
  EXPECT_EQ(loaded_material->GetHandle(), material_handle);
  const auto loaded_albedo = loaded_material->GetTexture(&GltfShadeMaterial::pbr_base_color_texture);
  const auto loaded_normal = loaded_material->GetTexture(&GltfShadeMaterial::normal_texture);
  const auto loaded_metallic_roughness =
      loaded_material->GetTexture(&GltfShadeMaterial::pbr_metallic_roughness_texture);
  const auto loaded_emissive = loaded_material->GetTexture(&GltfShadeMaterial::emissive_texture);
  const auto loaded_ao = loaded_material->GetTexture(&GltfShadeMaterial::occlusion_texture);
  ASSERT_TRUE(loaded_albedo);
  ASSERT_TRUE(loaded_normal);
  ASSERT_TRUE(loaded_metallic_roughness);
  ASSERT_TRUE(loaded_emissive);
  ASSERT_TRUE(loaded_ao);
  EXPECT_EQ(loaded_albedo->GetHandle(), texture_handles[0]);
  EXPECT_EQ(loaded_normal->GetHandle(), texture_handles[1]);
  EXPECT_EQ(loaded_metallic_roughness->GetHandle(), texture_handles[2]);
  EXPECT_EQ(loaded_emissive->GetHandle(), texture_handles[3]);
  EXPECT_EQ(loaded_ao->GetHandle(), texture_handles[4]);
}

TEST(SerializationRegistry, PrefabSaveKeepsProjectTextureHandlesExternal) {
  TempProject project;
  const auto prefab_path = project.RootPath() / "ProjectTexturePrefab.eveprefab";
  Handle material_handle = 0;
  Handle texture_handle = 0;

  {
    Application app;
    ApplicationContextScope scope(app);
    app.Initialize(ProjectSettings(project));
    RegisterTestablePrefabHandlers();

    const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
    ASSERT_TRUE(scene);
    app.Attach(scene);

    const auto project_texture = CreateSinglePixelTexture(glm::vec4(0.2f, 0.4f, 0.8f, 1.0f));
    ASSERT_TRUE(project_texture->SetPathAndSave("Textures/ProjectTexture.evetexture2d"));
    ASSERT_FALSE(project_texture->IsTemporary());
    texture_handle = project_texture->GetHandle();

    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    ASSERT_TRUE(material);
    material->SetTexture(&GltfShadeMaterial::pbr_base_color_texture, project_texture);
    material_handle = material->GetHandle();

    const auto entity = scene->CreateEntity("Project Texture Prefab Source");
    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
    ASSERT_TRUE(mesh_renderer);
    mesh_renderer->material = material;

    TestablePrefab prefab;
    prefab.FromEntity(entity);
    ASSERT_TRUE(prefab.SaveTo(prefab_path));
  }

  {
    std::ifstream file(prefab_path);
    const std::string saved_text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    EXPECT_NE(saved_text.find("TypeName: Material"), std::string::npos);
    EXPECT_NE(saved_text.find("Handle: " + std::to_string(material_handle.GetValue())), std::string::npos);
    EXPECT_NE(saved_text.find("asset_handle_: " + std::to_string(texture_handle.GetValue())), std::string::npos);
    EXPECT_EQ(saved_text.find("TypeName: Texture2D"), std::string::npos);
  }

  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(ProjectSettings(project));
  RegisterTestablePrefabHandlers();
  TestablePrefab loaded_prefab;
  ASSERT_TRUE(loaded_prefab.LoadFrom(prefab_path));

  const auto loaded_mesh_renderer = loaded_prefab.GetPrivateComponent<MeshRenderer>();
  ASSERT_TRUE(loaded_mesh_renderer);
  const auto loaded_material = loaded_mesh_renderer->material.Get<Material>();
  ASSERT_TRUE(loaded_material);
  const auto loaded_albedo = loaded_material->GetTexture(&GltfShadeMaterial::pbr_base_color_texture);
  ASSERT_TRUE(loaded_albedo);
  EXPECT_EQ(loaded_albedo->GetHandle(), texture_handle);
}

TEST(SerializationRegistry, SponzaImportSavesProjectTextureHandles) {
  const auto project_path = std::filesystem::absolute("Resources/EvoEngine-DemoProjects/Rendering/Rendering.eveproj");
  if (!std::filesystem::exists(project_path)) {
    GTEST_SKIP() << "Sponza demo project is not available.";
  }
  const auto prefab_path = std::filesystem::temp_directory_path() / "EvoEngine_SponzaImportTextureRefs.eveprefab";
  std::filesystem::remove(prefab_path);

  ApplicationInitializationSettings settings = EmptyProjectSettings();
  settings.allow_empty_project = false;
  settings.project_path = project_path;

  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(settings);
  RegisterTestablePrefabHandlers();

  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  ASSERT_TRUE(scene);
  app.Attach(scene);

  const auto sponza =
      std::dynamic_pointer_cast<Prefab>(ProjectManager::GetOrCreateAsset("Models/Sponza_FBX/Sponza.fbx"));
  ASSERT_TRUE(sponza);
  const auto sponza_entity = sponza->ToEntity(scene);

  TestablePrefab prefab;
  prefab.FromEntity(sponza_entity);
  ASSERT_TRUE(prefab.SaveTo(prefab_path));

  std::string saved_text;
  {
    std::ifstream file(prefab_path);
    saved_text = std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  }

  const auto texture_handles = ReadMaterialTextureHandles(saved_text);
  ASSERT_GT(texture_handles.size(), 0);
  for (const auto& handle : texture_handles) {
    EXPECT_TRUE(FileManager::GetFile(handle)) << "Unresolved saved Sponza texture handle: " << handle.GetValue();
  }
  std::filesystem::remove(prefab_path);
}

TEST(SerializationRegistry, UnregistersOwnerSupportHandlers) {
  Application app;
  ApplicationContextScope scope(app);

  ASSERT_TRUE(Serialization::RegisterSerializationSupportHandler<TestAsset>(
      [](TestAsset&, std::vector<AssetRef>&) {
      },
      {}, {}, "owner-a"));
  ASSERT_TRUE(Serialization::RegisterSerializationSupportHandler<TestSystem>(
      [](TestSystem&, std::vector<AssetRef>&) {
      },
      {}, {}, "owner-b"));

  EXPECT_EQ(Serialization::UnregisterSerializationSupportHandlersByOwner("owner-a"), 1);
  EXPECT_FALSE(Serialization::HasSerializationSupportHandler<TestAsset>());
  EXPECT_TRUE(Serialization::HasSerializationSupportHandler<TestSystem>());

  TestAsset asset;
  std::vector<AssetRef> asset_refs;
  Serialization::CollectAssetRefs(static_cast<IAsset&>(asset), asset_refs);
  EXPECT_EQ(asset.collect_asset_ref_count, 0);
}

TEST(SerializationRegistry, UnregistersOwnerAssetIoHandlers) {
  Application app;
  ApplicationContextScope scope(app);

  ASSERT_TRUE(Serialization::RegisterAssetIoHandler<TestAssetIo>(
      [](const TestAssetIo&, const std::filesystem::path&) {
        return true;
      },
      {}, {}, {}, {}, "owner-a"));
  ASSERT_TRUE(Serialization::RegisterAssetIoHandler<IAsset>(
      [](const IAsset&, const std::filesystem::path&) {
        return true;
      },
      {}, {}, {}, {}, "owner-b"));

  EXPECT_EQ(Serialization::UnregisterAssetIoHandlersByOwner("owner-a"), 1);
  EXPECT_FALSE(Serialization::HasAssetIoHandler<TestAssetIo>());
  EXPECT_TRUE(Serialization::HasAssetIoHandler<IAsset>());

  TestAssetIo asset;
  EXPECT_FALSE(Serialization::SaveAsset(asset, std::filesystem::path("handled.eveasset")));
  EXPECT_EQ(asset.concrete_save_count, 0);
}

TEST(SerializationRegistry, UnregistersOwnerAssetPreviewHandlers) {
  Application app;
  ApplicationContextScope scope(app);

  ASSERT_TRUE(Serialization::RegisterAssetPreviewHandler<TestAssetIo>(
      [](const std::shared_ptr<TestAssetIo>&, const OffscreenPreviewSettings&) {
        return nullptr;
      },
      "owner-a", "TestAssetIo"));
  ASSERT_TRUE(Serialization::RegisterAssetPreviewHandler<IAsset>(
      [](const std::shared_ptr<IAsset>&, const OffscreenPreviewSettings&) {
        return nullptr;
      },
      "owner-b", "IAsset"));

  EXPECT_TRUE(Serialization::HasAssetPreviewHandler("TestAssetIo"));
  EXPECT_EQ(Serialization::UnregisterAssetPreviewHandlersByOwner("owner-a"), 1);
  EXPECT_FALSE(Serialization::HasAssetPreviewHandler<TestAssetIo>());
  EXPECT_TRUE(Serialization::HasAssetPreviewHandler<IAsset>());
  EXPECT_FALSE(Serialization::HasAssetPreviewHandler("TestAssetIo"));
}

TEST(SerializationRegistry, ApplicationRegistersPilotExternalHandlers) {
  Application app;
  ApplicationContextScope scope(app);
  InspectorRegistry::GetInstance().Clear();
  app.Initialize(EmptyProjectSettings());

  const auto* json_io = Serialization::FindAssetIoHandler(typeid(Json).hash_code());
  ASSERT_NE(json_io, nullptr);
  EXPECT_EQ(json_io->type_name, "Json");

  const auto* json_inspector = InspectorRegistry::GetInstance().FindInspector(typeid(Json));
  ASSERT_NE(json_inspector, nullptr);
  EXPECT_EQ(json_inspector->type_name, "Json");

  const auto* way_points_inspector = InspectorRegistry::GetInstance().FindInspector(typeid(WayPoints));
  ASSERT_NE(way_points_inspector, nullptr);
  EXPECT_EQ(way_points_inspector->type_name, "WayPoints");
}

TEST(SerializationRegistry, SceneSerializationRoutesComponentsAndSystemsThroughHandlers) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  app.RegisterPrivateComponent<RoutedSceneComponent>("RoutedSceneComponent");
  app.RegisterSystem<RoutedSceneSystem>("RoutedSceneSystem");

  int component_serialize_count = 0;
  int component_deserialize_count = 0;
  int component_collect_count = 0;
  int system_serialize_count = 0;
  int system_deserialize_count = 0;
  int system_collect_count = 0;

  ASSERT_TRUE(Serialization::RegisterSerializationHandler<RoutedSceneComponent>(
      [&](YAML::Emitter& out, const RoutedSceneComponent& component) {
        ++component_serialize_count;
        out << YAML::Key << "value" << YAML::Value << component.value;
      },
      [&](const YAML::Node& in, RoutedSceneComponent& component) {
        ++component_deserialize_count;
        component.value = in["value"].as<int>() + 100;
      },
      "test-owner"));
  ASSERT_TRUE(Serialization::RegisterSerializationSupportHandler<RoutedSceneComponent>(
      [&](RoutedSceneComponent&, std::vector<AssetRef>&) {
        ++component_collect_count;
      },
      {}, {}, "test-owner"));
  ASSERT_TRUE(Serialization::RegisterSerializationHandler<RoutedSceneSystem>(
      [&](YAML::Emitter& out, const RoutedSceneSystem& system) {
        ++system_serialize_count;
        out << YAML::Key << "value" << YAML::Value << system.value;
      },
      [&](const YAML::Node& in, RoutedSceneSystem& system) {
        ++system_deserialize_count;
        system.value = in["value"].as<int>() + 200;
      },
      "test-owner"));
  ASSERT_TRUE(Serialization::RegisterSerializationSupportHandler<RoutedSceneSystem>(
      [&](RoutedSceneSystem&, std::vector<AssetRef>&) {
        ++system_collect_count;
      },
      {}, {}, "test-owner"));

  const auto source = AssetManager::CreateTemporaryAsset<Scene>();
  ASSERT_TRUE(source);
  const auto entity = source->CreateEntity("Routed Entity");
  const auto component = source->GetOrSetPrivateComponent<RoutedSceneComponent>(entity).lock();
  ASSERT_TRUE(component);
  component->value = 21;
  const auto system = source->GetOrCreateSystem<RoutedSceneSystem>(3.0f);
  ASSERT_TRUE(system);
  system->value = 31;

  YAML::Emitter out;
  BeginMap(out);
  Serialization::SerializeObject(out, static_cast<IAsset&>(*source));
  EXPECT_EQ(component_serialize_count, 1);
  EXPECT_EQ(component_collect_count, 1);
  EXPECT_EQ(system_serialize_count, 1);
  EXPECT_EQ(system_collect_count, 1);

  const auto target = AssetManager::CreateTemporaryAsset<Scene>();
  ASSERT_TRUE(target);
  Serialization::DeserializeObject(YAML::Load(out.c_str()), static_cast<IAsset&>(*target));

  EXPECT_EQ(component_deserialize_count, 1);
  EXPECT_EQ(system_deserialize_count, 1);

  const auto owners = target->GetPrivateComponentOwnersList<RoutedSceneComponent>();
  ASSERT_EQ(owners.size(), 1);
  const auto routed_component = target->GetOrSetPrivateComponent<RoutedSceneComponent>(owners.front()).lock();
  ASSERT_TRUE(routed_component);
  EXPECT_EQ(routed_component->value, 121);

  const auto routed_system = target->GetSystem<RoutedSceneSystem>();
  ASSERT_TRUE(routed_system);
  EXPECT_EQ(routed_system->value, 231);
}
