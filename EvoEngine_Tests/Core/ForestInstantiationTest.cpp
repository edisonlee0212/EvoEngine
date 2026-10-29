#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "AssetManager.hpp"
#include "BasicBarkDescriptor.hpp"
#include "BasicFoliageDescriptor.hpp"
#include "BranchRenderer.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EcoSysLabSystem.hpp"
#include "EvoEngine_SDK_PCH.hpp"
#include "FoliageRenderer.hpp"
#include "ForestDescriptor.hpp"
#include "HeightField.hpp"
#include "Material.hpp"
#include "Scene.hpp"

using namespace eco_sys_lab_package;
using namespace evo_engine;

TEST(ForestInstantiation, EveryPathAddsRenderersWithDescriptorMaterials) {
  Application application;
  ApplicationContextScope scope(application);
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  application.Initialize(settings);
  application.RegisterAsset<BasicBarkDescriptor>("BasicBarkDescriptor", {".bark"});
  application.RegisterAsset<BasicFoliageDescriptor>("BasicFoliageDescriptor", {".foliage"});
  application.RegisterAsset<TreeDescriptor>("TreeDescriptor", {".tree"});
  application.RegisterAsset<ForestPatch>("ForestPatch", {".forestpatch"});
  application.RegisterAsset<ClimateDescriptor>("ClimateDescriptor", {".climate"});
  application.RegisterAsset<SoilDescriptor>("SoilDescriptor", {".soil"});
  application.RegisterAsset<HeightField>("HeightField", {".heightfield"});
  application.RegisterSystem<EcoSysLabSystem>("EcoSysLabSystem");
  application.RegisterPrivateComponent<Climate>("Climate");
  application.RegisterPrivateComponent<Soil>("Soil");
  application.RegisterPrivateComponent<Tree>("Tree");
  application.RegisterPrivateComponent<DynamicTreeSkeleton>("DynamicTreeSkeleton");
  application.RegisterPrivateComponent<BranchRenderer>("BranchRenderer");
  application.RegisterPrivateComponent<FoliageRenderer>("FoliageRenderer");
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  application.Attach(scene);

  const auto empty_scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto defaults = empty_scene->GetOrCreateSystem<EcoSysLabSystem>(0.0f);
  defaults->environment.EnsureConfiguration(empty_scene);
  EXPECT_NE(defaults->environment.climate_descriptor.Get<ClimateDescriptor>(), nullptr);
  const auto default_soil = defaults->environment.soil_descriptor.Get<SoilDescriptor>();
  ASSERT_NE(default_soil, nullptr);
  EXPECT_NE(default_soil->height_field.Get<HeightField>(), nullptr);

  const auto climate_descriptor = AssetManager::CreateTemporaryAsset<ClimateDescriptor>();
  const auto soil_descriptor = AssetManager::CreateTemporaryAsset<SoilDescriptor>();
  const auto disabled_scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto disabled_entity = disabled_scene->CreateEntity("Legacy environment");
  disabled_scene->GetOrSetPrivateComponent<Climate>(disabled_entity).lock()->climate_descriptor_ref =
      climate_descriptor;
  disabled_scene->GetOrSetPrivateComponent<Soil>(disabled_entity).lock()->soil_descriptor_ref = soil_descriptor;
  disabled_scene->SetEnable(disabled_entity, false);
  const auto disabled_system = disabled_scene->GetOrCreateSystem<EcoSysLabSystem>(0.0f);
  disabled_system->environment.EnsureConfiguration(disabled_scene);
  EXPECT_EQ(disabled_system->environment.climate_descriptor.Get<ClimateDescriptor>(), climate_descriptor);
  EXPECT_EQ(disabled_system->environment.soil_descriptor.Get<SoilDescriptor>(), soil_descriptor);
  EXPECT_FALSE(disabled_scene->HasPrivateComponent<Climate>(disabled_entity));
  EXPECT_FALSE(disabled_scene->HasPrivateComponent<Soil>(disabled_entity));
  const auto climate_entity = scene->CreateEntity("Climate");
  const auto soil_entity = scene->CreateEntity("Soil");
  scene->GetOrSetPrivateComponent<Climate>(climate_entity).lock()->climate_descriptor_ref = climate_descriptor;
  scene->GetOrSetPrivateComponent<Soil>(soil_entity).lock()->soil_descriptor_ref = soil_descriptor;
  const auto visualization = scene->CreateEntity("Ground Mesh");
  scene->SetParent(visualization, soil_entity);

  const auto bark_material = AssetManager::CreateTemporaryAsset<Material>();
  const auto leaf_material = AssetManager::CreateTemporaryAsset<Material>();
  const auto bark = AssetManager::CreateTemporaryAsset<BasicBarkDescriptor>();
  const auto leaves = AssetManager::CreateTemporaryAsset<BasicFoliageDescriptor>();
  bark->bark_material_ref = bark_material;
  leaves->leaf_material_ref = leaf_material;
  const auto descriptor = AssetManager::CreateTemporaryAsset<TreeDescriptor>();
  descriptor->bark_descriptor = bark;
  descriptor->foliage_descriptor = leaves;

  const auto check_last_tree = [&](const size_t expected_count) {
    const auto trees = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
    ASSERT_NE(trees, nullptr);
    ASSERT_EQ(trees->size(), expected_count);
    const auto entity = trees->back();
    ASSERT_TRUE(scene->HasPrivateComponent<DynamicTreeSkeleton>(entity));
    ASSERT_TRUE(scene->HasPrivateComponent<BranchRenderer>(entity));
    ASSERT_TRUE(scene->HasPrivateComponent<FoliageRenderer>(entity));
    EXPECT_EQ(scene->GetOrSetPrivateComponent<BranchRenderer>(entity).lock()->material_ref.Get<Material>(),
              bark_material);
    EXPECT_EQ(scene->GetOrSetPrivateComponent<FoliageRenderer>(entity).lock()->material_ref.Get<Material>(),
              leaf_material);
  };

  ForestPatch patch;
  patch.tree_descriptor = descriptor;
  patch.InstantiatePatch({1, 1}, false);
  check_last_tree(1);
  const auto system = scene->GetSystem<EcoSysLabSystem>();
  ASSERT_NE(system, nullptr);
  EXPECT_EQ(system->environment.climate_descriptor.Get<ClimateDescriptor>(), climate_descriptor);
  EXPECT_EQ(system->environment.soil_descriptor.Get<SoilDescriptor>(), soil_descriptor);
  EXPECT_FALSE(scene->HasPrivateComponent<Climate>(climate_entity));
  EXPECT_FALSE(scene->HasPrivateComponent<Soil>(soil_entity));
  EXPECT_TRUE(scene->IsEntityValid(visualization));
  EXPECT_EQ(system->environment.soil_visualization_owner, scene->GetEntityHandle(soil_entity));
  system->growth.Queue(45.0f);
  system->growth.simulated_time = 5.0f;
  const auto runtime_scene = AssetManager::CreateTemporaryAsset<Scene>();
  Scene::Clone(scene, runtime_scene);
  const auto runtime_system = runtime_scene->GetSystem<EcoSysLabSystem>();
  ASSERT_NE(runtime_system, nullptr);
  EXPECT_FLOAT_EQ(runtime_system->growth.remaining_time, 0.0f);
  EXPECT_FLOAT_EQ(runtime_system->growth.simulated_time, 0.0f);
  EXPECT_EQ(runtime_system->environment.soil_descriptor.Get<SoilDescriptor>(), soil_descriptor);
  system->growth.Stop();
  system->growth.simulated_time = 0.0f;

  patch.InstantiatePatch({{TreeGrowthSettings{}, descriptor}}, {1, 1}, false);
  check_last_tree(2);

  ForestDescriptor forest;
  TreeInfo info;
  info.tree_descriptor = descriptor;
  forest.tree_infos.push_back(info);
  forest.InstantiatePatch(false, 0);
  check_last_tree(3);

  const auto no_material_bark = AssetManager::CreateTemporaryAsset<BasicBarkDescriptor>();
  const auto no_material_descriptor = AssetManager::CreateTemporaryAsset<TreeDescriptor>();
  no_material_descriptor->bark_descriptor = no_material_bark;
  patch.tree_descriptor = no_material_descriptor;
  patch.InstantiatePatch({2, 1}, false);
  const auto trees = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  ASSERT_EQ(trees->size(), 5);
  const auto first_material =
      scene->GetOrSetPrivateComponent<BranchRenderer>((*trees)[3]).lock()->material_ref.Get<Material>();
  const auto second_material =
      scene->GetOrSetPrivateComponent<BranchRenderer>((*trees)[4]).lock()->material_ref.Get<Material>();
  ASSERT_NE(first_material, nullptr);
  EXPECT_EQ(first_material, second_material);
  EXPECT_TRUE(glm::all(glm::equal(first_material->material_data.shade_material.pbr_base_color_factor,
                                  glm::vec4(109.0f, 79.0f, 75.0f, 255.0f) / 255.0f)));

  patch.tree_descriptor = descriptor;
  patch.grid_size = {2, 2};
  patch.InstantiatePatch(patch.grid_size, false);
  ASSERT_EQ(trees->size(), 9);
  for (size_t i = 5; i < 9; ++i) {
    EXPECT_TRUE(scene->HasPrivateComponent<DynamicTreeSkeleton>((*trees)[i]));
    EXPECT_TRUE(scene->HasPrivateComponent<BranchRenderer>((*trees)[i]));
    EXPECT_TRUE(scene->HasPrivateComponent<FoliageRenderer>((*trees)[i]));
  }
  YAML::Emitter out;
  out << YAML::BeginMap;
  SerializeForestPatch(out, patch);
  out << YAML::EndMap;

  ForestPatch loaded;
  DeserializeForestPatch(YAML::Load(out.c_str()), loaded);
  EXPECT_EQ(loaded.grid_size, glm::ivec2(2, 2));

  ForestPatch legacy;
  DeserializeForestPatch(YAML::Load("grid_distance: [1, 1]"), legacy);
  EXPECT_EQ(legacy.grid_size, glm::ivec2(8, 8));
}

TEST(ForestPatch, AppleForestAssetUsesFourTrees) {
  const auto path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
                    "Resources/EcoSysLabProject/Assets/AppleForest.forestpatch";
  const auto asset = YAML::LoadFile(path.string());
  EXPECT_EQ(asset["grid_size"].as<glm::ivec2>(), glm::ivec2(2, 2));
  EXPECT_FLOAT_EQ(asset["simulation_time"].as<float>(), 4.0f);
}
