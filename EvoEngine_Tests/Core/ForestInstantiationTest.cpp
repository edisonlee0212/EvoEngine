#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "AssetManager.hpp"
#include "BasicBarkDescriptor.hpp"
#include "BasicFoliageDescriptor.hpp"
#include "BranchRenderer.hpp"
#include "EvoEngine_SDK_PCH.hpp"
#include "FoliageRenderer.hpp"
#include "ForestDescriptor.hpp"
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
  application.RegisterPrivateComponent<Tree>("Tree");
  application.RegisterPrivateComponent<BranchRenderer>("BranchRenderer");
  application.RegisterPrivateComponent<FoliageRenderer>("FoliageRenderer");
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  application.Attach(scene);

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
}
