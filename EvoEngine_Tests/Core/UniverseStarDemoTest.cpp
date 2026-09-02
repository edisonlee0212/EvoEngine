#include <gtest/gtest.h>

#include "Application.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "PostProcessingStack.hpp"
#include "StarCluster.hpp"
#include "StarDemoCamera.hpp"
#include "UniverseSerializationAdapters.hpp"

using namespace evo_engine;
using namespace universe_package;

namespace {
class UniverseStarDemo : public testing::Test {
 protected:
  Application application_;
  void SetUp() override {
    application_.RegisterAsset<PostProcessingStack>("PostProcessingStack", {".evepostprocessingstack"});
  }
};
}  // namespace

TEST_F(UniverseStarDemo, CameraOverridesDeepCopySettingsAndRestoreOriginalReferences) {
  const auto original = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
  ASSERT_TRUE(original);
  original->enable_ambient_occlusion = false;
  original->enable_screen_space_reflection = false;
  original->enable_anti_aliasing = false;
  original->ambient_occlusion->radius = 0.8f;
  original->bloom->intensity = 0.17f;
  original->bloom->compression_start = 3;
  original->bloom->source_ceiling = 12;
  original->screen_space_reflection->max_distance = 57;
  original->anti_aliasing->preset = AntiAliasing::Preset::Medium;
  original->tone_mapping->exposure = 3;
  const auto main_camera = std::make_shared<Camera>();
  const auto scene_camera = std::make_shared<Camera>();
  main_camera->post_processing_stack_ref = original;
  scene_camera->post_processing_stack_ref = original;
  StarDemoCameraOverride main_override, scene_override;
  main_override.Apply(main_camera);
  scene_override.Apply(scene_camera);
  const auto main_stack = main_camera->post_processing_stack_ref.Get<PostProcessingStack>();
  const auto scene_stack = scene_camera->post_processing_stack_ref.Get<PostProcessingStack>();
  ASSERT_TRUE(main_stack);
  ASSERT_TRUE(scene_stack);
  EXPECT_NE(main_stack->GetHandle(), original->GetHandle());
  EXPECT_NE(scene_stack->GetHandle(), main_stack->GetHandle());
  EXPECT_FALSE(main_stack->enable_tone_mapping);
  EXPECT_FALSE(scene_stack->enable_tone_mapping);
  EXPECT_TRUE(original->enable_tone_mapping);
  EXPECT_FALSE(main_stack->enable_ambient_occlusion);
  EXPECT_TRUE(main_stack->enable_bloom);
  EXPECT_FALSE(main_stack->enable_screen_space_reflection);
  EXPECT_FALSE(main_stack->enable_anti_aliasing);
  EXPECT_NE(main_stack->ambient_occlusion, original->ambient_occlusion);
  EXPECT_NE(main_stack->bloom, original->bloom);
  EXPECT_NE(main_stack->screen_space_reflection, original->screen_space_reflection);
  EXPECT_NE(main_stack->anti_aliasing, original->anti_aliasing);
  EXPECT_NE(main_stack->tone_mapping, original->tone_mapping);
  EXPECT_FLOAT_EQ(0.8f, main_stack->ambient_occlusion->radius);
  EXPECT_FLOAT_EQ(0.17f, main_stack->bloom->intensity);
  EXPECT_FLOAT_EQ(3, main_stack->bloom->compression_start);
  EXPECT_FLOAT_EQ(12, main_stack->bloom->source_ceiling);
  EXPECT_FLOAT_EQ(57, main_stack->screen_space_reflection->max_distance);
  EXPECT_EQ(AntiAliasing::Preset::Medium, main_stack->anti_aliasing->preset);
  EXPECT_FLOAT_EQ(3, main_stack->tone_mapping->exposure);
  main_stack->bloom->intensity = 0.6f;
  main_stack->enable_tone_mapping = true;
  main_override.Apply(main_camera);
  EXPECT_EQ(main_stack, main_camera->post_processing_stack_ref.Get<PostProcessingStack>());
  EXPECT_FALSE(main_stack->enable_tone_mapping);
  EXPECT_FLOAT_EQ(0.17f, original->bloom->intensity);
  EXPECT_FLOAT_EQ(0.17f, scene_stack->bloom->intensity);
  main_override.Restore();
  scene_override.Restore();
  EXPECT_EQ(original, main_camera->post_processing_stack_ref.Get<PostProcessingStack>());
  EXPECT_EQ(original, scene_camera->post_processing_stack_ref.Get<PostProcessingStack>());
}

TEST_F(UniverseStarDemo, CameraSwitchAndUserReplacementRespectCurrentOwnership) {
  const auto first_stack = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
  const auto replacement = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
  const auto first_camera = std::make_shared<Camera>();
  const auto second_camera = std::make_shared<Camera>();
  first_camera->post_processing_stack_ref = first_stack;
  second_camera->post_processing_stack_ref = replacement;
  replacement->enable_bloom = false;
  StarDemoCameraOverride controller;
  controller.Apply(first_camera);
  std::weak_ptr<PostProcessingStack> first_override =
      first_camera->post_processing_stack_ref.Get<PostProcessingStack>();
  controller.Apply(second_camera);
  EXPECT_EQ(first_stack, first_camera->post_processing_stack_ref.Get<PostProcessingStack>());
  EXPECT_TRUE(first_override.expired());
  EXPECT_FALSE(second_camera->post_processing_stack_ref.Get<PostProcessingStack>()->enable_bloom);
  second_camera->post_processing_stack_ref = first_stack;
  controller.Restore();
  EXPECT_EQ(first_stack, second_camera->post_processing_stack_ref.Get<PostProcessingStack>());
  controller.Apply(second_camera);
  second_camera->post_processing_stack_ref = replacement;
  controller.Apply(second_camera);
  EXPECT_FALSE(second_camera->post_processing_stack_ref.Get<PostProcessingStack>()->enable_tone_mapping);
  EXPECT_FALSE(second_camera->post_processing_stack_ref.Get<PostProcessingStack>()->enable_bloom);
  controller.Restore();
  EXPECT_EQ(replacement, second_camera->post_processing_stack_ref.Get<PostProcessingStack>());
}

TEST_F(UniverseStarDemo, NullCamerasAndMissingStacksRestoreSafely) {
  const auto camera = std::make_shared<Camera>();
  StarDemoCameraOverride controller;
  controller.Apply(nullptr);
  controller.Apply(camera);
  ASSERT_TRUE(camera->post_processing_stack_ref.Get<PostProcessingStack>());
  EXPECT_FALSE(camera->post_processing_stack_ref.Get<PostProcessingStack>()->enable_tone_mapping);
  controller.Apply(nullptr);
  EXPECT_EQ(Handle(0), camera->post_processing_stack_ref.GetAssetHandle());
  controller.Restore();
}

TEST(UniverseStarDemoDefaults, AuthoringDefaultsAndExplicitSerializedValues) {
  StarCluster cluster;
  EXPECT_DOUBLE_EQ(1, cluster.time_scale);
  EXPECT_FLOAT_EQ(8, cluster.disk_emission_intensity);
  EXPECT_FLOAT_EQ(8, cluster.core_emission_intensity);
  EXPECT_FLOAT_EQ(8, cluster.center_emission_intensity);
  cluster.time_scale = -7;
  cluster.disk_emission_intensity = 1.5f;
  cluster.core_emission_intensity = 2.5f;
  cluster.center_emission_intensity = 3.5f;
  YAML::Emitter out;
  out << YAML::BeginMap;
  SerializeStarCluster(out, cluster);
  out << YAML::EndMap;
  StarCluster loaded;
  DeserializeStarCluster(YAML::Load(out.c_str()), loaded);
  EXPECT_DOUBLE_EQ(-7, loaded.time_scale);
  EXPECT_FLOAT_EQ(1.5f, loaded.disk_emission_intensity);
  EXPECT_FLOAT_EQ(2.5f, loaded.core_emission_intensity);
  EXPECT_FLOAT_EQ(3.5f, loaded.center_emission_intensity);
}
