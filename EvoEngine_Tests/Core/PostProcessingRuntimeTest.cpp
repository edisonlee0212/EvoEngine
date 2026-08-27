#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "Camera.hpp"
#include "PostProcessingStack.hpp"

using namespace evo_engine;

namespace evo_engine {
class PostProcessingRuntimeTestAccess {
 public:
  static PostProcessingCameraResources& Acquire(Camera& camera, const std::shared_ptr<PostProcessingStack>& stack) {
    return camera.AcquirePostProcessingResources(stack);
  }

  static void Synchronize(Camera& camera, const std::shared_ptr<PostProcessingStack>& stack) {
    camera.SynchronizePostProcessingResources(stack);
  }

  static bool HasRuntime(const Camera& camera) {
    return static_cast<bool>(camera.post_processing_resources_);
  }

  static void FinishClone(Camera& camera) {
    camera.PostCloneAction({});
  }
};
}  // namespace evo_engine

namespace {
class PostProcessingRuntime : public testing::Test {
 protected:
  Application application;
  ApplicationContextScope context{application};
};

template <typename T>
std::shared_ptr<T> MakeFakeResource() {
  const auto owner = std::make_shared<uint8_t>();
  return std::shared_ptr<T>(owner, reinterpret_cast<T*>(owner.get()));
}

}  // namespace

TEST_F(PostProcessingRuntime, AssetVersionResetsEachCameraLazilyAndPreservesScratch) {
  const auto stack = std::make_shared<PostProcessingStack>();
  Camera first_camera;
  Camera second_camera;
  auto& first = PostProcessingRuntimeTestAccess::Acquire(first_camera, stack);
  auto& second = PostProcessingRuntimeTestAccess::Acquire(second_camera, stack);

  first.stack.source_color_texture = MakeFakeResource<RenderTexture>();
  first.stack.result_texture = MakeFakeResource<RenderTexture>();
  first.stack.swap_texture = MakeFakeResource<RenderTexture>();
  second.stack.source_color_texture = MakeFakeResource<RenderTexture>();
  second.stack.result_texture = MakeFakeResource<RenderTexture>();
  second.stack.swap_texture = MakeFakeResource<RenderTexture>();
  first.tone_mapping.auto_exposure_time_initialized = true;
  second.tone_mapping.auto_exposure_time_initialized = true;
  first.tone_mapping.luminance_reset_pending = false;
  second.tone_mapping.luminance_reset_pending = false;

  const auto first_source = first.stack.source_color_texture;
  const auto first_result = first.stack.result_texture;
  const auto first_swap = first.stack.swap_texture;
  const auto second_source = second.stack.source_color_texture;
  const auto first_version_resets = first.version_reset_count;
  const auto second_version_resets = second.version_reset_count;
  const auto previous_version = stack->GetVersion();
  stack->SetUnsaved();
  ASSERT_GT(stack->GetVersion(), previous_version);

  PostProcessingRuntimeTestAccess::Synchronize(first_camera, stack);
  EXPECT_FALSE(first.tone_mapping.auto_exposure_time_initialized);
  EXPECT_TRUE(first.tone_mapping.luminance_reset_pending);
  EXPECT_EQ(first.stack.source_color_texture, first_source);
  EXPECT_EQ(first.stack.result_texture, first_result);
  EXPECT_EQ(first.stack.swap_texture, first_swap);
  EXPECT_EQ(first.version_reset_count, first_version_resets + 1);
  EXPECT_TRUE(second.tone_mapping.auto_exposure_time_initialized);
  EXPECT_EQ(second.version_reset_count, second_version_resets);

  PostProcessingRuntimeTestAccess::Synchronize(second_camera, stack);
  EXPECT_FALSE(second.tone_mapping.auto_exposure_time_initialized);
  EXPECT_TRUE(second.tone_mapping.luminance_reset_pending);
  EXPECT_EQ(second.stack.source_color_texture, second_source);
  EXPECT_EQ(second.version_reset_count, second_version_resets + 1);
  EXPECT_NE(first.stack.source_color_texture, second.stack.source_color_texture);
}

TEST_F(PostProcessingRuntime, TechniqueAndExplicitCameraResetsKeepScratchIndependent) {
  const auto stack = std::make_shared<PostProcessingStack>();
  Camera camera;
  auto& resources = PostProcessingRuntimeTestAccess::Acquire(camera, stack);
  resources.stack.source_color_texture = MakeFakeResource<RenderTexture>();
  const auto source = resources.stack.source_color_texture;
  const auto version_resets = resources.version_reset_count;
  const auto technique_resets = resources.technique_reset_count;
  const auto resolution_resets = resources.resolution_reset_count;
  const auto temporal_resets = resources.temporal_reset_count;

  camera.Resize({128, 64});
  PostProcessingRuntimeTestAccess::Synchronize(camera, stack);
  EXPECT_EQ(resources.stack.source_color_texture, source);
  EXPECT_EQ(resources.version_reset_count, version_resets);
  EXPECT_EQ(resources.technique_reset_count, technique_resets);
  EXPECT_EQ(resources.resolution_reset_count, resolution_resets + 1);
  EXPECT_EQ(resources.temporal_reset_count, temporal_resets + 1);
  PostProcessingRuntimeTestAccess::Synchronize(camera, stack);
  EXPECT_EQ(resources.resolution_reset_count, resolution_resets + 1);
  EXPECT_EQ(resources.temporal_reset_count, temporal_resets + 1);

  camera.camera_render_mode = Camera::CameraRenderMode::RayQuery;
  PostProcessingRuntimeTestAccess::Synchronize(camera, stack);
  EXPECT_EQ(resources.stack.source_color_texture, source);
  EXPECT_EQ(resources.version_reset_count, version_resets);
  EXPECT_EQ(resources.technique_reset_count, technique_resets + 1);

  resources.previous_inverse_projection = glm::mat4(2.0f);
  resources.previous_inverse_view = glm::mat4(3.0f);
  resources.previous_matrices_valid = true;
  camera.ResetFrameCount();
  EXPECT_EQ(resources.previous_inverse_projection, glm::mat4(1.0f));
  EXPECT_EQ(resources.previous_inverse_view, glm::mat4(1.0f));
  EXPECT_FALSE(resources.previous_matrices_valid);
  EXPECT_EQ(resources.stack.source_color_texture, source);
}

TEST_F(PostProcessingRuntime, CameraCloneSharesSettingsButNotRuntime) {
  const auto stack = std::make_shared<PostProcessingStack>();
  Camera source;
  auto& source_resources = PostProcessingRuntimeTestAccess::Acquire(source, stack);
  source_resources.stack.source_color_texture = MakeFakeResource<RenderTexture>();

  Camera clone = source;
  ASSERT_TRUE(PostProcessingRuntimeTestAccess::HasRuntime(clone));
  PostProcessingRuntimeTestAccess::FinishClone(clone);
  EXPECT_FALSE(PostProcessingRuntimeTestAccess::HasRuntime(clone));
  auto& clone_resources = PostProcessingRuntimeTestAccess::Acquire(clone, stack);
  EXPECT_NE(&source_resources, &clone_resources);
  EXPECT_FALSE(clone_resources.stack.source_color_texture);
}

TEST_F(PostProcessingRuntime, ApplyDefaultSettingsRestoresEffectsAndEnableFlags) {
  PostProcessingStack stack;
  stack.ApplyDefaultSettings();
  const auto previous_bloom = stack.bloom;
  stack.bloom->threshold = 7.0f;
  stack.tone_mapping->exposure = 3.0f;
  stack.enable_ambient_occlusion = false;
  stack.enable_bloom = true;
  stack.enable_screen_space_reflection = true;
  stack.enable_anti_aliasing = false;
  stack.enable_tone_mapping = false;

  stack.ApplyDefaultSettings();

  ASSERT_TRUE(stack.ambient_occlusion);
  ASSERT_TRUE(stack.bloom);
  ASSERT_TRUE(stack.screen_space_reflection);
  ASSERT_TRUE(stack.anti_aliasing);
  ASSERT_TRUE(stack.tone_mapping);
  EXPECT_NE(stack.bloom, previous_bloom);
  EXPECT_FLOAT_EQ(stack.bloom->threshold, Bloom{}.threshold);
  EXPECT_FLOAT_EQ(stack.bloom->intensity, 0.2f);
  EXPECT_FLOAT_EQ(stack.tone_mapping->exposure, ToneMapping{}.exposure);
  EXPECT_TRUE(stack.enable_ambient_occlusion);
  EXPECT_TRUE(stack.enable_bloom);
  EXPECT_FALSE(stack.enable_screen_space_reflection);
  EXPECT_TRUE(stack.enable_anti_aliasing);
  EXPECT_TRUE(stack.enable_tone_mapping);
}
