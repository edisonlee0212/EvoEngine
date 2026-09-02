#include "StarDemoCamera.hpp"

#include "AssetManager.hpp"
#include "Camera.hpp"
#include "PostProcessingStack.hpp"

using namespace evo_engine;
using namespace universe_package;

namespace {
template <typename T>
std::shared_ptr<T> CopySettings(const std::shared_ptr<T>& settings) {
  return settings ? std::make_shared<T>(*settings) : nullptr;
}
}  // namespace

StarDemoCameraOverride::~StarDemoCameraOverride() {
  Restore();
}

void StarDemoCameraOverride::Apply(const std::shared_ptr<Camera>& camera) {
  if (camera && camera_.lock() == camera && override_ &&
      camera->post_processing_stack_ref.Peek<PostProcessingStack>() == override_) {
    override_->enable_tone_mapping = false;
    camera->camera_settings.far_distance = 1000000;
    return;
  }
  Restore();
  if (!camera)
    return;
  const auto original = camera->post_processing_stack_ref.Get<PostProcessingStack>();
  auto replacement = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
  if (!replacement)
    return;
  if (original) {
    replacement->enable_ambient_occlusion = original->enable_ambient_occlusion;
    replacement->enable_bloom = original->enable_bloom;
    replacement->enable_screen_space_reflection = original->enable_screen_space_reflection;
    replacement->enable_anti_aliasing = original->enable_anti_aliasing;
    replacement->ambient_occlusion = CopySettings(original->ambient_occlusion);
    replacement->bloom = CopySettings(original->bloom);
    replacement->screen_space_reflection = CopySettings(original->screen_space_reflection);
    replacement->anti_aliasing = CopySettings(original->anti_aliasing);
    replacement->tone_mapping = CopySettings(original->tone_mapping);
  }
  replacement->enable_tone_mapping = false;
  original_ = camera->post_processing_stack_ref;
  camera_ = camera;
  original_far_distance_ = camera->camera_settings.far_distance;
  camera->camera_settings.far_distance = 1000000;
  override_ = std::move(replacement);
  camera->post_processing_stack_ref = override_;
}

void StarDemoCameraOverride::Restore() {
  if (const auto camera = camera_.lock()) {
    if (override_ && camera->post_processing_stack_ref.Peek<PostProcessingStack>() == override_)
      camera->post_processing_stack_ref = original_;
    if (camera->camera_settings.far_distance == 1000000)
      camera->camera_settings.far_distance = original_far_distance_;
  }
  camera_.reset();
  original_.Clear();
  override_.reset();
}
