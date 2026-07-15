#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>

#include "Camera.hpp"
#include "RenderGraph.hpp"
#include "RenderLayer.hpp"

using namespace evo_engine;

namespace evo_engine {
class RayCameraHistoryTestAccess {
 public:
  static RayCameraHistoryResources& Acquire(
      Camera& camera, const RayCameraHistoryTechnique technique, const uint64_t scene_handle, const VkExtent3D extent,
      const std::function<RayCameraHistoryResources(VkExtent3D)>& resource_factory) {
    return camera.AcquireRayCameraHistory(technique, scene_handle, extent, resource_factory);
  }

  static void Invalidate(Camera& camera) {
    camera.InvalidateRayCameraHistory();
  }

  static void Release(Camera& camera) {
    camera.ReleaseRayCameraHistory();
  }

  static void Register(RenderLayer& render_layer, const uint64_t handle, const std::shared_ptr<Camera>& camera) {
    render_layer.ray_camera_history_cameras_[handle] = camera;
    render_layer.UpdateRayCameraHistoryPeaks();
  }

  static void Prune(RenderLayer& render_layer, const std::shared_ptr<RenderInstanceStorage>& render_instances) {
    render_layer.PruneRayCameraHistories(render_instances);
  }

  static void Clear(RenderLayer& render_layer) {
    render_layer.ClearRayCameraHistories();
  }

  static void SetFrameCount(Camera& camera, const uint32_t frame_count) {
    camera.frame_count_ = frame_count;
  }

  static std::shared_ptr<DescriptorSet> AcquireOutputDescriptor(
      Camera& camera, const uint32_t frame_index, const uint64_t frame_serial,
      const std::function<std::shared_ptr<DescriptorSet>()>& resource_factory) {
    return camera.AcquireRayCameraOutputDescriptor(frame_index, frame_serial, {}, resource_factory);
  }
};
}  // namespace evo_engine

namespace {
template <typename T>
std::shared_ptr<T> MakeFakeResource() {
  const auto owner = std::make_shared<uint8_t>();
  return std::shared_ptr<T>(owner, reinterpret_cast<T*>(owner.get()));
}

std::shared_ptr<ImageView> MakeFakeImageView(const std::shared_ptr<Image>& image) {
  const auto owner = std::make_shared<std::shared_ptr<Image>>(image);
  return std::shared_ptr<ImageView>(owner, reinterpret_cast<ImageView*>(owner.get()));
}

RayCameraHistoryResources MakeFakeHistory(const VkExtent3D extent) {
  RayCameraHistoryResources result;
  result.extent = extent;
  result.radiance_image = MakeFakeResource<Image>();
  result.radiance_view = MakeFakeImageView(result.radiance_image);
  result.convergence_image = MakeFakeResource<Image>();
  result.convergence_view = MakeFakeImageView(result.convergence_image);
  return result;
}

std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::filesystem::path SourcePath(const std::filesystem::path& relative_path) {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / relative_path;
}
}  // namespace

TEST(RayCameraHistory, CameraUsesOneSlotAndResetsAcrossTechniqueSwitches) {
  Camera camera;
  uint32_t factory_calls = 0;
  const auto factory = [&](const VkExtent3D extent) {
    ++factory_calls;
    return MakeFakeHistory(extent);
  };
  constexpr VkExtent3D extent = {64, 32, 1};

  auto& ray_tracing =
      RayCameraHistoryTestAccess::Acquire(camera, RayCameraHistoryTechnique::RayTracing, 11, extent, factory);
  ray_tracing.valid = true;
  ray_tracing.frame_id = 7;
  RayCameraHistoryTestAccess::SetFrameCount(camera, 7);
  const auto radiance = ray_tracing.radiance_image;

  auto& ray_query =
      RayCameraHistoryTestAccess::Acquire(camera, RayCameraHistoryTechnique::RayQuery, 11, extent, factory);
  EXPECT_EQ(ray_query.radiance_image, radiance);
  EXPECT_EQ(ray_query.technique, RayCameraHistoryTechnique::RayQuery);
  EXPECT_FALSE(ray_query.valid);
  EXPECT_EQ(ray_query.frame_id, 0u);
  EXPECT_EQ(camera.GetFrameCount(), 0u);
  ray_query.valid = true;
  ray_query.frame_id = 3;

  auto& reused = RayCameraHistoryTestAccess::Acquire(camera, RayCameraHistoryTechnique::RayQuery, 11, extent, factory);
  EXPECT_EQ(reused.radiance_image, radiance);
  EXPECT_TRUE(reused.valid);
  EXPECT_EQ(reused.frame_id, 3u);
  RayCameraHistoryTestAccess::SetFrameCount(camera, 3);

  auto& switched_back =
      RayCameraHistoryTestAccess::Acquire(camera, RayCameraHistoryTechnique::RayTracing, 11, extent, factory);
  EXPECT_EQ(switched_back.radiance_image, radiance);
  EXPECT_EQ(switched_back.technique, RayCameraHistoryTechnique::RayTracing);
  EXPECT_FALSE(switched_back.valid);
  EXPECT_EQ(switched_back.frame_id, 0u);
  EXPECT_EQ(camera.GetFrameCount(), 0u);
  EXPECT_EQ(factory_calls, 1u);

  const auto stats = camera.GetRayCameraHistoryStats();
  EXPECT_EQ(stats.live_camera_count, 1u);
  EXPECT_EQ(stats.live_history_count, 1u);
  EXPECT_EQ(stats.live_ray_tracing_history_count, 1u);
  EXPECT_EQ(stats.live_ray_query_history_count, 0u);
  EXPECT_EQ(stats.valid_history_count, 0u);
  EXPECT_EQ(stats.radiance_image_count, 1u);
  EXPECT_EQ(stats.convergence_image_count, 1u);
  EXPECT_EQ(stats.radiance_view_count, 1u);
  EXPECT_EQ(stats.convergence_view_count, 1u);
  EXPECT_EQ(stats.live_byte_size, 64u * 32u * sizeof(glm::vec4) * 2u);
  EXPECT_EQ(stats.creation_count, 1u);
  EXPECT_EQ(stats.reuse_count, 3u);
  EXPECT_EQ(stats.invalidation_count, 2u);
  EXPECT_EQ(stats.peak_live_history_count, 1u);
}

TEST(RayCameraHistory, OutputDescriptorsAreCachedPerFrameSlotWithinOneHistoryGeneration) {
  Camera camera;
  auto& history = RayCameraHistoryTestAccess::Acquire(camera, RayCameraHistoryTechnique::RayTracing, 1, {64, 32, 1},
                                                      MakeFakeHistory);
  const auto generation = history.resource_generation;
  uint32_t factory_calls = 0;
  const auto factory = [&]() {
    ++factory_calls;
    return MakeFakeResource<DescriptorSet>();
  };

  auto slot_0 = RayCameraHistoryTestAccess::AcquireOutputDescriptor(camera, 0, 10, factory);
  auto slot_1 = RayCameraHistoryTestAccess::AcquireOutputDescriptor(camera, 1, 11, factory);
  EXPECT_EQ(factory_calls, 2u);
  EXPECT_NE(slot_0, slot_1);
  EXPECT_EQ(RayCameraHistoryTestAccess::AcquireOutputDescriptor(camera, 0, 12, factory), slot_0);
  EXPECT_NE(RayCameraHistoryTestAccess::AcquireOutputDescriptor(camera, 0, 12, factory), slot_0);
  EXPECT_EQ(factory_calls, 3u);

  auto& switched =
      RayCameraHistoryTestAccess::Acquire(camera, RayCameraHistoryTechnique::RayQuery, 1, {64, 32, 1}, MakeFakeHistory);
  EXPECT_EQ(switched.resource_generation, generation);
  EXPECT_EQ(RayCameraHistoryTestAccess::AcquireOutputDescriptor(camera, 1, 13, factory), slot_1);
  const auto stats = camera.GetRayCameraHistoryStats();
  EXPECT_EQ(stats.live_history_count, 1u);
  EXPECT_EQ(stats.live_output_descriptor_count, 2u);
  EXPECT_EQ(stats.output_descriptor_creation_count, 3u);
  EXPECT_EQ(stats.output_descriptor_reuse_count, 3u);
  EXPECT_EQ(stats.peak_live_output_descriptor_count, 2u);

  camera.Resize({32, 16});
  auto& resized =
      RayCameraHistoryTestAccess::Acquire(camera, RayCameraHistoryTechnique::RayQuery, 1, {32, 16, 1}, MakeFakeHistory);
  EXPECT_GT(resized.resource_generation, generation);
  EXPECT_EQ(camera.GetRayCameraHistoryStats().live_output_descriptor_count, 0u);
}

TEST(RayCameraHistory, InvalidationAndSceneChangesDoNotResetUnrelatedCameras) {
  Camera camera_a;
  Camera camera_b;
  const auto factory = [](const VkExtent3D extent) {
    return MakeFakeHistory(extent);
  };
  constexpr VkExtent3D extent = {32, 16, 1};
  auto& history_a =
      RayCameraHistoryTestAccess::Acquire(camera_a, RayCameraHistoryTechnique::RayTracing, 1, extent, factory);
  auto& history_b =
      RayCameraHistoryTestAccess::Acquire(camera_b, RayCameraHistoryTechnique::RayTracing, 1, extent, factory);
  history_a.valid = true;
  history_a.frame_id = 4;
  history_b.valid = true;
  history_b.frame_id = 9;

  RayCameraHistoryTestAccess::Invalidate(camera_a);
  EXPECT_FALSE(history_a.valid);
  EXPECT_EQ(history_a.frame_id, 0u);
  EXPECT_TRUE(history_b.valid);
  EXPECT_EQ(history_b.frame_id, 9u);

  history_a.valid = true;
  history_a.frame_id = 2;
  const auto radiance = history_a.radiance_image;
  auto& new_scene =
      RayCameraHistoryTestAccess::Acquire(camera_a, RayCameraHistoryTechnique::RayTracing, 2, extent, factory);
  EXPECT_EQ(new_scene.radiance_image, radiance);
  EXPECT_FALSE(new_scene.valid);
  EXPECT_EQ(new_scene.frame_id, 0u);
  EXPECT_TRUE(history_b.valid);
  new_scene.valid = true;
  auto& ray_query =
      RayCameraHistoryTestAccess::Acquire(camera_a, RayCameraHistoryTechnique::RayQuery, 2, extent, factory);
  EXPECT_FALSE(ray_query.valid);
  EXPECT_TRUE(history_b.valid);
  EXPECT_EQ(camera_a.GetRayCameraHistoryStats().creation_count, 1u);
  EXPECT_EQ(camera_a.GetRayCameraHistoryStats().invalidation_count, 3u);
}

TEST(RayCameraHistory, ResizeAndReleaseRemainBoundedDuringCameraChurn) {
  const auto factory = [](const VkExtent3D extent) {
    return MakeFakeHistory(extent);
  };
  for (uint32_t iteration = 0; iteration < 256; ++iteration) {
    Camera camera;
    auto& ray_tracing = RayCameraHistoryTestAccess::Acquire(camera, RayCameraHistoryTechnique::RayTracing, iteration,
                                                            {16, 16, 1}, factory);
    const std::weak_ptr<Image> old_radiance = ray_tracing.radiance_image;
    auto& ray_query = RayCameraHistoryTestAccess::Acquire(camera, RayCameraHistoryTechnique::RayQuery, iteration,
                                                          {16, 16, 1}, factory);
    EXPECT_EQ(ray_query.radiance_image, ray_tracing.radiance_image);
    EXPECT_LE(camera.GetRayCameraHistoryStats().live_history_count, 1u);

    camera.Resize({32, 16});
    EXPECT_TRUE(old_radiance.expired());
    EXPECT_EQ(camera.GetRayCameraHistoryStats().live_history_count, 0u);
    EXPECT_EQ(camera.GetRayCameraHistoryStats().retirement_count, 1u);

    RayCameraHistoryTestAccess::Acquire(camera, RayCameraHistoryTechnique::RayTracing, iteration, {32, 16, 1}, factory);
    RayCameraHistoryTestAccess::Acquire(camera, RayCameraHistoryTechnique::RayQuery, iteration, {32, 16, 1}, factory);
    EXPECT_EQ(camera.GetRayCameraHistoryStats().live_history_count, 1u);
    EXPECT_EQ(camera.GetRayCameraHistoryStats().creation_count, 2u);
    EXPECT_EQ(camera.GetRayCameraHistoryStats().retirement_count, 1u);

    RayCameraHistoryTestAccess::Release(camera);
    EXPECT_EQ(camera.GetRayCameraHistoryStats().live_history_count, 0u);
    EXPECT_EQ(camera.GetRayCameraHistoryStats().retirement_count, 2u);
  }
}

TEST(RayCameraHistory, CameraDestructionReleasesOwnedHistory) {
  std::weak_ptr<Image> radiance;
  std::weak_ptr<ImageView> convergence_view;
  {
    Camera camera;
    auto& history = RayCameraHistoryTestAccess::Acquire(camera, RayCameraHistoryTechnique::RayTracing, 1, {16, 16, 1},
                                                        MakeFakeHistory);
    radiance = history.radiance_image;
    convergence_view = history.convergence_view;
  }
  EXPECT_TRUE(radiance.expired());
  EXPECT_TRUE(convergence_view.expired());
}

TEST(RayCameraHistory, RenderLayerPreservesCumulativeCountersAfterPruneAndClear) {
  const auto factory = [](const VkExtent3D extent) {
    return MakeFakeHistory(extent);
  };
  RenderLayer render_layer;
  auto camera_a = std::make_shared<Camera>();
  auto camera_b = std::make_shared<Camera>();
  RayCameraHistoryTestAccess::Acquire(*camera_a, RayCameraHistoryTechnique::RayTracing, 1, {16, 16, 1}, factory);
  RayCameraHistoryTestAccess::Acquire(*camera_b, RayCameraHistoryTechnique::RayQuery, 1, {16, 16, 1}, factory);
  RayCameraHistoryTestAccess::Register(render_layer, 1, camera_a);
  RayCameraHistoryTestAccess::Register(render_layer, 2, camera_b);

  auto stats = render_layer.GetRayCameraHistoryStats();
  EXPECT_EQ(stats.live_camera_count, 2u);
  EXPECT_EQ(stats.live_history_count, 2u);
  EXPECT_EQ(stats.peak_live_history_count, 2u);
  EXPECT_EQ(stats.peak_live_byte_size, 2u * 16u * 16u * sizeof(glm::vec4) * 2u);
  EXPECT_EQ(stats.creation_count, 2u);

  RayCameraHistoryTestAccess::Clear(render_layer);
  stats = render_layer.GetRayCameraHistoryStats();
  EXPECT_EQ(stats.live_camera_count, 0u);
  EXPECT_EQ(stats.live_history_count, 0u);
  EXPECT_EQ(stats.peak_live_history_count, 2u);
  EXPECT_EQ(stats.peak_live_byte_size, 2u * 16u * 16u * sizeof(glm::vec4) * 2u);
  EXPECT_EQ(stats.creation_count, 2u);
  EXPECT_EQ(stats.retirement_count, 2u);

  auto camera_c = std::make_shared<Camera>();
  RayCameraHistoryTestAccess::Acquire(*camera_c, RayCameraHistoryTechnique::RayTracing, 1, {8, 8, 1}, factory);
  RayCameraHistoryTestAccess::Register(render_layer, camera_c->GetHandle().GetValue(), camera_c);
  RayCameraHistoryTestAccess::Prune(render_layer, {});
  stats = render_layer.GetRayCameraHistoryStats();
  EXPECT_EQ(stats.live_history_count, 0u);
  EXPECT_EQ(stats.creation_count, 3u);
  EXPECT_EQ(stats.retirement_count, 3u);
}

TEST(RayCameraHistory, TransientStoreKeepsSubmittedViewsAliveAfterCameraRelease) {
  Camera camera;
  const auto factory = [](const VkExtent3D extent) {
    return MakeFakeHistory(extent);
  };
  auto& history =
      RayCameraHistoryTestAccess::Acquire(camera, RayCameraHistoryTechnique::RayTracing, 1, {16, 16, 1}, factory);
  std::weak_ptr<Image> radiance_image = history.radiance_image;
  std::weak_ptr<Image> convergence_image = history.convergence_image;
  std::weak_ptr<ImageView> radiance_view = history.radiance_view;
  std::weak_ptr<ImageView> convergence_view = history.convergence_view;
  RenderGraphTransientResourceStore submitted_resources;
  submitted_resources.RetainImageView(history.radiance_view);
  submitted_resources.RetainImageView(history.convergence_view);

  RayCameraHistoryTestAccess::Release(camera);
  EXPECT_FALSE(radiance_image.expired());
  EXPECT_FALSE(convergence_image.expired());
  EXPECT_FALSE(radiance_view.expired());
  EXPECT_FALSE(convergence_view.expired());
  submitted_resources.Clear();
  EXPECT_TRUE(radiance_image.expired());
  EXPECT_TRUE(convergence_image.expired());
  EXPECT_TRUE(radiance_view.expired());
  EXPECT_TRUE(convergence_view.expired());
}

TEST(RayCameraHistory, TransientStoreKeepsImportedImageAliveIndependentlyOfItsView) {
  auto image = MakeFakeResource<Image>();
  auto view = MakeFakeResource<ImageView>();
  const std::weak_ptr<Image> weak_image = image;
  const std::weak_ptr<ImageView> weak_view = view;
  RenderGraphTransientResourceStore submitted_resources;
  submitted_resources.RetainImage(image);
  submitted_resources.RetainImageView(view);
  image.reset();
  view.reset();

  EXPECT_FALSE(weak_image.expired());
  EXPECT_FALSE(weak_view.expired());
  submitted_resources.Clear();
  EXPECT_TRUE(weak_image.expired());
  EXPECT_TRUE(weak_view.expired());
}

TEST(RayCameraHistory, RuntimeWiringHasNoStaticHistoryMapAndClearsAfterGpuDrain) {
  const auto camera_header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Rendering/Camera.hpp"));
  const auto pass_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderPasses/RayTracingCameraPass.cpp"));
  const auto render_layer_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));

  EXPECT_NE(camera_header.find("RayCameraHistoryResources ray_camera_history_"), std::string::npos);
  EXPECT_EQ(pass_source.find("static std::unordered_map"), std::string::npos);
  EXPECT_NE(pass_source.find("RetainImageView(history_resources.radiance_view)"), std::string::npos);
  EXPECT_NE(pass_source.find("RetainImageView(history_resources.convergence_view)"), std::string::npos);
  EXPECT_NE(pass_source.find("RetainImage(render_texture->GetColorImage())"), std::string::npos);
  EXPECT_EQ(pass_source.find("Platform::EverythingBarrier"), std::string::npos);
  EXPECT_NE(render_layer_source.find("render_graph_transient_resource_stores_.at(current_frame_index)"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("!camera->ray_camera_history_owner_alive_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("if (!render_texture)"), std::string::npos);
  const auto shutdown = render_layer_source.find("void RenderLayer::OnDestroy()");
  const auto drain = render_layer_source.find("Platform::DrainGpuResourceWork()", shutdown);
  const auto clear = render_layer_source.find("ClearRayCameraHistories()", shutdown);
  ASSERT_NE(shutdown, std::string::npos);
  ASSERT_NE(drain, std::string::npos);
  ASSERT_NE(clear, std::string::npos);
  EXPECT_LT(drain, clear);
}
