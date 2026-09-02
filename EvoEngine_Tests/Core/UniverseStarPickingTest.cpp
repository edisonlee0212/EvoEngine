#include <gtest/gtest.h>
#include <fstream>
#include <iterator>

#include "Application.hpp"
#include "GeometryStorage.hpp"
#include "Jobs.hpp"
#include "RenderLayer.hpp"
#include "RenderTexture.hpp"
#include "Shader.hpp"
#include "StarPicking.hpp"
#include "TextureStorage.hpp"
#include "UniverseLayer.hpp"

namespace evo_engine {
class PlatformLifecycleTestAccess {
 public:
  static void Initialize(const ApplicationInitializationSettings& settings) {
    Platform::Initialize(settings);
  }
  static void Destroy() {
    Platform::OnDestroy();
  }
};
}  // namespace evo_engine

using namespace universe_package;

namespace {
StarPickRequest Request(const std::shared_ptr<StarCluster>& cluster, const uint64_t frame = 1) {
  StarPickRequest request;
  request.camera_handle = 100;
  request.population_revision = 1;
  request.frame = frame;
  request.valid = true;
  request.cursor_uv = glm::vec2(0.5f);
  request.display_size = glm::vec2(100);
  request.ranges.push_back({7, cluster->seed, 0, cluster->GetStarCount(), cluster});
  return request;
}

StarPickResult Hit(const uint32_t index = 0) {
  return {{0, 0, -5, 0.1}, 4.9, index, 1};
}

std::shared_ptr<StarCluster> TestCluster(const uint32_t count = 10) {
  auto cluster = std::make_shared<StarCluster>();
  cluster->SetStarCount(count);
  return cluster;
}

class PickingGpuFixture {
 public:
  PickingGpuFixture() {
    application_ = std::make_unique<evo_engine::Application>();
    application_->PushLayer<RenderLayer>("Render Layer");
    Jobs::Initialize(2);
    ApplicationInitializationSettings settings;
    settings.load_default_resources = false;
    settings.load_project_assets = false;
    settings.load_project_start_scene = false;
    settings.enable_runtime_packages = false;
    settings.graphics_settings.use_mesh_shader = false;
    settings.graphics_settings.use_ray_tracing = false;
    const_cast<ApplicationInitializationSettings&>(application_->GetApplicationInfo()) = settings;
    evo_engine::PlatformLifecycleTestAccess::Initialize(settings);
  }
  ~PickingGpuFixture() {
    application_->PopLayer<RenderLayer>();
    Platform::DrainGpuResourceWork();
    TextureStorage::OnDestroy();
    GeometryStorage::OnDestroy();
    evo_engine::PlatformLifecycleTestAccess::Destroy();
    Jobs::OnDestroy();
  }

 private:
  std::unique_ptr<evo_engine::Application> application_;
};

std::shared_ptr<Buffer> PickTestBuffer(const size_t size) {
  VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  info.size = size;
  info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  VmaAllocationCreateInfo allocation{};
  allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  return std::make_shared<Buffer>(info, allocation);
}

StarPickResult ReferencePick(const std::vector<StarClusterGpuResult>& stars, const CameraInfoBlock& camera,
                             const StarPickRequest& request, const float depth) {
  StarPickResult result;
  if (!request.valid)
    return result;
  const auto ndc = glm::dvec2(request.cursor_uv) * 2.0 - 1.0;
  auto near_point = glm::dmat4(camera.inverse_projection) * glm::dvec4(ndc, 0, 1);
  auto far_point = glm::dmat4(camera.inverse_projection) * glm::dvec4(ndc, 1, 1);
  near_point /= near_point.w;
  far_point /= far_point.w;
  const auto delta = glm::dvec3(far_point - near_point);
  if (std::abs(delta.z) < 1e-12)
    return result;
  for (uint32_t index = 0; index < stars.size(); ++index) {
    const auto& star = stars[index];
    const auto relative = glm::dvec3(star.world_position_radius) - glm::dvec3(camera.inverse_view[3]);
    const auto center = glm::dvec3(glm::dmat4(camera.view) * glm::dvec4(relative, 0));
    const auto fraction = (center.z - near_point.z) / delta.z;
    if (fraction < 0 || fraction > 1)
      continue;
    const auto point = glm::dvec3(near_point) + delta * fraction;
    const auto clip = glm::dmat4(camera.projection) * glm::dvec4(point, 1);
    if (clip.w <= 0 || clip.z / clip.w > depth + 1e-6)
      continue;
    const auto per_pixel =
        2.0 * std::abs(clip.w) /
        (glm::abs(glm::dvec2(camera.projection[0][0], camera.projection[1][1])) * glm::dvec2(request.display_size));
    const auto radius = glm::max(glm::dvec2(std::abs(star.world_position_radius.w)),
                                 per_pixel * static_cast<double>(request.minimum_radius));
    if (radius.x <= 0 || radius.y <= 0)
      continue;
    const auto offset = (glm::dvec2(point) - glm::dvec2(center)) / radius;
    if (glm::dot(offset, offset) >= 1)
      continue;
    const auto distance = glm::length(point - glm::dvec3(near_point));
    if (!result.valid || distance < result.distance)
      result = {star.world_position_radius, distance, index, 1};
  }
  return result;
}
}  // namespace

TEST(UniverseStarPicking, ResultAndPushConstantLayoutsMatchShader) {
  EXPECT_EQ(sizeof(StarPickResult), 48u);
  EXPECT_EQ(offsetof(StarPickResult, distance), 32u);
  EXPECT_EQ(offsetof(StarPickResult, index), 40u);
  EXPECT_EQ(offsetof(StarPickResult, valid), 44u);
  EXPECT_EQ(sizeof(StarPickPushConstant), 48u);
  EXPECT_EQ(offsetof(StarPickPushConstant, cursor_uv), 8u);
  EXPECT_EQ(offsetof(StarPickPushConstant, display_size), 16u);
  EXPECT_EQ(offsetof(StarPickPushConstant, final_pass), 32u);
}

TEST(UniverseStarPicking, ClickMissPreservesSelectionUntilNextValidSelection) {
  const auto cluster = TestCluster();
  StarPickState state;
  state.Update(Request(cluster), true);
  state.Complete(state.current, Hit(2));
  ASSERT_EQ(state.selected.ordinal, 2u);
  state.Update(Request(cluster, 2), true);
  state.Complete(state.current, {});
  EXPECT_EQ(state.selected.ordinal, 2u);
  EXPECT_FALSE(state.hovered.result.valid);
  state.Update(Request(cluster, 3), true);
  const auto pending = state.current;
  state.Complete(pending, Hit(4));
  EXPECT_TRUE(state.selected.result.valid);
  EXPECT_EQ(state.selected.ordinal, 4u);
  EXPECT_EQ(state.hovered.ordinal, 4u);
}

TEST(UniverseStarPicking, OnlyLatestClickAndNewestHoverCompletionArePublished) {
  const auto cluster = TestCluster();
  StarPickState state;
  state.Update(Request(cluster), true);
  const auto older = state.current;
  state.Update(Request(cluster, 2), true);
  const auto newer = state.current;
  state.Complete(older, Hit(1));
  EXPECT_FALSE(state.selected.result.valid);
  state.Complete(newer, Hit(2));
  state.Complete(older, Hit(1));
  EXPECT_EQ(state.selected.ordinal, 2u);
  EXPECT_EQ(state.hovered.ordinal, 2u);
  EXPECT_EQ(state.hovered.frame, 2u);
}

TEST(UniverseStarPicking, InteractionLockPreservesSelectionAndKeepsGpuRequestValid) {
  const auto cluster = TestCluster();
  StarPickState state;
  state.Update(Request(cluster), true);
  state.Complete(state.current, Hit(2));
  ASSERT_EQ(state.selected.ordinal, 2u);
  ASSERT_TRUE(state.hovered.result.valid);
  state.Update(Request(cluster, 2), true);
  const auto before_lock = state.current;
  state.SetInteractionLocked(true);
  EXPECT_TRUE(state.current.valid);
  EXPECT_FALSE(state.hovered.result.valid);
  EXPECT_EQ(state.current.click, 0u);
  EXPECT_GT(state.generation, before_lock.generation);
  const auto locked_generation = state.generation;
  state.SetInteractionLocked(true);
  EXPECT_EQ(state.generation, locked_generation);
  state.Complete(before_lock, Hit(3));
  state.Update(Request(cluster, 3), true);
  EXPECT_TRUE(state.current.valid);
  EXPECT_EQ(state.current.click, 0u);
  const auto while_locked = state.current;
  state.Complete(while_locked, Hit(4));
  EXPECT_EQ(state.selected.ordinal, 2u);
  EXPECT_FALSE(state.hovered.result.valid);
  state.SetInteractionLocked(false);
  EXPECT_TRUE(state.current.valid);
  EXPECT_GT(state.generation, locked_generation);
  state.Complete(before_lock, Hit(3));
  state.Complete(while_locked, Hit(4));
  EXPECT_EQ(state.selected.ordinal, 2u);
  EXPECT_FALSE(state.hovered.result.valid);
  state.Update(Request(cluster, 4), true);
  state.Complete(state.current, Hit(5));
  EXPECT_EQ(state.selected.ordinal, 5u);
  EXPECT_EQ(state.hovered.ordinal, 5u);
}

TEST(UniverseStarPicking, CameraViewportPopulationAndToleranceRejectStaleResults) {
  const auto cluster = TestCluster();
  const auto camera = std::make_shared<Camera>();
  const auto replacement_camera = std::make_shared<Camera>();
  for (uint32_t change = 0; change < 7; ++change) {
    StarPickState state;
    auto initial = Request(cluster);
    initial.camera = camera;
    state.Update(initial, true);
    const auto pending = state.current;
    auto next = Request(cluster, 2);
    if (change == 6)
      ++next.reference_generation;
    next.camera = camera;
    if (change == 0)
      ++next.camera_handle;
    if (change == 1)
      next.display_size.x *= 2;
    if (change == 2)
      ++next.population_revision;
    if (change == 3)
      next.minimum_radius = 4;
    if (change == 4)
      next.image_origin.x = 50;
    if (change == 5)
      next.camera = replacement_camera;  // Same reported handle, different camera object.
    state.Update(next, false);
    state.Complete(pending, Hit());
    EXPECT_FALSE(state.selected.result.valid) << change;
    EXPECT_FALSE(state.hovered.result.valid) << change;
  }
}

TEST(UniverseStarPicking, LeavingImageClearsHoverButPreservesPendingClick) {
  const auto cluster = TestCluster();
  StarPickState state;
  state.Update(Request(cluster), true);
  const auto pending = state.current;
  auto outside = Request(cluster, 2);
  outside.valid = false;
  state.Update(outside, false);
  state.Complete(pending, Hit(3));
  EXPECT_FALSE(state.hovered.result.valid);
  EXPECT_TRUE(state.selected.result.valid);
  EXPECT_EQ(state.selected.ordinal, 3u);
}

TEST(UniverseStarPicking, SelectionRecordSurvivesPackedOffsetChangesAndIdentityRemoval) {
  const auto cluster = TestCluster();
  for (uint32_t change = 0; change < 4; ++change) {
    StarPickState state;
    auto request = Request(cluster);
    request.ranges[0].offset = 100;
    state.Update(request, true);
    state.Complete(state.current, Hit(103));
    ASSERT_EQ(state.selected.ordinal, 3u);
    request.ranges[0].offset = 7;
    ++request.population_revision;
    state.Update(request, false);
    EXPECT_EQ(state.selected.ordinal, 3u);
    EXPECT_TRUE(state.selected.result.valid);
    if (change == 0)
      ++request.ranges[0].seed;
    if (change == 1)
      request.ranges[0].count = 3;
    if (change == 2)
      request.ranges.clear();  // Disabled/deleted clusters are omitted from packed ranges.
    if (change == 3)
      ++request.ranges[0].identity;
    state.Update(request, false);
    EXPECT_TRUE(state.selected.result.valid) << change;
    EXPECT_EQ(state.selected.ordinal, 3u);
  }
}

TEST(UniverseStarPicking, ExpiredClusterAndSceneResetCannotRepublishSelection) {
  auto cluster = TestCluster();
  StarPicker picker;
  picker.state.Update(Request(cluster), true);
  const auto pending = picker.state.current;
  cluster.reset();
  picker.state.Complete(pending, Hit());
  EXPECT_FALSE(picker.state.hovered.result.valid);
  const auto old_generation = picker.state.generation;
  picker.Reset();
  EXPECT_GT(picker.state.generation, old_generation);
  picker.state.Complete(pending, Hit());
  EXPECT_FALSE(picker.state.selected.result.valid);
  EXPECT_FALSE(picker.Pending());
}

TEST(UniverseStarPicking, RealGpuReductionMatchesReferenceAcrossCountsCamerasAndDepth) {
  PickingGpuFixture gpu;
  if (!Platform::GetSelectedPhysicalDevice()->features.shaderFloat64)
    GTEST_SKIP() << "shaderFloat64 unavailable";
  const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
  Shader::RegisterShaderIncludePath(root / "EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules");
  const auto shader_path =
      root / "EvoEngine_Packages/Universe/Internals/UniverseResources/Shaders/Compute/StarPick.slang";
  std::ifstream shader_file(shader_path);
  ASSERT_TRUE(shader_file.good());
  const std::string shader_source{std::istreambuf_iterator<char>(shader_file), std::istreambuf_iterator<char>()};
  for (const bool reduction : {false, true}) {
    ShaderReflectionInfo reflection;
    std::string diagnostics;
    const auto source =
        Platform::GetShaderGlobalDefines() + (reduction ? "\n#define EE_STAR_PICK_REDUCE 1\n" : "\n") + shader_source;
    ASSERT_TRUE(Shader::ReflectSlang(ShaderType::Compute, source, reflection, diagnostics, shader_path)) << diagnostics;
    EXPECT_EQ(reflection.compute_thread_group_size, (std::array<uint32_t, 3>{256, 1, 1}));
    ASSERT_EQ(reflection.push_constant_ranges.size(), 1u);
    EXPECT_EQ(reflection.push_constant_ranges.front().size, sizeof(StarPickPushConstant));
    for (const uint32_t binding : {0u, 2u, 3u}) {
      const auto found = std::find_if(reflection.descriptor_bindings.begin(), reflection.descriptor_bindings.end(),
                                      [&](const auto& item) {
                                        return item.set == 1 && item.binding == binding;
                                      });
      ASSERT_NE(found, reflection.descriptor_bindings.end()) << binding;
      EXPECT_EQ(found->descriptor_type, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
    }
  }
  auto layout = std::make_shared<DescriptorSetLayout>();
  layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->Initialize();
  StarPicker picker;
  ASSERT_TRUE(picker.Initialize(layout, shader_path));
  picker.EnsureResources(2, 500000);
  auto camera = std::make_shared<Camera>();
  camera->OnCreate();
  camera->Resize({64, 64});
  auto camera_buffer = PickTestBuffer(2 * sizeof(CameraInfoBlock));
  auto camera_descriptor = std::make_shared<DescriptorSet>(layout);
  camera_descriptor->UpdateBufferDescriptorBinding(2, camera_buffer);
  auto star_buffer = PickTestBuffer(500000 * sizeof(StarClusterGpuResult));
  const auto first = TestCluster(250000);
  const auto second = TestCluster(250000);
  uint64_t frame = 0;
  const auto run = [&](const std::vector<StarClusterGpuResult>& stars, CameraInfoBlock block, StarPickRequest request,
                       const float clear_depth = 1.0f) {
    block.inverse_projection = glm::inverse(block.projection);
    block.resolution = glm::vec2(64);
    camera_buffer->Upload(std::array{CameraInfoBlock{}, block});
    if (!stars.empty())
      star_buffer->UploadVector(stars);
    request.camera = camera;
    request.camera_handle = camera->GetHandle().GetValue();
    request.frame = ++frame;
    request.population_revision = frame;
    picker.state.Update(request, false);
    request = picker.state.current;
    const auto expected = ReferencePick(stars, block, request, clear_depth);
    const auto slot = static_cast<uint32_t>(frame % 2);
    Platform::ImmediateSubmit([&](const VkCommandBuffer command) {
      const auto target = camera->GetRenderTexture();
      target->GetDepthImage()->TransitImageLayout(command, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
      auto depth = target->GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
      depth.clearValue.depthStencil.depth = clear_depth;
      VkRenderingInfo rendering{VK_STRUCTURE_TYPE_RENDERING_INFO};
      rendering.renderArea = {{0, 0}, {64, 64}};
      rendering.layerCount = 1;
      rendering.pDepthAttachment = &depth;
      Platform::BeginRendering(command, rendering);
      Platform::EndRendering(command);
      picker.Record(command, slot, star_buffer, camera, 1, camera_descriptor, request);
    });
    EXPECT_TRUE(picker.Pending());
    picker.Consume(slot);
    EXPECT_FALSE(picker.Pending());
    const auto actual = picker.state.hovered.result;
    EXPECT_EQ(actual.valid, expected.valid) << "frame " << frame;
    if (expected.valid) {
      EXPECT_EQ(actual.index, expected.index) << "frame " << frame;
      EXPECT_NEAR(actual.distance, expected.distance, 1e-8) << "frame " << frame;
      EXPECT_EQ(actual.position_radius, expected.position_radius);
    }
    return actual;
  };
  CameraInfoBlock block{};
  block.view = block.inverse_view = glm::mat4(1);
  block.projection = glm::perspectiveRH_ZO(glm::half_pi<float>(), 1.0f, 0.1f, 100.0f);
  for (const uint32_t count : {0u, 1u, 255u, 256u, 257u, 500000u}) {
    std::vector<StarClusterGpuResult> stars(count);
    for (auto& star : stars)
      star.world_position_radius = {100, 100, -10, 0.01};
    if (count != 0)
      stars.back().world_position_radius = {0, 0, -5, 0.01};
    auto request = Request(first);
    request.ranges = {{7, first->seed, 0, (std::min)(count, 250000u), first}};
    if (count > 250000)
      request.ranges.push_back({8, second->seed, 250000, count - 250000, second});
    const auto hit = run(stars, block, request);
    if (count != 0)
      EXPECT_EQ(hit.index, count - 1);
    if (count == 500000) {
      EXPECT_EQ(picker.state.hovered.identity, 8u);
      EXPECT_EQ(picker.state.hovered.ordinal, 249999u);
      stars[3] = stars.back();  // Exact tie spans reduction groups; smaller packed index wins.
      EXPECT_EQ(run(stars, block, request).index, 3u);
    }
  }
  for (const bool perspective : {false, true}) {
    block.projection = perspective ? glm::perspectiveRH_ZO(glm::half_pi<float>(), 1.0f, 0.1f, 100.0f)
                                   : glm::orthoRH_ZO(-5.0f, 5.0f, -5.0f, 5.0f, 0.1f, 100.0f);
    auto request = Request(first);
    request.ranges[0].count = 1;
    std::vector<StarClusterGpuResult> stars(1);
    stars[0].world_position_radius = {0.25, 0, -5, 0.001};
    EXPECT_TRUE(run(stars, block, request).valid);  // Expanded 3-display-pixel footprint.
    request.display_size = glm::vec2(200);
    EXPECT_FALSE(run(stars, block, request).valid);
    request.display_size = glm::vec2(100);
    request.minimum_radius = 0;
    EXPECT_FALSE(run(stars, block, request).valid);
    stars[0].world_position_radius = {0, 0, -5, 0.5};
    EXPECT_TRUE(run(stars, block, request).valid);
    EXPECT_FALSE(run(stars, block, request, 0.01f).valid);
    stars[0].world_position_radius.x = 0.499;
    EXPECT_TRUE(run(stars, block, request).valid);
    stars[0].world_position_radius.x = 0.501;
    EXPECT_FALSE(run(stars, block, request).valid);
    stars[0].world_position_radius = {0.2, -0.3, -5, 0.5};
    const auto off_axis_clip = block.projection * glm::vec4(0.2f, -0.3f, -5.0f, 1.0f);
    request.cursor_uv = glm::vec2(off_axis_clip) / off_axis_clip.w * 0.5f + 0.5f;
    EXPECT_TRUE(run(stars, block, request).valid);
    request.cursor_uv = glm::vec2(0.5f);
    stars[0].world_position_radius = {0, 5, -5, 0.5};
    request.cursor_uv.y = 1.0f;  // Display top edge maps to the final framebuffer row, not row 64.
    EXPECT_TRUE(run(stars, block, request).valid);
    EXPECT_FALSE(run(stars, block, request, 0.01f).valid);
    request.cursor_uv = glm::vec2(0.5f);
    stars[0].world_position_radius = {0, 0, -5, 0.5};
    stars.push_back(stars[0]);
    stars[1].world_position_radius.z = -3;
    request.ranges[0].count = 2;
    EXPECT_EQ(run(stars, block, request).index, 1u);
    stars.resize(1);
    request.ranges[0].count = 1;
    request.valid = false;
    EXPECT_FALSE(run(stars, block, request).valid);
    request.valid = true;
    for (const double z : {-0.01, -200.0, 5.0}) {
      stars[0].world_position_radius.z = z;
      EXPECT_FALSE(run(stars, block, request).valid);
    }
    block.inverse_view =
        glm::translate(glm::mat4(1), glm::vec3(1000, 2000, 3000)) * glm::rotate(glm::mat4(1), 0.7f, glm::vec3(0, 1, 0));
    block.view = glm::inverse(block.inverse_view);
    stars[0].world_position_radius = glm::dmat4(block.inverse_view) * glm::dvec4(0, 0, -5, 1);
    stars[0].world_position_radius.w = 0.5;
    EXPECT_TRUE(run(stars, block, request).valid);
    block.view = block.inverse_view = glm::mat4(1);
  }
}
