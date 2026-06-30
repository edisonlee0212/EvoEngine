#include "EvoEngine_SDK_PCH.hpp"

#include "RenderGraph.hpp"
#include "RenderLayer.hpp"
#include "RenderPasses/GaussianSplatPass.hpp"
#include "RenderPasses/PostProcessingPass.hpp"
#include "RenderPasses/RayTracingCameraPass.hpp"
#include "RenderPasses/VolumetricCloudsPass.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <iterator>

using namespace evo_engine;

namespace {
constexpr const char* kTestDebugOutputResource = "Frame.TestDebugOutput";
}  // namespace

TEST(RenderGraph, ExecutesPassesInInsertionOrderAndKeepsDescriptors) {
  RenderGraph graph;
  graph.AddResource({RenderResourceNames::camera_color, RenderResourceType::Image, RenderResourceLifetime::Camera});
  graph.AddResource({RenderResourceNames::camera_color, RenderResourceType::Image, RenderResourceLifetime::Camera});
  graph.AddResource({RenderResourceNames::camera_depth, RenderResourceType::Image, RenderResourceLifetime::Camera});

  int sequence = 0;
  graph.AddPass(
      {"DepthPrepass",
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_depth, RenderResourceUsage::Write, RenderResourceState::DepthAttachment}}},
      [&]() {
        EXPECT_EQ(sequence++, 0);
      });
  graph.AddPass(
      {"Lighting",
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
        {RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
      [&]() {
        EXPECT_EQ(sequence++, 1);
      });

  ASSERT_EQ(graph.GetResources().size(), 2);
  ASSERT_EQ(graph.GetPasses().size(), 2);
  EXPECT_TRUE(graph.HasResource(RenderResourceNames::camera_color));
  EXPECT_EQ(graph.GetPasses()[0].name, "DepthPrepass");
  EXPECT_EQ(graph.GetPasses()[1].resources[1].resource_name, RenderResourceNames::camera_color);
  EXPECT_TRUE(graph.Validate());

  graph.Execute();
  EXPECT_EQ(sequence, 2);
}

TEST(RenderGraph, ValidateRejectsUnknownResources) {
  RenderGraph graph;
  graph.AddPass(
      {"Lighting",
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
      []() {
      });

  EXPECT_FALSE(graph.Validate());
}

TEST(RenderGraph, ValidateRejectsUnknownOrForwardDependencies) {
  RenderGraph graph;
  graph.AddResource({RenderResourceNames::camera_color, RenderResourceType::Image, RenderResourceLifetime::Camera});
  graph.AddPass(
      {"Lighting",
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}},
       {"DepthPrepass"}},
      []() {
      });

  EXPECT_FALSE(graph.Validate());

  graph.AddPass({"DepthPrepass",
                 RenderPassQueue::Graphics,
                 RenderPassScope::Camera,
                 {{RenderResourceNames::camera_color, RenderResourceUsage::Read, RenderResourceState::ShaderRead}}},
                []() {
                });

  EXPECT_FALSE(graph.Validate());
}

TEST(RenderGraph, ValidateRejectsDuplicatePassNames) {
  RenderGraph graph;
  graph.AddResource({RenderResourceNames::camera_color, RenderResourceType::Image, RenderResourceLifetime::Camera});
  graph.AddPass({"Copy",
                 RenderPassQueue::Graphics,
                 RenderPassScope::Camera,
                 {{RenderResourceNames::camera_color, RenderResourceUsage::Read, RenderResourceState::ShaderRead}}},
                []() {
                });
  graph.AddPass(
      {"Copy",
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
      []() {
      });

  EXPECT_FALSE(graph.Validate());
}

TEST(RenderGraph, ClearRemovesResourcesAndPasses) {
  RenderGraph graph;
  graph.AddResource(
      {RenderResourceNames::frame_render_instances, RenderResourceType::Buffer, RenderResourceLifetime::Frame});
  graph.AddPass({"Prepare", RenderPassQueue::Compute, RenderPassScope::Frame, {}}, []() {
  });

  graph.Clear();

  EXPECT_TRUE(graph.GetResources().empty());
  EXPECT_TRUE(graph.GetPasses().empty());
}

TEST(RenderGraph, ReportsPassQueuesForFutureSchedulers) {
  RenderGraph graph;
  graph.AddResource({RenderResourceNames::camera_color, RenderResourceType::Image, RenderResourceLifetime::Camera});
  graph.AddPass(
      {"Raster",
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
      []() {
      });
  graph.AddPass(
      {"Denoise",
       RenderPassQueue::Compute,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::StorageReadWrite}},
       {"Raster"}},
      []() {
      });

  const auto graphics_passes = graph.GetPassIndices(RenderPassQueue::Graphics);
  const auto compute_passes = graph.GetPassIndices(RenderPassQueue::Compute);

  EXPECT_TRUE(graph.UsesQueue(RenderPassQueue::Graphics));
  EXPECT_TRUE(graph.UsesQueue(RenderPassQueue::Compute));
  EXPECT_FALSE(graph.UsesQueue(RenderPassQueue::RayTracing));
  ASSERT_EQ(graphics_passes.size(), 1);
  ASSERT_EQ(compute_passes.size(), 1);
  EXPECT_EQ(graphics_passes[0], 0);
  EXPECT_EQ(compute_passes[0], 1);
  EXPECT_TRUE(graph.Validate());
}

TEST(RenderGraph, CompileBuildsQueueScheduleFromResourceHazards) {
  RenderGraph graph;
  graph.AddResource({RenderResourceNames::camera_depth, RenderResourceType::Image, RenderResourceLifetime::Camera});
  graph.AddResource({RenderResourceNames::camera_color, RenderResourceType::Image, RenderResourceLifetime::Camera});
  graph.AddResource({kTestDebugOutputResource, RenderResourceType::External, RenderResourceLifetime::Frame});
  graph.AddResource({RenderResourceNames::camera_depth_pyramid,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative, 0, 0, 1, 1, 0},
                     "DepthPyramid",
                     1,
                     1,
                     true});
  graph.AddPass(
      {"GBuffer",
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_depth, RenderResourceUsage::Write, RenderResourceState::DepthAttachment},
        {RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
      []() {
      });
  graph.AddPass({"DebugOverlay",
                 RenderPassQueue::Compute,
                 RenderPassScope::Frame,
                 {{kTestDebugOutputResource, RenderResourceUsage::Write, RenderResourceState::General}}},
                []() {
                });
  graph.AddPass({"BuildDepthPyramid",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
                  {RenderResourceNames::camera_depth_pyramid, RenderResourceUsage::Write,
                   RenderResourceState::StorageReadWrite}}},
                []() {
                });
  graph.AddPass(
      {"Lighting",
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
        {RenderResourceNames::camera_depth_pyramid, RenderResourceUsage::Read, RenderResourceState::ShaderRead}}},
      []() {
      });

  RenderGraphCompileContext context;
  context.camera_width = 1280;
  context.camera_height = 720;
  const auto plan = graph.Compile(context);

  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(plan.passes.size(), 4);
  ASSERT_EQ(plan.schedule_steps.size(), 3);
  EXPECT_TRUE(plan.passes[0].schedule_dependency_indices.empty());
  EXPECT_TRUE(plan.passes[1].schedule_dependency_indices.empty());
  ASSERT_EQ(plan.passes[2].resource_dependency_indices.size(), 1);
  EXPECT_EQ(plan.passes[2].resource_dependency_indices[0], 0);
  EXPECT_NE(std::find(plan.passes[3].resource_dependency_indices.begin(),
                      plan.passes[3].resource_dependency_indices.end(), 0),
            plan.passes[3].resource_dependency_indices.end());
  EXPECT_NE(std::find(plan.passes[3].resource_dependency_indices.begin(),
                      plan.passes[3].resource_dependency_indices.end(), 2),
            plan.passes[3].resource_dependency_indices.end());
  EXPECT_EQ(plan.passes[0].schedule_step_index, 0);
  EXPECT_EQ(plan.passes[1].schedule_step_index, 0);
  EXPECT_EQ(plan.passes[2].schedule_step_index, 1);
  EXPECT_EQ(plan.passes[3].schedule_step_index, 2);
  ASSERT_EQ(plan.schedule_steps[0].graphics_pass_indices.size(), 1);
  ASSERT_EQ(plan.schedule_steps[0].compute_pass_indices.size(), 1);
  EXPECT_EQ(plan.schedule_steps[0].graphics_pass_indices[0], 0);
  EXPECT_EQ(plan.schedule_steps[0].compute_pass_indices[0], 1);
  ASSERT_EQ(plan.schedule_steps[1].compute_pass_indices.size(), 1);
  EXPECT_EQ(plan.schedule_steps[1].compute_pass_indices[0], 2);
  ASSERT_EQ(plan.schedule_steps[2].graphics_pass_indices.size(), 1);
  EXPECT_EQ(plan.schedule_steps[2].graphics_pass_indices[0], 3);
}

TEST(RenderGraph, ExecuteUsesCompiledScheduleSteps) {
  RenderGraph graph;
  graph.AddResource({RenderResourceNames::camera_depth, RenderResourceType::Image, RenderResourceLifetime::Camera});
  graph.AddResource({kTestDebugOutputResource, RenderResourceType::External, RenderResourceLifetime::Frame});

  std::vector<int> execution_order;
  graph.AddPass(
      {"Depth",
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_depth, RenderResourceUsage::Write, RenderResourceState::DepthAttachment}}},
      [&]() {
        execution_order.emplace_back(0);
      });
  graph.AddPass({"DepthPyramid",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead}}},
                [&]() {
                  execution_order.emplace_back(1);
                });
  graph.AddPass({"Debug",
                 RenderPassQueue::Compute,
                 RenderPassScope::Frame,
                 {{kTestDebugOutputResource, RenderResourceUsage::Write, RenderResourceState::General}}},
                [&]() {
                  execution_order.emplace_back(2);
                });

  const auto plan = graph.Compile();

  ASSERT_EQ(plan.schedule_steps.size(), 2);
  EXPECT_EQ(plan.passes[0].schedule_step_index, 0);
  EXPECT_EQ(plan.passes[1].schedule_step_index, 1);
  EXPECT_EQ(plan.passes[2].schedule_step_index, 0);

  graph.Execute(plan, {});

  ASSERT_EQ(execution_order.size(), 3);
  EXPECT_EQ(execution_order[0], 0);
  EXPECT_EQ(execution_order[1], 2);
  EXPECT_EQ(execution_order[2], 1);
}

TEST(RenderGraph, CompileTracksResourceLifetimesAndDependencies) {
  constexpr const char* temporary_radiance = "Camera.TemporaryRadiance";
  RenderGraph graph;
  graph.AddResource({RenderResourceNames::camera_color, RenderResourceType::Image, RenderResourceLifetime::Camera});
  graph.AddResource({temporary_radiance,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "RGBA16F",
                     1,
                     1,
                     true});
  graph.AddResource({RenderResourceNames::camera_color_history,
                     RenderResourceType::Image,
                     RenderResourceLifetime::History,
                     {RenderResourceSizeMode::CameraRelative},
                     "Color",
                     1,
                     2,
                     true});
  graph.AddPass({"Raster",
                 RenderPassQueue::Graphics,
                 RenderPassScope::Camera,
                 {{RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment},
                  {temporary_radiance, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}}},
                []() {
                });
  graph.AddPass(
      {"TemporalResolve",
       RenderPassQueue::Compute,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
        {temporary_radiance, RenderResourceUsage::ReadWrite, RenderResourceState::StorageReadWrite},
        {RenderResourceNames::camera_color_history, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}},
       {"Raster"}},
      []() {
      });

  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(plan.passes.size(), 2);
  ASSERT_EQ(plan.resources.size(), 3);
  EXPECT_TRUE(plan.uses_graphics_queue);
  EXPECT_TRUE(plan.uses_compute_queue);
  ASSERT_EQ(plan.passes[1].dependency_indices.size(), 1);
  EXPECT_EQ(plan.passes[1].dependency_indices[0], 0);

  const auto& color_plan = plan.resources[0];
  const auto& temporary_plan = plan.resources[1];
  const auto& history_plan = plan.resources[2];
  EXPECT_TRUE(color_plan.imported);
  EXPECT_FALSE(color_plan.can_alias);
  EXPECT_FALSE(temporary_plan.imported);
  EXPECT_TRUE(temporary_plan.can_alias);
  EXPECT_EQ(temporary_plan.first_pass_index, 0);
  EXPECT_EQ(temporary_plan.last_pass_index, 1);
  ASSERT_EQ(temporary_plan.reader_pass_indices.size(), 1);
  ASSERT_EQ(temporary_plan.writer_pass_indices.size(), 2);
  EXPECT_EQ(temporary_plan.reader_pass_indices[0], 1);
  EXPECT_EQ(temporary_plan.writer_pass_indices[0], 0);
  EXPECT_EQ(temporary_plan.writer_pass_indices[1], 1);
  EXPECT_FALSE(history_plan.imported);
  EXPECT_FALSE(history_plan.can_alias);
}

TEST(RenderGraph, ValidateRejectsManagedImagesWithoutSizeMode) {
  RenderGraph graph;
  graph.AddResource(
      {"Camera.InvalidTransient", RenderResourceType::Image, RenderResourceLifetime::Camera, {}, "RGBA8", 1, 1, true});
  graph.AddPass({"WriteInvalid",
                 RenderPassQueue::Graphics,
                 RenderPassScope::Camera,
                 {{"Camera.InvalidTransient", RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
                []() {
                });

  EXPECT_FALSE(graph.Validate());
  EXPECT_FALSE(graph.Compile().valid);
}

TEST(RenderGraph, CompileAliasesCompatibleNonOverlappingTransients) {
  constexpr const char* bloom_a = "Camera.BloomA";
  constexpr const char* bloom_b = "Camera.BloomB";
  RenderGraph graph;
  graph.AddResource({bloom_a,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "RGBA16F",
                     1,
                     1,
                     true});
  graph.AddResource({bloom_b,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "RGBA16F",
                     1,
                     1,
                     true});
  graph.AddPass({"BloomDownsample",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{bloom_a, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}}},
                []() {
                });
  graph.AddPass({"BloomUpsample",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{bloom_b, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}},
                 {"BloomDownsample"}},
                []() {
                });

  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(plan.allocations.size(), 1);
  ASSERT_EQ(plan.allocations[0].resource_indices.size(), 2);
  EXPECT_EQ(plan.resources[0].allocation_slot_index, plan.resources[1].allocation_slot_index);
}

TEST(RenderGraph, CompileSeparatesOverlappingTransients) {
  constexpr const char* ping = "Camera.Ping";
  constexpr const char* pong = "Camera.Pong";
  RenderGraph graph;
  graph.AddResource({ping,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "RGBA16F",
                     1,
                     1,
                     true});
  graph.AddResource({pong,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "RGBA16F",
                     1,
                     1,
                     true});
  graph.AddPass({"WritePing",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{ping, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}}},
                []() {
                });
  graph.AddPass({"WritePongReadPing",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{ping, RenderResourceUsage::Read, RenderResourceState::StorageReadWrite},
                  {pong, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}},
                 {"WritePing"}},
                []() {
                });

  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(plan.allocations.size(), 2);
  EXPECT_NE(plan.resources[0].allocation_slot_index, plan.resources[1].allocation_slot_index);
}

TEST(RenderGraph, CompileRecordsStateAndQueueTransitions) {
  RenderGraph graph;
  graph.AddResource({RenderResourceNames::camera_color, RenderResourceType::Image, RenderResourceLifetime::Camera});
  graph.AddPass(
      {"Raster",
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
      []() {
      });
  graph.AddPass(
      {"Denoise",
       RenderPassQueue::Compute,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::StorageReadWrite}},
       {"Raster"}},
      []() {
      });
  graph.AddPass({"Sample",
                 RenderPassQueue::Graphics,
                 RenderPassScope::Camera,
                 {{RenderResourceNames::camera_color, RenderResourceUsage::Read, RenderResourceState::ShaderRead}},
                 {"Denoise"}},
                []() {
                });

  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(plan.transitions.size(), 3);
  EXPECT_TRUE(plan.transitions[0].initial_use);
  EXPECT_EQ(plan.transitions[0].previous_state, RenderResourceState::Undefined);
  EXPECT_EQ(plan.transitions[0].next_state, RenderResourceState::ColorAttachment);
  EXPECT_FALSE(plan.transitions[0].memory_dependency);
  EXPECT_EQ(plan.transitions[1].pass_index, 1);
  EXPECT_EQ(plan.transitions[1].previous_pass_index, 0);
  EXPECT_TRUE(plan.transitions[1].queue_change);
  EXPECT_TRUE(plan.transitions[1].memory_dependency);
  EXPECT_EQ(plan.transitions[1].previous_state, RenderResourceState::ColorAttachment);
  EXPECT_EQ(plan.transitions[1].next_state, RenderResourceState::StorageReadWrite);
  EXPECT_EQ(plan.transitions[2].pass_index, 2);
  EXPECT_EQ(plan.transitions[2].previous_pass_index, 1);
  EXPECT_TRUE(plan.transitions[2].queue_change);
  EXPECT_EQ(plan.transitions[2].next_state, RenderResourceState::ShaderRead);
}

TEST(RenderGraph, CompileClassifiesBarrierPlansForImageAndBufferMemoryDependencies) {
  constexpr const char* history_target = "Camera.HistoryTarget";
  constexpr const char* visibility_buffer = "Frame.VisibilityBuffer";
  RenderGraph graph;
  graph.AddResource({history_target, RenderResourceType::Image, RenderResourceLifetime::Camera});
  graph.AddResource({visibility_buffer, RenderResourceType::Buffer, RenderResourceLifetime::Frame});
  bool saw_release_barriers = false;
  bool saw_acquire_barriers = false;
  graph.AddPass({"Produce",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{history_target, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite},
                  {visibility_buffer, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}}},
                [&](const RenderGraphExecutionContext& context) {
                  const auto release_barriers = context.GetCurrentPassReleaseBarriers();
                  ASSERT_EQ(release_barriers.size(), 2);
                  EXPECT_TRUE(std::all_of(
                      release_barriers.begin(), release_barriers.end(), [](const RenderResourceBarrierPlan* barrier) {
                        return barrier && barrier->queue_change && barrier->previous_pass_index == 0;
                      }));
                  saw_release_barriers = true;
                });
  graph.AddPass(
      {"Consume",
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{history_target, RenderResourceUsage::Read, RenderResourceState::StorageReadWrite},
        {visibility_buffer, RenderResourceUsage::Read, RenderResourceState::ShaderRead}},
       {"Produce"}},
      [&](const RenderGraphExecutionContext& context) {
        const auto barriers = context.GetCurrentPassBarriers();
        ASSERT_EQ(barriers.size(), 2);
        EXPECT_NE(std::find_if(barriers.begin(), barriers.end(),
                               [&](const auto* barrier) {
                                 const auto* descriptor = context.GetResourceDescriptor(barrier->resource_index);
                                 return descriptor && descriptor->name == history_target &&
                                        barrier->barrier_type == RenderGraphBarrierType::ImageMemory;
                               }),
                  barriers.end());
        EXPECT_NE(std::find_if(barriers.begin(), barriers.end(),
                               [&](const auto* barrier) {
                                 const auto* descriptor = context.GetResourceDescriptor(barrier->resource_index);
                                 return descriptor && descriptor->name == visibility_buffer &&
                                        barrier->barrier_type == RenderGraphBarrierType::BufferMemory &&
                                        barrier->queue_change;
                               }),
                  barriers.end());
        saw_acquire_barriers = true;
      });

  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(plan.transitions.size(), 4);
  ASSERT_EQ(plan.barriers.size(), 3);
  EXPECT_EQ(plan.barriers[0].barrier_type, RenderGraphBarrierType::ImageLayout);
  EXPECT_EQ(plan.barriers[1].barrier_type, RenderGraphBarrierType::ImageMemory);
  EXPECT_EQ(plan.barriers[2].barrier_type, RenderGraphBarrierType::BufferMemory);
  const RenderGraphResourceRegistry registry;
  graph.Execute(plan, registry);
  EXPECT_TRUE(saw_release_barriers);
  EXPECT_TRUE(saw_acquire_barriers);
}

TEST(RenderGraph, RenderLayerAppliesQueueFamilyOwnershipTransfersForQueueChangingBarriers) {
  const auto render_layer_path =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp";
  std::ifstream render_layer_file(render_layer_path);
  ASSERT_TRUE(render_layer_file.good()) << render_layer_path.string();
  const std::string render_layer_source((std::istreambuf_iterator<char>(render_layer_file)),
                                        std::istreambuf_iterator<char>());

  EXPECT_NE(render_layer_source.find("uint32_t GetRenderPassQueueFamilyIndex(const RenderPassQueue queue)"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("TryGetQueueFamilyOwnershipTransfer"), std::string::npos);
  EXPECT_NE(render_layer_source.find("const auto expected_queue = release_barrier ? barrier.previous_queue : "
                                     "barrier.next_queue"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("src_queue_family_index = GetRenderPassQueueFamilyIndex(barrier.previous_queue)"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("dst_queue_family_index = GetRenderPassQueueFamilyIndex(barrier.next_queue)"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("return src_queue_family_index != dst_queue_family_index"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ApplyGraphImageQueueOwnershipBarrier"), std::string::npos);
  EXPECT_NE(render_layer_source.find("image->TransitImageLayout(vk_command_buffer, previous_layout, next_layout, "
                                     "src_queue_family_index"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("ApplyGraphBufferQueueOwnershipBarrier"), std::string::npos);
  EXPECT_NE(render_layer_source.find("Platform::BufferMemoryBarrier(vk_command_buffer, *binding->buffer, "
                                     "src_queue_family_index, dst_queue_family_index"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("ApplyGraphResourceReleaseBarriers"), std::string::npos);
  EXPECT_NE(render_layer_source.find("context.GetCurrentPassReleaseBarriers()"), std::string::npos);
}

TEST(RenderGraph, CompilePlansDdgiAtlasPrepareResources) {
  RenderLayer::DdgiSettings settings;
  settings.volume_defaults.probe_counts = {4, 4, 4};
  settings.storage.atlas_probe_columns = 8;
  settings.storage.irradiance_tile_resolution = 8;
  settings.storage.visibility_tile_resolution = 16;
  settings.runtime.ray_count = 32;
  const auto layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings);

  RenderGraph graph;
  graph.AddResource({RenderResourceNames::frame_ddgi_probe_metadata,
                     RenderResourceType::Buffer,
                     RenderResourceLifetime::Persistent,
                     {},
                     {},
                     1,
                     1,
                     false,
                     layout.probe_metadata_byte_size});
  graph.AddResource({RenderResourceNames::frame_ddgi_probe_update_indices,
                     RenderResourceType::Buffer,
                     RenderResourceLifetime::Persistent,
                     {},
                     {},
                     1,
                     1,
                     false,
                     layout.probe_update_index_byte_size});
  graph.AddResource({RenderResourceNames::frame_ddgi_ray_output,
                     RenderResourceType::Buffer,
                     RenderResourceLifetime::Frame,
                     {},
                     {},
                     1,
                     1,
                     true,
                     layout.ray_output_byte_size});
  graph.AddResource({RenderResourceNames::frame_ddgi_irradiance_atlas,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Persistent,
                     {RenderResourceSizeMode::Absolute, layout.irradiance_atlas.resolution.x,
                      layout.irradiance_atlas.resolution.y, 1, 1, 1},
                     "RGBA16F",
                     1,
                     1,
                     false});
  graph.AddResource({RenderResourceNames::frame_ddgi_visibility_atlas,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Persistent,
                     {RenderResourceSizeMode::Absolute, layout.visibility_atlas.resolution.x,
                      layout.visibility_atlas.resolution.y, 1, 1, 1},
                     "RG16F",
                     1,
                     1,
                     false});
  graph.AddResource({RenderResourceNames::frame_ddgi_variability_atlas,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Persistent,
                     {RenderResourceSizeMode::Absolute, layout.variability_atlas.resolution.x,
                      layout.variability_atlas.resolution.y, 1, 1, 1},
                     "R16F",
                     1,
                     1,
                     false});
  graph.AddResource({RenderResourceNames::frame_ddgi_variability_reduction_a,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Frame,
                     {RenderResourceSizeMode::Absolute, layout.variability_reduction_extent.x,
                      layout.variability_reduction_extent.y, 1, 1, 1},
                     "RG32F",
                     1,
                     1,
                     true});
  graph.AddResource({RenderResourceNames::frame_ddgi_variability_reduction_b,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Frame,
                     {RenderResourceSizeMode::Absolute, layout.variability_reduction_extent.x,
                      layout.variability_reduction_extent.y, 1, 1, 1},
                     "RG32F",
                     1,
                     1,
                     true});
  graph.AddPass({RenderPassNames::ddgi_atlas_prepare,
                 RenderPassQueue::Compute,
                 RenderPassScope::Frame,
                 {{RenderResourceNames::frame_ddgi_probe_metadata, RenderResourceUsage::Write,
                   RenderResourceState::TransferDestination},
                  {RenderResourceNames::frame_ddgi_ray_output, RenderResourceUsage::Write,
                   RenderResourceState::TransferDestination},
                  {RenderResourceNames::frame_ddgi_irradiance_atlas, RenderResourceUsage::Write,
                   RenderResourceState::TransferDestination},
                  {RenderResourceNames::frame_ddgi_visibility_atlas, RenderResourceUsage::Write,
                   RenderResourceState::TransferDestination},
                  {RenderResourceNames::frame_ddgi_variability_atlas, RenderResourceUsage::Write,
                   RenderResourceState::TransferDestination}}},
                []() {
                });

  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.uses_compute_queue);
  EXPECT_TRUE(plan.resources[0].imported);
  EXPECT_TRUE(plan.resources[1].imported);
  EXPECT_FALSE(plan.resources[2].imported);
  EXPECT_TRUE(plan.resources[3].imported);
  EXPECT_TRUE(plan.resources[4].imported);
  ASSERT_EQ(plan.allocations.size(), 1);
  EXPECT_EQ(plan.allocations[0].resource_indices, std::vector<size_t>({2}));
  ASSERT_EQ(plan.barriers.size(), 3);
  EXPECT_NE(std::find_if(plan.barriers.begin(), plan.barriers.end(),
                         [&](const RenderResourceBarrierPlan& barrier) {
                           return graph.GetResources()[barrier.resource_index].name ==
                                      RenderResourceNames::frame_ddgi_irradiance_atlas &&
                                  barrier.barrier_type == RenderGraphBarrierType::ImageLayout &&
                                  barrier.next_state == RenderResourceState::TransferDestination;
                         }),
            plan.barriers.end());
  EXPECT_NE(std::find_if(plan.barriers.begin(), plan.barriers.end(),
                         [&](const RenderResourceBarrierPlan& barrier) {
                           return graph.GetResources()[barrier.resource_index].name ==
                                      RenderResourceNames::frame_ddgi_visibility_atlas &&
                                  barrier.barrier_type == RenderGraphBarrierType::ImageLayout &&
                                  barrier.next_state == RenderResourceState::TransferDestination;
                         }),
            plan.barriers.end());
  EXPECT_NE(std::find_if(plan.barriers.begin(), plan.barriers.end(),
                         [&](const RenderResourceBarrierPlan& barrier) {
                           return graph.GetResources()[barrier.resource_index].name ==
                                      RenderResourceNames::frame_ddgi_variability_atlas &&
                                  barrier.barrier_type == RenderGraphBarrierType::ImageLayout &&
                                  barrier.next_state == RenderResourceState::TransferDestination;
                         }),
            plan.barriers.end());
}

TEST(RenderGraph, CompilePlansDdgiRayDiagnosticsAfterAtlasPrepare) {
  RenderLayer::DdgiSettings settings;
  settings.volume_defaults.probe_counts = {2, 2, 2};
  settings.runtime.ray_count = 16;
  const auto layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings);

  RenderGraph graph;
  graph.AddResource({RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceType::DescriptorSet,
                     RenderResourceLifetime::Frame});
  graph.AddResource({RenderResourceNames::frame_ray_tracing_descriptor_set, RenderResourceType::DescriptorSet,
                     RenderResourceLifetime::Frame});
  graph.AddResource(
      {RenderResourceNames::scene_mesh_tlas, RenderResourceType::AccelerationStructure, RenderResourceLifetime::Frame});
  graph.AddResource({RenderResourceNames::frame_ddgi_ray_output,
                     RenderResourceType::Buffer,
                     RenderResourceLifetime::Frame,
                     {},
                     {},
                     1,
                     1,
                     true,
                     layout.ray_output_byte_size});
  graph.AddResource({RenderResourceNames::frame_ddgi_probe_state,
                     RenderResourceType::Buffer,
                     RenderResourceLifetime::Persistent,
                     {},
                     {},
                     1,
                     1,
                     false,
                     layout.probe_state_byte_size});
  graph.AddResource({RenderResourceNames::frame_ddgi_irradiance_atlas,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Persistent,
                     {RenderResourceSizeMode::Absolute, layout.irradiance_atlas.resolution.x,
                      layout.irradiance_atlas.resolution.y, 1, 1, 1},
                     "RGBA16F",
                     1,
                     1,
                     false});
  graph.AddResource({RenderResourceNames::frame_ddgi_visibility_atlas,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Persistent,
                     {RenderResourceSizeMode::Absolute, layout.visibility_atlas.resolution.x,
                      layout.visibility_atlas.resolution.y, 1, 1, 1},
                     "RG16F",
                     1,
                     1,
                     false});
  graph.AddResource({RenderResourceNames::frame_ddgi_variability_atlas,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Persistent,
                     {RenderResourceSizeMode::Absolute, layout.variability_atlas.resolution.x,
                      layout.variability_atlas.resolution.y, 1, 1, 1},
                     "R16F",
                     1,
                     1,
                     false});
  graph.AddResource({RenderResourceNames::frame_ddgi_variability_reduction_a,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Frame,
                     {RenderResourceSizeMode::Absolute, layout.variability_reduction_extent.x,
                      layout.variability_reduction_extent.y, 1, 1, 1},
                     "RG32F",
                     1,
                     1,
                     true});
  graph.AddResource({RenderResourceNames::frame_ddgi_variability_reduction_b,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Frame,
                     {RenderResourceSizeMode::Absolute, layout.variability_reduction_extent.x,
                      layout.variability_reduction_extent.y, 1, 1, 1},
                     "RG32F",
                     1,
                     1,
                     true});
  graph.AddPass({RenderPassNames::ddgi_atlas_prepare,
                 RenderPassQueue::Compute,
                 RenderPassScope::Frame,
                 {{RenderResourceNames::frame_ddgi_ray_output, RenderResourceUsage::Write,
                   RenderResourceState::TransferDestination},
                  {RenderResourceNames::frame_ddgi_irradiance_atlas, RenderResourceUsage::Write,
                   RenderResourceState::TransferDestination},
                  {RenderResourceNames::frame_ddgi_visibility_atlas, RenderResourceUsage::Write,
                   RenderResourceState::TransferDestination},
                  {RenderResourceNames::frame_ddgi_variability_atlas, RenderResourceUsage::Write,
                   RenderResourceState::TransferDestination}}},
                []() {
                });
  graph.AddPass(
      {RenderPassNames::ddgi_ray_diagnostics,
       RenderPassQueue::RayTracing,
       RenderPassScope::Frame,
       {{RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
        {RenderResourceNames::frame_ray_tracing_descriptor_set, RenderResourceUsage::Read,
         RenderResourceState::General},
        {RenderResourceNames::scene_mesh_tlas, RenderResourceUsage::Read,
         RenderResourceState::AccelerationStructureRead},
        {RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
        {RenderResourceNames::frame_ddgi_irradiance_atlas, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
        {RenderResourceNames::frame_ddgi_visibility_atlas, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
        {RenderResourceNames::frame_ddgi_ray_output, RenderResourceUsage::ReadWrite,
         RenderResourceState::StorageReadWrite}}},
      []() {
      });

  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.uses_compute_queue);
  EXPECT_TRUE(plan.uses_ray_tracing_queue);
  ASSERT_EQ(plan.passes.size(), 2);
  EXPECT_EQ(plan.passes[1].resource_dependency_indices, std::vector<size_t>({0}));
  ASSERT_EQ(plan.allocations.size(), 1);
  EXPECT_EQ(plan.allocations[0].byte_size, layout.ray_output_byte_size);
  EXPECT_NE(std::find_if(plan.barriers.begin(), plan.barriers.end(),
                         [&](const RenderResourceBarrierPlan& barrier) {
                           return graph.GetResources()[barrier.resource_index].name ==
                                      RenderResourceNames::frame_ddgi_ray_output &&
                                  barrier.barrier_type == RenderGraphBarrierType::BufferMemory &&
                                  barrier.next_state == RenderResourceState::StorageReadWrite;
                         }),
            plan.barriers.end());
  EXPECT_NE(std::find_if(plan.barriers.begin(), plan.barriers.end(),
                         [&](const RenderResourceBarrierPlan& barrier) {
                           return graph.GetResources()[barrier.resource_index].name ==
                                      RenderResourceNames::frame_ddgi_irradiance_atlas &&
                                  barrier.barrier_type == RenderGraphBarrierType::ImageLayout &&
                                  barrier.next_state == RenderResourceState::ShaderRead;
                         }),
            plan.barriers.end());
  EXPECT_NE(std::find_if(plan.barriers.begin(), plan.barriers.end(),
                         [&](const RenderResourceBarrierPlan& barrier) {
                           return graph.GetResources()[barrier.resource_index].name ==
                                      RenderResourceNames::frame_ddgi_visibility_atlas &&
                                  barrier.barrier_type == RenderGraphBarrierType::ImageLayout &&
                                  barrier.next_state == RenderResourceState::ShaderRead;
                         }),
            plan.barriers.end());
}

TEST(RenderGraph, CompilePlansDdgiProbeUpdateAfterRayDiagnostics) {
  RenderLayer::DdgiSettings settings;
  settings.volume_defaults.probe_counts = {2, 2, 2};
  settings.runtime.ray_count = 16;
  const auto layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings);

  RenderGraph graph;
  graph.AddResource({RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceType::DescriptorSet,
                     RenderResourceLifetime::Frame});
  graph.AddResource({RenderResourceNames::frame_ray_tracing_descriptor_set, RenderResourceType::DescriptorSet,
                     RenderResourceLifetime::Frame});
  graph.AddResource(
      {RenderResourceNames::scene_mesh_tlas, RenderResourceType::AccelerationStructure, RenderResourceLifetime::Frame});
  graph.AddResource({RenderResourceNames::frame_ddgi_ray_output,
                     RenderResourceType::Buffer,
                     RenderResourceLifetime::Frame,
                     {},
                     {},
                     1,
                     1,
                     true,
                     layout.ray_output_byte_size});
  graph.AddResource({RenderResourceNames::frame_ddgi_probe_state,
                     RenderResourceType::Buffer,
                     RenderResourceLifetime::Persistent,
                     {},
                     {},
                     1,
                     1,
                     false,
                     layout.probe_state_byte_size});
  graph.AddResource({RenderResourceNames::frame_ddgi_probe_metadata,
                     RenderResourceType::Buffer,
                     RenderResourceLifetime::Persistent,
                     {},
                     {},
                     1,
                     1,
                     false,
                     layout.probe_metadata_byte_size});
  graph.AddResource({RenderResourceNames::frame_ddgi_probe_update_indices,
                     RenderResourceType::Buffer,
                     RenderResourceLifetime::Persistent,
                     {},
                     {},
                     1,
                     1,
                     false,
                     layout.probe_update_index_byte_size});
  graph.AddResource({RenderResourceNames::frame_ddgi_irradiance_atlas,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Persistent,
                     {RenderResourceSizeMode::Absolute, layout.irradiance_atlas.resolution.x,
                      layout.irradiance_atlas.resolution.y, 1, 1, 1},
                     "RGBA16F",
                     1,
                     1,
                     false});
  graph.AddResource({RenderResourceNames::frame_ddgi_visibility_atlas,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Persistent,
                     {RenderResourceSizeMode::Absolute, layout.visibility_atlas.resolution.x,
                      layout.visibility_atlas.resolution.y, 1, 1, 1},
                     "RG16F",
                     1,
                     1,
                     false});
  graph.AddResource({RenderResourceNames::frame_ddgi_variability_atlas,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Persistent,
                     {RenderResourceSizeMode::Absolute, layout.variability_atlas.resolution.x,
                      layout.variability_atlas.resolution.y, 1, 1, 1},
                     "R16F",
                     1,
                     1,
                     false});
  graph.AddResource({RenderResourceNames::frame_ddgi_variability_reduction_a,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Frame,
                     {RenderResourceSizeMode::Absolute, layout.variability_reduction_extent.x,
                      layout.variability_reduction_extent.y, 1, 1, 1},
                     "RG32F",
                     1,
                     1,
                     true});
  graph.AddResource({RenderResourceNames::frame_ddgi_variability_reduction_b,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Frame,
                     {RenderResourceSizeMode::Absolute, layout.variability_reduction_extent.x,
                      layout.variability_reduction_extent.y, 1, 1, 1},
                     "RG32F",
                     1,
                     1,
                     true});
  graph.AddPass({RenderPassNames::ddgi_atlas_prepare,
                 RenderPassQueue::Compute,
                 RenderPassScope::Frame,
                 {{RenderResourceNames::frame_ddgi_ray_output, RenderResourceUsage::Write,
                   RenderResourceState::TransferDestination},
                  {RenderResourceNames::frame_ddgi_irradiance_atlas, RenderResourceUsage::Write,
                   RenderResourceState::TransferDestination},
                  {RenderResourceNames::frame_ddgi_visibility_atlas, RenderResourceUsage::Write,
                   RenderResourceState::TransferDestination},
                  {RenderResourceNames::frame_ddgi_variability_atlas, RenderResourceUsage::Write,
                   RenderResourceState::TransferDestination}}},
                []() {
                });
  graph.AddPass(
      {RenderPassNames::ddgi_ray_diagnostics,
       RenderPassQueue::RayTracing,
       RenderPassScope::Frame,
       {{RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
        {RenderResourceNames::frame_ray_tracing_descriptor_set, RenderResourceUsage::Read,
         RenderResourceState::General},
        {RenderResourceNames::scene_mesh_tlas, RenderResourceUsage::Read,
         RenderResourceState::AccelerationStructureRead},
        {RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
        {RenderResourceNames::frame_ddgi_ray_output, RenderResourceUsage::ReadWrite,
         RenderResourceState::StorageReadWrite}}},
      []() {
      });
  RenderPassDescriptor update_descriptor{
      RenderPassNames::ddgi_probe_update,
      RenderPassQueue::Compute,
      RenderPassScope::Frame,
      {{RenderResourceNames::frame_ddgi_ray_output, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_probe_update_indices, RenderResourceUsage::Read,
        RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_irradiance_atlas, RenderResourceUsage::ReadWrite,
        RenderResourceState::StorageReadWrite},
       {RenderResourceNames::frame_ddgi_visibility_atlas, RenderResourceUsage::ReadWrite,
        RenderResourceState::StorageReadWrite},
       {RenderResourceNames::frame_ddgi_variability_atlas, RenderResourceUsage::Write,
        RenderResourceState::StorageReadWrite}}};
  update_descriptor.resources.push_back({RenderResourceNames::frame_ddgi_probe_metadata, RenderResourceUsage::Write,
                                         RenderResourceState::StorageReadWrite});
  update_descriptor.resources.push_back(
      {RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::Read, RenderResourceState::ShaderRead});
  update_descriptor.dependencies = {RenderPassNames::ddgi_ray_diagnostics};
  graph.AddPass(update_descriptor, []() {
  });
  RenderPassDescriptor relocation_descriptor{
      RenderPassNames::ddgi_probe_relocation,
      RenderPassQueue::Compute,
      RenderPassScope::Frame,
      {{RenderResourceNames::frame_ddgi_ray_output, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_probe_update_indices, RenderResourceUsage::Read,
        RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::ReadWrite,
        RenderResourceState::StorageReadWrite}}};
  relocation_descriptor.dependencies = {RenderPassNames::ddgi_probe_update};
  graph.AddPass(relocation_descriptor, []() {
  });
  RenderPassDescriptor classification_descriptor{
      RenderPassNames::ddgi_probe_classification,
      RenderPassQueue::Compute,
      RenderPassScope::Frame,
      {{RenderResourceNames::frame_ddgi_ray_output, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_probe_update_indices, RenderResourceUsage::Read,
        RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::ReadWrite,
        RenderResourceState::StorageReadWrite}}};
  classification_descriptor.dependencies = {RenderPassNames::ddgi_probe_update};
  graph.AddPass(classification_descriptor, []() {
  });
  RenderPassDescriptor variability_descriptor{
      RenderPassNames::ddgi_probe_variability,
      RenderPassQueue::Compute,
      RenderPassScope::Frame,
      {{RenderResourceNames::frame_ddgi_variability_atlas, RenderResourceUsage::Read,
        RenderResourceState::StorageReadWrite},
       {RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_variability_reduction_a, RenderResourceUsage::ReadWrite,
        RenderResourceState::StorageReadWrite},
       {RenderResourceNames::frame_ddgi_variability_reduction_b, RenderResourceUsage::ReadWrite,
        RenderResourceState::StorageReadWrite}}};
  variability_descriptor.dependencies = {RenderPassNames::ddgi_probe_update};
  graph.AddPass(variability_descriptor, []() {
  });

  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.uses_compute_queue);
  EXPECT_TRUE(plan.uses_ray_tracing_queue);
  ASSERT_EQ(plan.passes.size(), 6);
  EXPECT_NE(std::find(plan.passes[2].dependency_indices.begin(), plan.passes[2].dependency_indices.end(), 1),
            plan.passes[2].dependency_indices.end());
  EXPECT_NE(std::find(plan.passes[2].resource_dependency_indices.begin(),
                      plan.passes[2].resource_dependency_indices.end(), 1),
            plan.passes[2].resource_dependency_indices.end());
  EXPECT_NE(std::find(plan.passes[2].resource_dependency_indices.begin(),
                      plan.passes[2].resource_dependency_indices.end(), 0),
            plan.passes[2].resource_dependency_indices.end());
  EXPECT_NE(std::find(plan.passes[3].dependency_indices.begin(), plan.passes[3].dependency_indices.end(), 2),
            plan.passes[3].dependency_indices.end());
  EXPECT_NE(std::find(plan.passes[3].resource_dependency_indices.begin(),
                      plan.passes[3].resource_dependency_indices.end(), 2),
            plan.passes[3].resource_dependency_indices.end());
  EXPECT_NE(std::find(plan.passes[4].dependency_indices.begin(), plan.passes[4].dependency_indices.end(), 2),
            plan.passes[4].dependency_indices.end());
  EXPECT_NE(std::find(plan.passes[4].resource_dependency_indices.begin(),
                      plan.passes[4].resource_dependency_indices.end(), 3),
            plan.passes[4].resource_dependency_indices.end());
  EXPECT_NE(std::find(plan.passes[5].dependency_indices.begin(), plan.passes[5].dependency_indices.end(), 2),
            plan.passes[5].dependency_indices.end());
  EXPECT_NE(std::find(plan.passes[5].resource_dependency_indices.begin(),
                      plan.passes[5].resource_dependency_indices.end(), 4),
            plan.passes[5].resource_dependency_indices.end());
  EXPECT_NE(std::find_if(plan.barriers.begin(), plan.barriers.end(),
                         [&](const RenderResourceBarrierPlan& barrier) {
                           return graph.GetResources()[barrier.resource_index].name ==
                                      RenderResourceNames::frame_ddgi_ray_output &&
                                  barrier.barrier_type == RenderGraphBarrierType::BufferMemory &&
                                  barrier.next_state == RenderResourceState::ShaderRead;
                         }),
            plan.barriers.end());
  EXPECT_NE(std::find_if(plan.barriers.begin(), plan.barriers.end(),
                         [&](const RenderResourceBarrierPlan& barrier) {
                           return graph.GetResources()[barrier.resource_index].name ==
                                      RenderResourceNames::frame_ddgi_irradiance_atlas &&
                                  barrier.barrier_type == RenderGraphBarrierType::ImageLayout &&
                                  barrier.next_state == RenderResourceState::StorageReadWrite;
                         }),
            plan.barriers.end());
}

TEST(RenderGraph, DdgiAtlasPrepareResourcesRemainValidAfterResizeAndReset) {
  auto validate_layout = [](const RenderLayer::DdgiFrameResourceLayout& layout) {
    RenderGraph graph;
    auto add_buffer = [&](const char* name, const RenderResourceLifetime lifetime, const bool managed_by_graph,
                          const uint64_t byte_size) {
      graph.AddResource({name, RenderResourceType::Buffer, lifetime, {}, {}, 1, 1, managed_by_graph, byte_size});
    };
    auto add_atlas = [&](const char* name, const RenderLayer::DdgiAtlasLayout& atlas_layout, const char* format_name) {
      graph.AddResource(
          {name,
           RenderResourceType::Image,
           RenderResourceLifetime::Persistent,
           {RenderResourceSizeMode::Absolute, atlas_layout.resolution.x, atlas_layout.resolution.y, 1, 1, 1},
           format_name,
           1,
           1,
           false});
    };

    add_buffer(RenderResourceNames::frame_ddgi_probe_metadata, RenderResourceLifetime::Persistent, false,
               layout.probe_metadata_byte_size);
    add_buffer(RenderResourceNames::frame_ddgi_probe_state, RenderResourceLifetime::Persistent, false,
               layout.probe_state_byte_size);
    add_buffer(RenderResourceNames::frame_ddgi_ray_output, RenderResourceLifetime::Frame, true,
               layout.ray_output_byte_size);
    add_atlas(RenderResourceNames::frame_ddgi_irradiance_atlas, layout.irradiance_atlas, "RGBA16F");
    add_atlas(RenderResourceNames::frame_ddgi_visibility_atlas, layout.visibility_atlas, "RG16F");
    add_atlas(RenderResourceNames::frame_ddgi_variability_atlas, layout.variability_atlas, "R16F");
    graph.AddPass({RenderPassNames::ddgi_atlas_prepare,
                   RenderPassQueue::Compute,
                   RenderPassScope::Frame,
                   {{RenderResourceNames::frame_ddgi_probe_metadata, RenderResourceUsage::Write,
                     RenderResourceState::TransferDestination},
                    {RenderResourceNames::frame_ddgi_ray_output, RenderResourceUsage::Write,
                     RenderResourceState::TransferDestination},
                    {RenderResourceNames::frame_ddgi_irradiance_atlas, RenderResourceUsage::Write,
                     RenderResourceState::TransferDestination},
                    {RenderResourceNames::frame_ddgi_visibility_atlas, RenderResourceUsage::Write,
                     RenderResourceState::TransferDestination},
                    {RenderResourceNames::frame_ddgi_variability_atlas, RenderResourceUsage::Write,
                     RenderResourceState::TransferDestination}}},
                  []() {
                  });
    graph.AddPass(
        {"DDGIAtlasPreviewRead",
         RenderPassQueue::Compute,
         RenderPassScope::Frame,
         {{RenderResourceNames::frame_ddgi_probe_metadata, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
          {RenderResourceNames::frame_ddgi_irradiance_atlas, RenderResourceUsage::Read,
           RenderResourceState::ShaderRead},
          {RenderResourceNames::frame_ddgi_visibility_atlas, RenderResourceUsage::Read,
           RenderResourceState::ShaderRead},
          {RenderResourceNames::frame_ddgi_variability_atlas, RenderResourceUsage::Read,
           RenderResourceState::ShaderRead}},
         {RenderPassNames::ddgi_atlas_prepare}},
        []() {
        });

    const auto plan = graph.Compile();
    ASSERT_TRUE(plan.valid);
    auto find_resource_index = [&](const char* name) {
      const auto& resources = graph.GetResources();
      const auto resource = std::find_if(resources.begin(), resources.end(), [&](const auto& descriptor) {
        return descriptor.name == name;
      });
      return resource == resources.end() ? RenderGraphConstants::invalid_resource_index
                                         : static_cast<size_t>(resource - resources.begin());
    };
    const auto metadata_index = find_resource_index(RenderResourceNames::frame_ddgi_probe_metadata);
    const auto state_index = find_resource_index(RenderResourceNames::frame_ddgi_probe_state);
    const auto ray_output_index = find_resource_index(RenderResourceNames::frame_ddgi_ray_output);
    const auto irradiance_index = find_resource_index(RenderResourceNames::frame_ddgi_irradiance_atlas);
    const auto visibility_index = find_resource_index(RenderResourceNames::frame_ddgi_visibility_atlas);
    const auto variability_index = find_resource_index(RenderResourceNames::frame_ddgi_variability_atlas);
    ASSERT_NE(metadata_index, RenderGraphConstants::invalid_resource_index);
    ASSERT_NE(state_index, RenderGraphConstants::invalid_resource_index);
    ASSERT_NE(ray_output_index, RenderGraphConstants::invalid_resource_index);
    ASSERT_NE(irradiance_index, RenderGraphConstants::invalid_resource_index);
    ASSERT_NE(visibility_index, RenderGraphConstants::invalid_resource_index);
    ASSERT_NE(variability_index, RenderGraphConstants::invalid_resource_index);

    EXPECT_TRUE(plan.resources[metadata_index].imported);
    EXPECT_TRUE(plan.resources[state_index].imported);
    EXPECT_FALSE(plan.resources[ray_output_index].imported);
    EXPECT_TRUE(plan.resources[irradiance_index].imported);
    EXPECT_TRUE(plan.resources[visibility_index].imported);
    EXPECT_TRUE(plan.resources[variability_index].imported);
    EXPECT_EQ(graph.GetResources()[metadata_index].byte_size, layout.probe_metadata_byte_size);
    EXPECT_EQ(graph.GetResources()[state_index].byte_size, layout.probe_state_byte_size);
    ASSERT_EQ(plan.allocations.size(), 1);
    ASSERT_EQ(plan.allocations[0].resource_indices.size(), 1);
    EXPECT_EQ(plan.allocations[0].resource_indices[0], ray_output_index);
    EXPECT_EQ(plan.allocations[0].byte_size, layout.ray_output_byte_size);
    EXPECT_EQ(plan.resources[irradiance_index].resolved_dimensions.width, layout.irradiance_atlas.resolution.x);
    EXPECT_EQ(plan.resources[irradiance_index].resolved_dimensions.height, layout.irradiance_atlas.resolution.y);
    EXPECT_EQ(plan.resources[visibility_index].resolved_dimensions.width, layout.visibility_atlas.resolution.x);
    EXPECT_EQ(plan.resources[visibility_index].resolved_dimensions.height, layout.visibility_atlas.resolution.y);
    EXPECT_EQ(plan.resources[variability_index].resolved_dimensions.width, layout.variability_atlas.resolution.x);
    EXPECT_EQ(plan.resources[variability_index].resolved_dimensions.height, layout.variability_atlas.resolution.y);

    auto has_barrier = [&](const size_t resource_index, const RenderGraphBarrierType type,
                           const RenderResourceState next_state) {
      return std::find_if(plan.barriers.begin(), plan.barriers.end(), [&](const auto& barrier) {
               return barrier.resource_index == resource_index && barrier.barrier_type == type &&
                      barrier.next_state == next_state;
             }) != plan.barriers.end();
    };
    EXPECT_TRUE(has_barrier(metadata_index, RenderGraphBarrierType::BufferMemory, RenderResourceState::ShaderRead));
    EXPECT_TRUE(
        has_barrier(irradiance_index, RenderGraphBarrierType::ImageLayout, RenderResourceState::TransferDestination));
    EXPECT_TRUE(has_barrier(irradiance_index, RenderGraphBarrierType::ImageLayout, RenderResourceState::ShaderRead));
    EXPECT_TRUE(
        has_barrier(visibility_index, RenderGraphBarrierType::ImageLayout, RenderResourceState::TransferDestination));
    EXPECT_TRUE(has_barrier(visibility_index, RenderGraphBarrierType::ImageLayout, RenderResourceState::ShaderRead));
    EXPECT_TRUE(
        has_barrier(variability_index, RenderGraphBarrierType::ImageLayout, RenderResourceState::TransferDestination));
    EXPECT_TRUE(has_barrier(variability_index, RenderGraphBarrierType::ImageLayout, RenderResourceState::ShaderRead));
  };

  RenderLayer::DdgiSettings settings;
  settings.volume_defaults.probe_counts = {2, 2, 2};
  settings.storage.atlas_probe_columns = 4;
  settings.storage.irradiance_tile_resolution = 6;
  settings.storage.visibility_tile_resolution = 10;
  settings.runtime.ray_count = 12;
  const auto initial_layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings);
  validate_layout(initial_layout);

  settings.volume_defaults.probe_counts = {5, 3, 2};
  settings.storage.atlas_probe_columns = 5;
  settings.storage.irradiance_tile_resolution = 10;
  settings.storage.visibility_tile_resolution = 18;
  settings.runtime.ray_count = 48;
  const auto resized_layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings);
  ASSERT_NE(resized_layout.irradiance_atlas.resolution, initial_layout.irradiance_atlas.resolution);
  ASSERT_NE(resized_layout.visibility_atlas.resolution, initial_layout.visibility_atlas.resolution);
  ASSERT_NE(resized_layout.ray_output_byte_size, initial_layout.ray_output_byte_size);
  validate_layout(resized_layout);
}

TEST(RenderGraph, ExecuteExposesCurrentPassTransitions) {
  constexpr const char* target = "Camera.TransitionTarget";
  RenderGraph graph;
  graph.AddResource({target, RenderResourceType::Image, RenderResourceLifetime::Camera});
  graph.AddPass({"Raster",
                 RenderPassQueue::Graphics,
                 RenderPassScope::Camera,
                 {{target, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
                []() {
                });
  graph.AddPass({"Denoise",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{target, RenderResourceUsage::ReadWrite, RenderResourceState::StorageReadWrite}},
                 {"Raster"}},
                [&](const RenderGraphExecutionContext& context) {
                  const auto transitions = context.GetCurrentPassTransitions();
                  ASSERT_EQ(transitions.size(), 1);
                  ASSERT_NE(transitions[0], nullptr);
                  EXPECT_EQ(transitions[0]->pass_index, context.GetCurrentPassIndex());
                  EXPECT_TRUE(transitions[0]->memory_dependency);
                  EXPECT_EQ(transitions[0]->previous_state, RenderResourceState::ColorAttachment);
                  EXPECT_EQ(transitions[0]->next_state, RenderResourceState::StorageReadWrite);
                  const auto* descriptor = context.GetResourceDescriptor(transitions[0]->resource_index);
                  ASSERT_NE(descriptor, nullptr);
                  EXPECT_EQ(descriptor->name, target);
                });
  graph.AddPass({"Sample",
                 RenderPassQueue::Graphics,
                 RenderPassScope::Camera,
                 {{target, RenderResourceUsage::Read, RenderResourceState::ShaderRead}},
                 {"Denoise"}},
                []() {
                });

  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  const RenderGraphResourceRegistry registry;
  graph.Execute(plan, registry);
}

TEST(RenderGraph, DefaultRasterCameraGraphPlansPostProcessingBoundary) {
  RenderGraph graph;
  AddDefaultRasterCameraResources(graph);
  graph.AddPass(
      {RenderPassNames::deferred_camera,
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
      []() {
      });
  graph.AddPass(
      {RenderPassNames::post_processing,
       RenderPassQueue::Compute,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::StorageReadWrite}},
       {RenderPassNames::deferred_camera}},
      []() {
      });

  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(plan.transitions.size(), 2);
  EXPECT_EQ(plan.transitions[1].pass_index, 1);
  EXPECT_TRUE(plan.transitions[1].memory_dependency);
  EXPECT_TRUE(plan.transitions[1].queue_change);
  EXPECT_EQ(plan.transitions[1].previous_queue, RenderPassQueue::Graphics);
  EXPECT_EQ(plan.transitions[1].next_queue, RenderPassQueue::Compute);
  EXPECT_EQ(plan.transitions[1].previous_state, RenderResourceState::ColorAttachment);
  EXPECT_EQ(plan.transitions[1].next_state, RenderResourceState::StorageReadWrite);
}

TEST(RenderGraph, DefaultRasterCameraGraphPlansDepthPyramidProducerBoundary) {
  RenderGraph graph;
  AddDefaultRasterCameraResources(graph);
  AddAdvancedCameraResources(graph);
  graph.AddPass(
      {RenderPassNames::deferred_geometry,
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_depth, RenderResourceUsage::Write, RenderResourceState::DepthAttachment},
        {RenderResourceNames::camera_g_buffer, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
      []() {
      });
  graph.AddPass(
      {RenderPassNames::depth_pyramid,
       RenderPassQueue::Compute,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
        {RenderResourceNames::camera_depth_pyramid, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}},
       {RenderPassNames::deferred_geometry}},
      []() {
      });
  graph.AddPass(
      {RenderPassNames::deferred_camera,
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
        {RenderResourceNames::camera_g_buffer, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
        {RenderResourceNames::camera_depth_pyramid, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
        {RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}},
       {RenderPassNames::depth_pyramid}},
      []() {
      });

  RenderGraphCompileContext context;
  context.camera_width = 1024;
  context.camera_height = 512;
  const auto plan = graph.Compile(context);

  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(plan.passes.size(), 3);
  ASSERT_EQ(plan.allocations.size(), 1);
  EXPECT_EQ(plan.passes[1].queue, RenderPassQueue::Compute);
  EXPECT_EQ(plan.passes[1].schedule_step_index, 1);
  EXPECT_EQ(plan.passes[2].schedule_step_index, 2);
  ASSERT_EQ(plan.passes[1].dependency_indices.size(), 1);
  EXPECT_EQ(plan.passes[1].dependency_indices[0], 0);
  EXPECT_NE(std::find(plan.passes[2].resource_dependency_indices.begin(),
                      plan.passes[2].resource_dependency_indices.end(), 1),
            plan.passes[2].resource_dependency_indices.end());

  const auto& resources = graph.GetResources();
  const auto depth_pyramid =
      std::find_if(resources.begin(), resources.end(), [](const RenderResourceDescriptor& descriptor) {
        return descriptor.name == RenderResourceNames::camera_depth_pyramid;
      });
  ASSERT_NE(depth_pyramid, resources.end());
  const auto depth_pyramid_index = static_cast<size_t>(depth_pyramid - resources.begin());
  ASSERT_LT(depth_pyramid_index, plan.resources.size());
  EXPECT_FALSE(plan.resources[depth_pyramid_index].imported);
  EXPECT_EQ(plan.resources[depth_pyramid_index].allocation_slot_index, plan.allocations[0].allocation_slot_index);
  EXPECT_EQ(plan.resources[depth_pyramid_index].resolved_dimensions.mip_levels, 11);
}

TEST(RenderGraph, CompileResolvesCameraRelativeAllocationDimensions) {
  constexpr const char* resolved_target = "Camera.ResolvedTarget";
  RenderGraph graph;
  graph.AddResource({resolved_target,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "RGBA16F",
                     1,
                     1,
                     true});
  graph.AddPass({"WriteResolvedTarget",
                 RenderPassQueue::Graphics,
                 RenderPassScope::Camera,
                 {{resolved_target, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
                []() {
                });

  RenderGraphCompileContext context;
  context.camera_width = 1280;
  context.camera_height = 720;
  const auto plan = graph.Compile(context);

  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(plan.resources.size(), 1);
  ASSERT_EQ(plan.allocations.size(), 1);
  EXPECT_EQ(plan.resources[0].resolved_dimensions.size_mode, RenderResourceSizeMode::Absolute);
  EXPECT_EQ(plan.resources[0].resolved_dimensions.width, 1280);
  EXPECT_EQ(plan.resources[0].resolved_dimensions.height, 720);
  EXPECT_EQ(plan.resources[0].resolved_dimensions.depth, 1);
  EXPECT_EQ(plan.resources[0].resolved_dimensions.layers, 1);
  EXPECT_EQ(plan.resources[0].resolved_dimensions.mip_levels, 1);
  EXPECT_EQ(plan.allocations[0].resolved_dimensions.width, 1280);
  EXPECT_EQ(plan.allocations[0].resolved_dimensions.height, 720);
  ASSERT_EQ(plan.allocations[0].required_states.size(), 1);
  EXPECT_EQ(plan.allocations[0].required_states[0], RenderResourceState::ColorAttachment);
}

TEST(RenderGraph, CompileComputesFullMipChainForZeroMipRelativeImages) {
  RenderGraph graph;
  graph.AddResource({RenderResourceNames::camera_depth_pyramid,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative, 0, 0, 1, 1, 0},
                     "DepthPyramid",
                     1,
                     1,
                     true});
  graph.AddPass({"BuildDepthPyramid",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{RenderResourceNames::camera_depth_pyramid, RenderResourceUsage::Write,
                   RenderResourceState::StorageReadWrite}}},
                []() {
                });

  RenderGraphCompileContext context;
  context.camera_width = 1024;
  context.camera_height = 512;
  const auto plan = graph.Compile(context);

  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(plan.resources.size(), 1);
  ASSERT_EQ(plan.allocations.size(), 1);
  EXPECT_EQ(plan.resources[0].resolved_dimensions.width, 1024);
  EXPECT_EQ(plan.resources[0].resolved_dimensions.height, 512);
  EXPECT_EQ(plan.resources[0].resolved_dimensions.mip_levels, 11);
  EXPECT_EQ(plan.allocations[0].resolved_dimensions.mip_levels, 11);
}

TEST(RenderGraph, CompileMergesRequiredStatesForAliasedAllocations) {
  constexpr const char* color_target = "Camera.ColorTransient";
  constexpr const char* compute_target = "Camera.ComputeTransient";
  RenderGraph graph;
  graph.AddResource({color_target,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "RGBA16F",
                     1,
                     1,
                     true});
  graph.AddResource({compute_target,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "RGBA16F",
                     1,
                     1,
                     true});
  graph.AddPass({"ColorPass",
                 RenderPassQueue::Graphics,
                 RenderPassScope::Camera,
                 {{color_target, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
                []() {
                });
  graph.AddPass({"ComputePass",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{compute_target, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}},
                 {"ColorPass"}},
                []() {
                });

  const auto plan = graph.Compile();

  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(plan.allocations.size(), 1);
  const auto& required_states = plan.allocations[0].required_states;
  EXPECT_NE(std::find(required_states.begin(), required_states.end(), RenderResourceState::ColorAttachment),
            required_states.end());
  EXPECT_NE(std::find(required_states.begin(), required_states.end(), RenderResourceState::StorageReadWrite),
            required_states.end());
}

TEST(RenderGraph, ExecuteProvidesCurrentPassPlanAndResourceBindings) {
  constexpr const char* transient_target = "Camera.ContextTarget";
  RenderGraph graph;
  graph.AddResource({transient_target,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "RGBA16F",
                     1,
                     1,
                     true});

  bool pass_executed = false;
  graph.AddPass({"ContextPass",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{transient_target, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}}},
                [&](const RenderGraphExecutionContext& context) {
                  pass_executed = true;
                  EXPECT_EQ(context.GetCurrentPassIndex(), 0);
                  ASSERT_NE(context.GetCurrentPassDescriptor(), nullptr);
                  EXPECT_EQ(context.GetCurrentPassDescriptor()->name, "ContextPass");
                  ASSERT_NE(context.GetResourceDescriptor(transient_target), nullptr);
                  ASSERT_NE(context.GetResourceUsagePlan(transient_target), nullptr);
                  ASSERT_NE(context.GetResourceAllocationPlan(transient_target), nullptr);
                  ASSERT_NE(context.GetResourceBinding(transient_target), nullptr);
                  EXPECT_TRUE(context.GetExecutionPlan().valid);
                });

  RenderGraphCompileContext compile_context;
  compile_context.camera_width = 640;
  compile_context.camera_height = 360;
  const auto plan = graph.Compile(compile_context);
  RenderGraphResourceRegistry registry;
  registry.BindImage(transient_target, {});
  graph.Execute(plan, registry);

  EXPECT_TRUE(pass_executed);
  EXPECT_TRUE(registry.HasResourceBinding(transient_target));
  ASSERT_EQ(registry.GetResourceBindings().size(), 1);
  EXPECT_EQ(registry.GetResourceBindings()[0].resource_name, transient_target);
}

TEST(RenderGraph, ResourceRegistryBindsAggregateImages) {
  constexpr const char* aggregate_target = "Camera.AggregateImages";
  RenderGraphResourceRegistry registry;
  std::vector<std::shared_ptr<Image>> images(2);
  registry.BindImages(aggregate_target, images);

  ASSERT_TRUE(registry.HasResourceBinding(aggregate_target));
  const auto* binding = registry.GetResourceBinding(aggregate_target);
  ASSERT_NE(binding, nullptr);
  EXPECT_EQ(binding->images.size(), 2);
  EXPECT_EQ(binding->image, images.front());
}

TEST(RenderGraph, CompilePlansAliasedImageResourcesForTransientStore) {
  constexpr const char* bloom_a = "Camera.BloomStoreA";
  constexpr const char* bloom_b = "Camera.BloomStoreB";
  RenderGraph graph;
  graph.AddResource({bloom_a,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "RGBA16F",
                     1,
                     1,
                     true});
  graph.AddResource({bloom_b,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "RGBA16F",
                     1,
                     1,
                     true});
  graph.AddPass({"WriteBloomA",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{bloom_a, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}}},
                []() {
                });
  graph.AddPass({"WriteBloomB",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{bloom_b, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}},
                 {"WriteBloomA"}},
                []() {
                });

  RenderGraphCompileContext context;
  context.camera_width = 320;
  context.camera_height = 180;
  const auto plan = graph.Compile(context);
  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(plan.allocations.size(), 1);
  ASSERT_EQ(plan.allocations[0].resource_indices.size(), 2);
  EXPECT_EQ(plan.allocations[0].resolved_dimensions.width, 320);
  EXPECT_EQ(plan.allocations[0].resolved_dimensions.height, 180);
}

TEST(RenderGraph, CompilePlansManagedBuffersWithByteSize) {
  constexpr const char* visibility_buffer = "Frame.TestVisibilityBuffer";
  RenderGraph graph;
  graph.AddResource({visibility_buffer,
                     RenderResourceType::Buffer,
                     RenderResourceLifetime::Frame,
                     {},
                     "Visibility",
                     1,
                     1,
                     true,
                     4096});
  graph.AddPass({"BuildVisibility",
                 RenderPassQueue::Compute,
                 RenderPassScope::Frame,
                 {{visibility_buffer, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}}},
                []() {
                });

  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(plan.allocations.size(), 1);
  EXPECT_EQ(plan.allocations[0].type, RenderResourceType::Buffer);
  EXPECT_EQ(plan.allocations[0].byte_size, 4096);
}

TEST(RenderGraph, TransientResourceStoreSkipsAllocationWithoutPlatform) {
  constexpr const char* transient_target = "Camera.NoPlatformTransient";
  RenderGraph graph;
  graph.AddResource({transient_target,
                     RenderResourceType::Image,
                     RenderResourceLifetime::Camera,
                     {RenderResourceSizeMode::CameraRelative},
                     "RGBA16F",
                     1,
                     1,
                     true});
  graph.AddPass({"WriteTransient",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{transient_target, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}}},
                []() {
                });

  RenderGraphCompileContext context;
  context.camera_width = 128;
  context.camera_height = 128;
  const auto plan = graph.Compile(context);
  RenderGraphTransientResourceStore store;
  RenderGraphResourceRegistry registry;
  store.Allocate(graph.GetResources(), plan);
  store.Bind(registry);

  EXPECT_TRUE(store.GetResourceBindings().empty());
  EXPECT_FALSE(registry.HasResourceBinding(transient_target));
}

TEST(RenderGraph, DefaultCameraResourcesRegisterCurrentRenderTargets) {
  RenderGraph raster_graph;
  AddDefaultRasterCameraResources(raster_graph);

  EXPECT_TRUE(raster_graph.HasResource(RenderResourceNames::frame_render_instances));
  EXPECT_TRUE(raster_graph.HasResource(RenderResourceNames::lighting_directional_shadow_map));
  EXPECT_TRUE(raster_graph.HasResource(RenderResourceNames::camera_depth));
  EXPECT_TRUE(raster_graph.HasResource(RenderResourceNames::camera_g_buffer));
  EXPECT_TRUE(raster_graph.HasResource(RenderResourceNames::camera_color));
  EXPECT_FALSE(raster_graph.HasResource(RenderResourceNames::camera_motion_vectors));

  RenderGraph ray_tracing_graph;
  AddDefaultRayTracingCameraResources(ray_tracing_graph);

  EXPECT_TRUE(ray_tracing_graph.HasResource(RenderResourceNames::frame_ray_tracing_descriptor_set));
  EXPECT_TRUE(ray_tracing_graph.HasResource(RenderResourceNames::scene_mesh_tlas));
  EXPECT_TRUE(ray_tracing_graph.HasResource(RenderResourceNames::camera_color));
  EXPECT_TRUE(ray_tracing_graph.HasResource(RenderResourceNames::camera_ray_hit_distance));
}

TEST(RenderGraph, VolumetricCloudResourcesRegisterCameraRelativeAccumulationTargets) {
  RenderGraph graph;
  AddVolumetricCloudCameraResources(graph, 2);

  const auto& resources = graph.GetResources();
  const auto accumulation =
      std::find_if(resources.begin(), resources.end(), [](const RenderResourceDescriptor& descriptor) {
        return descriptor.name == RenderResourceNames::camera_volumetric_cloud_accumulation;
      });
  const auto transmittance =
      std::find_if(resources.begin(), resources.end(), [](const RenderResourceDescriptor& descriptor) {
        return descriptor.name == RenderResourceNames::camera_volumetric_cloud_transmittance;
      });

  ASSERT_NE(accumulation, resources.end());
  EXPECT_EQ(accumulation->type, RenderResourceType::Image);
  EXPECT_EQ(accumulation->lifetime, RenderResourceLifetime::Camera);
  EXPECT_EQ(accumulation->dimensions.size_mode, RenderResourceSizeMode::CameraRelative);
  EXPECT_EQ(accumulation->dimensions.width, 2u);
  EXPECT_EQ(accumulation->dimensions.height, 2u);
  EXPECT_EQ(accumulation->format_name, "RGBA16F");
  EXPECT_TRUE(accumulation->managed_by_graph);

  ASSERT_NE(transmittance, resources.end());
  EXPECT_EQ(transmittance->type, RenderResourceType::Image);
  EXPECT_EQ(transmittance->lifetime, RenderResourceLifetime::Camera);
  EXPECT_EQ(transmittance->dimensions.size_mode, RenderResourceSizeMode::CameraRelative);
  EXPECT_EQ(transmittance->dimensions.width, 2u);
  EXPECT_EQ(transmittance->dimensions.height, 2u);
  EXPECT_EQ(transmittance->format_name, "R16F");
  EXPECT_TRUE(transmittance->managed_by_graph);

  const auto plan = graph.Compile({0, 0, 1280, 720});
  ASSERT_TRUE(plan.valid);
  const auto accumulation_index = std::distance(resources.begin(), accumulation);
  EXPECT_EQ(plan.resources[accumulation_index].resolved_dimensions.width, 640u);
  EXPECT_EQ(plan.resources[accumulation_index].resolved_dimensions.height, 360u);
}

TEST(RenderGraph, VolumetricCloudRasterAndRayTracingDescriptorsUseSharedCameraPass) {
  const auto raster_descriptor = VolumetricCloudsPass::CreateRasterDescriptor(RenderPassNames::deferred_camera);
  const auto ray_descriptor = VolumetricCloudsPass::CreateRayTracingDescriptor(RenderPassNames::ray_tracing_camera);

  EXPECT_EQ(raster_descriptor.name, RenderPassNames::volumetric_clouds);
  EXPECT_EQ(ray_descriptor.name, RenderPassNames::volumetric_clouds);
  EXPECT_EQ(raster_descriptor.queue, RenderPassQueue::Graphics);
  EXPECT_EQ(ray_descriptor.queue, RenderPassQueue::Graphics);
  EXPECT_EQ(raster_descriptor.scope, RenderPassScope::Camera);
  EXPECT_EQ(ray_descriptor.scope, RenderPassScope::Camera);
  ASSERT_EQ(raster_descriptor.dependencies.size(), 1);
  ASSERT_EQ(ray_descriptor.dependencies.size(), 1);
  EXPECT_EQ(raster_descriptor.dependencies[0], RenderPassNames::deferred_camera);
  EXPECT_EQ(ray_descriptor.dependencies[0], RenderPassNames::ray_tracing_camera);

  ASSERT_EQ(raster_descriptor.resources.size(), ray_descriptor.resources.size());
  EXPECT_EQ(raster_descriptor.resources[0].resource_name, RenderResourceNames::camera_depth);
  EXPECT_EQ(ray_descriptor.resources[0].resource_name, RenderResourceNames::camera_ray_hit_distance);
  for (size_t resource_index = 1; resource_index < raster_descriptor.resources.size(); ++resource_index) {
    EXPECT_EQ(raster_descriptor.resources[resource_index].resource_name,
              ray_descriptor.resources[resource_index].resource_name);
    EXPECT_EQ(raster_descriptor.resources[resource_index].usage, ray_descriptor.resources[resource_index].usage);
    EXPECT_EQ(raster_descriptor.resources[resource_index].state, ray_descriptor.resources[resource_index].state);
  }
}

TEST(RenderGraph, GaussianSplatDescriptorCompositesAfterDeferredLighting) {
  const auto cull_descriptor = GaussianSplatCullPass::CreateDescriptor(nullptr);

  EXPECT_EQ(cull_descriptor.name, RenderPassNames::gaussian_splat_cull);
  EXPECT_EQ(cull_descriptor.queue, RenderPassQueue::Graphics);
  EXPECT_EQ(cull_descriptor.scope, RenderPassScope::Camera);
  ASSERT_EQ(cull_descriptor.dependencies.size(), 1);
  EXPECT_EQ(cull_descriptor.dependencies[0], RenderPassNames::deferred_camera);
  ASSERT_EQ(cull_descriptor.resources.size(), 3);
  EXPECT_EQ(cull_descriptor.resources[0].resource_name, RenderResourceNames::frame_render_instances);
  EXPECT_EQ(cull_descriptor.resources[0].usage, RenderResourceUsage::Read);
  EXPECT_EQ(cull_descriptor.resources[0].state, RenderResourceState::ShaderRead);
  EXPECT_EQ(cull_descriptor.resources[1].resource_name, RenderResourceNames::frame_per_frame_descriptor_set);
  EXPECT_EQ(cull_descriptor.resources[1].usage, RenderResourceUsage::Read);
  EXPECT_EQ(cull_descriptor.resources[1].state, RenderResourceState::General);
  EXPECT_EQ(cull_descriptor.resources[2].resource_name, RenderResourceNames::camera_gaussian_splat_prepass);
  EXPECT_EQ(cull_descriptor.resources[2].usage, RenderResourceUsage::Write);
  EXPECT_EQ(cull_descriptor.resources[2].state, RenderResourceState::StorageReadWrite);

  const auto sort_descriptor = GaussianSplatSortPass::CreateDescriptor(nullptr);

  EXPECT_EQ(sort_descriptor.name, RenderPassNames::gaussian_splat_sort);
  EXPECT_EQ(sort_descriptor.queue, RenderPassQueue::Graphics);
  EXPECT_EQ(sort_descriptor.scope, RenderPassScope::Camera);
  ASSERT_EQ(sort_descriptor.dependencies.size(), 1);
  EXPECT_EQ(sort_descriptor.dependencies[0], RenderPassNames::gaussian_splat_cull);
  ASSERT_EQ(sort_descriptor.resources.size(), 2);
  EXPECT_EQ(sort_descriptor.resources[0].resource_name, RenderResourceNames::frame_render_instances);
  EXPECT_EQ(sort_descriptor.resources[0].usage, RenderResourceUsage::Read);
  EXPECT_EQ(sort_descriptor.resources[0].state, RenderResourceState::ShaderRead);
  EXPECT_EQ(sort_descriptor.resources[1].resource_name, RenderResourceNames::camera_gaussian_splat_prepass);
  EXPECT_EQ(sort_descriptor.resources[1].usage, RenderResourceUsage::ReadWrite);
  EXPECT_EQ(sort_descriptor.resources[1].state, RenderResourceState::StorageReadWrite);

  const auto descriptor = GaussianSplatPass::CreateDescriptor(nullptr);

  EXPECT_EQ(descriptor.name, RenderPassNames::gaussian_splat);
  EXPECT_EQ(descriptor.queue, RenderPassQueue::Graphics);
  EXPECT_EQ(descriptor.scope, RenderPassScope::Camera);
  ASSERT_EQ(descriptor.dependencies.size(), 1);
  EXPECT_EQ(descriptor.dependencies[0], RenderPassNames::deferred_camera);
  ASSERT_EQ(descriptor.resources.size(), 5);
  EXPECT_EQ(descriptor.resources[0].resource_name, RenderResourceNames::frame_render_instances);
  EXPECT_EQ(descriptor.resources[0].usage, RenderResourceUsage::Read);
  EXPECT_EQ(descriptor.resources[0].state, RenderResourceState::ShaderRead);
  EXPECT_EQ(descriptor.resources[1].resource_name, RenderResourceNames::frame_per_frame_descriptor_set);
  EXPECT_EQ(descriptor.resources[1].usage, RenderResourceUsage::Read);
  EXPECT_EQ(descriptor.resources[1].state, RenderResourceState::General);
  EXPECT_EQ(descriptor.resources[2].resource_name, RenderResourceNames::camera_gaussian_splat_prepass);
  EXPECT_EQ(descriptor.resources[2].usage, RenderResourceUsage::Read);
  EXPECT_EQ(descriptor.resources[2].state, RenderResourceState::ShaderRead);
  EXPECT_EQ(descriptor.resources[3].resource_name, RenderResourceNames::camera_depth);
  EXPECT_EQ(descriptor.resources[3].usage, RenderResourceUsage::Read);
  EXPECT_EQ(descriptor.resources[3].state, RenderResourceState::DepthAttachment);
  EXPECT_EQ(descriptor.resources[4].resource_name, RenderResourceNames::camera_color);
  EXPECT_EQ(descriptor.resources[4].usage, RenderResourceUsage::ReadWrite);
  EXPECT_EQ(descriptor.resources[4].state, RenderResourceState::ColorAttachment);

  const auto cloud_dependent_descriptor = GaussianSplatPass::CreateDescriptor(RenderPassNames::volumetric_clouds);
  ASSERT_EQ(cloud_dependent_descriptor.dependencies.size(), 1);
  EXPECT_EQ(cloud_dependent_descriptor.dependencies[0], RenderPassNames::volumetric_clouds);

  const auto overlay_descriptor = GaussianSplatPass::CreateOverlayDescriptor(nullptr);
  EXPECT_EQ(overlay_descriptor.name, RenderPassNames::gaussian_splat);
  EXPECT_EQ(overlay_descriptor.queue, RenderPassQueue::Graphics);
  EXPECT_EQ(overlay_descriptor.scope, RenderPassScope::Camera);
  ASSERT_EQ(overlay_descriptor.dependencies.size(), 1);
  EXPECT_EQ(overlay_descriptor.dependencies[0], RenderPassNames::ray_tracing_camera);
  ASSERT_EQ(overlay_descriptor.resources.size(), 4);
  EXPECT_EQ(overlay_descriptor.resources[0].resource_name, RenderResourceNames::frame_render_instances);
  EXPECT_EQ(overlay_descriptor.resources[1].resource_name, RenderResourceNames::frame_per_frame_descriptor_set);
  EXPECT_EQ(overlay_descriptor.resources[2].resource_name, RenderResourceNames::camera_gaussian_splat_prepass);
  EXPECT_EQ(overlay_descriptor.resources[2].usage, RenderResourceUsage::Read);
  EXPECT_EQ(overlay_descriptor.resources[2].state, RenderResourceState::ShaderRead);
  EXPECT_EQ(overlay_descriptor.resources[3].resource_name, RenderResourceNames::camera_color);
  EXPECT_EQ(overlay_descriptor.resources[3].usage, RenderResourceUsage::ReadWrite);
  EXPECT_EQ(overlay_descriptor.resources[3].state, RenderResourceState::ColorAttachment);
}

TEST(RenderGraph, GaussianSplatPassRunsAfterCloudsBeforePostProcessing) {
  RenderGraph graph;
  AddDefaultRasterCameraResources(graph);
  AddVolumetricCloudCameraResources(graph);
  AddGaussianSplatCameraResources(graph);

  graph.AddPass(
      {RenderPassNames::deferred_camera,
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
      []() {
      });
  graph.AddPass(VolumetricCloudsPass::CreateRasterDescriptor(RenderPassNames::deferred_camera), []() {
  });
  graph.AddPass(GaussianSplatCullPass::CreateDescriptor(RenderPassNames::volumetric_clouds), []() {
  });
  graph.AddPass(GaussianSplatSortPass::CreateDescriptor(RenderPassNames::gaussian_splat_cull), []() {
  });
  graph.AddPass(GaussianSplatPass::CreateDescriptor(RenderPassNames::gaussian_splat_sort), []() {
  });
  graph.AddPass(PostProcessingPass::CreateDescriptor(RenderPassNames::gaussian_splat), []() {
  });

  ASSERT_TRUE(graph.Validate());
  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(graph.GetPasses().size(), 6);
  EXPECT_EQ(graph.GetPasses()[1].name, RenderPassNames::volumetric_clouds);
  EXPECT_EQ(graph.GetPasses()[2].name, RenderPassNames::gaussian_splat_cull);
  EXPECT_EQ(graph.GetPasses()[3].name, RenderPassNames::gaussian_splat_sort);
  EXPECT_EQ(graph.GetPasses()[4].name, RenderPassNames::gaussian_splat);
  ASSERT_EQ(graph.GetPasses()[2].dependencies.size(), 1);
  EXPECT_EQ(graph.GetPasses()[2].dependencies[0], RenderPassNames::volumetric_clouds);
  ASSERT_EQ(graph.GetPasses()[3].dependencies.size(), 1);
  EXPECT_EQ(graph.GetPasses()[3].dependencies[0], RenderPassNames::gaussian_splat_cull);
  ASSERT_EQ(graph.GetPasses()[4].dependencies.size(), 1);
  EXPECT_EQ(graph.GetPasses()[4].dependencies[0], RenderPassNames::gaussian_splat_sort);
  ASSERT_EQ(graph.GetPasses()[5].dependencies.size(), 1);
  EXPECT_EQ(graph.GetPasses()[5].dependencies[0], RenderPassNames::gaussian_splat);
  ASSERT_EQ(plan.passes[2].dependency_indices.size(), 1);
  EXPECT_EQ(plan.passes[2].dependency_indices[0], 1);
  ASSERT_EQ(plan.passes[3].dependency_indices.size(), 1);
  EXPECT_EQ(plan.passes[3].dependency_indices[0], 2);
  ASSERT_EQ(plan.passes[4].dependency_indices.size(), 1);
  EXPECT_EQ(plan.passes[4].dependency_indices[0], 3);
  ASSERT_EQ(plan.passes[5].dependency_indices.size(), 1);
  EXPECT_EQ(plan.passes[5].dependency_indices[0], 4);

  const auto color_transition = std::find_if(
      plan.transitions.begin(), plan.transitions.end(), [&](const RenderResourceTransitionPlan& transition) {
        return graph.GetResources()[transition.resource_index].name == RenderResourceNames::camera_color &&
               transition.pass_index == 4;
      });
  ASSERT_NE(color_transition, plan.transitions.end());
  EXPECT_EQ(color_transition->previous_state, RenderResourceState::StorageReadWrite);
  EXPECT_EQ(color_transition->next_state, RenderResourceState::ColorAttachment);

  const auto prepass_transition = std::find_if(plan.transitions.begin(), plan.transitions.end(),
                                               [&](const RenderResourceTransitionPlan& transition) {
                                                 return graph.GetResources()[transition.resource_index].name ==
                                                            RenderResourceNames::camera_gaussian_splat_prepass &&
                                                        transition.pass_index == 4;
                                               });
  ASSERT_NE(prepass_transition, plan.transitions.end());
  EXPECT_EQ(prepass_transition->previous_state, RenderResourceState::StorageReadWrite);
  EXPECT_EQ(prepass_transition->next_state, RenderResourceState::ShaderRead);
}

TEST(RenderGraph, VolumetricCloudRasterPassRunsBetweenDeferredLightingAndPostProcessing) {
  RenderGraph graph;
  AddDefaultRasterCameraResources(graph);
  AddVolumetricCloudCameraResources(graph);

  graph.AddPass(
      {RenderPassNames::deferred_camera,
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
      []() {
      });
  graph.AddPass(VolumetricCloudsPass::CreateRasterDescriptor(RenderPassNames::deferred_camera), []() {
  });
  graph.AddPass(PostProcessingPass::CreateDescriptor(RenderPassNames::volumetric_clouds), []() {
  });

  ASSERT_TRUE(graph.Validate());
  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(graph.GetPasses().size(), 3);
  EXPECT_EQ(graph.GetPasses()[1].name, RenderPassNames::volumetric_clouds);
  ASSERT_EQ(graph.GetPasses()[2].dependencies.size(), 1);
  EXPECT_EQ(graph.GetPasses()[2].dependencies[0], RenderPassNames::volumetric_clouds);
  ASSERT_EQ(plan.passes[1].dependency_indices.size(), 1);
  EXPECT_EQ(plan.passes[1].dependency_indices[0], 0);
  ASSERT_EQ(plan.passes[2].dependency_indices.size(), 1);
  EXPECT_EQ(plan.passes[2].dependency_indices[0], 1);

  const auto color_transition = std::find_if(
      plan.transitions.begin(), plan.transitions.end(), [&](const RenderResourceTransitionPlan& transition) {
        return graph.GetResources()[transition.resource_index].name == RenderResourceNames::camera_color &&
               transition.pass_index == 1;
      });
  ASSERT_NE(color_transition, plan.transitions.end());
  EXPECT_EQ(color_transition->previous_state, RenderResourceState::ColorAttachment);
  EXPECT_EQ(color_transition->next_state, RenderResourceState::StorageReadWrite);
}

TEST(RenderGraph, VolumetricCloudRayTracingPassConsumesRayHitDistanceAfterRayTracingCamera) {
  RenderGraph graph;
  AddDefaultRayTracingCameraResources(graph);
  AddVolumetricCloudCameraResources(graph);

  graph.AddPass(RayTracingCameraPass::CreateDescriptor(), []() {
  });
  graph.AddPass(VolumetricCloudsPass::CreateRayTracingDescriptor(RenderPassNames::ray_tracing_camera), []() {
  });

  ASSERT_TRUE(graph.Validate());
  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(graph.GetPasses().size(), 2);
  EXPECT_EQ(graph.GetPasses()[0].name, RenderPassNames::ray_tracing_camera);
  EXPECT_EQ(graph.GetPasses()[1].name, RenderPassNames::volumetric_clouds);
  ASSERT_EQ(graph.GetPasses()[1].dependencies.size(), 1);
  EXPECT_EQ(graph.GetPasses()[1].dependencies[0], RenderPassNames::ray_tracing_camera);
  ASSERT_EQ(plan.passes[1].dependency_indices.size(), 1);
  EXPECT_EQ(plan.passes[1].dependency_indices[0], 0);

  const auto& ray_resources = graph.GetPasses()[0].resources;
  EXPECT_NE(std::find_if(ray_resources.begin(), ray_resources.end(),
                         [](const RenderResourceAccess& access) {
                           return access.resource_name == RenderResourceNames::camera_ray_hit_distance &&
                                  access.usage == RenderResourceUsage::Write;
                         }),
            ray_resources.end());
  const auto& cloud_resources = graph.GetPasses()[1].resources;
  EXPECT_NE(std::find_if(cloud_resources.begin(), cloud_resources.end(),
                         [](const RenderResourceAccess& access) {
                           return access.resource_name == RenderResourceNames::camera_ray_hit_distance &&
                                  access.usage == RenderResourceUsage::Read;
                         }),
            cloud_resources.end());
}

TEST(RenderGraph, GaussianSplatOverlayRunsAfterRayTracingClouds) {
  RenderGraph graph;
  AddDefaultRayTracingCameraResources(graph);
  AddVolumetricCloudCameraResources(graph);
  AddGaussianSplatCameraResources(graph);

  graph.AddPass(RayTracingCameraPass::CreateDescriptor(), []() {
  });
  graph.AddPass(VolumetricCloudsPass::CreateRayTracingDescriptor(RenderPassNames::ray_tracing_camera), []() {
  });
  graph.AddPass(GaussianSplatCullPass::CreateDescriptor(RenderPassNames::volumetric_clouds), []() {
  });
  graph.AddPass(GaussianSplatSortPass::CreateDescriptor(RenderPassNames::gaussian_splat_cull), []() {
  });
  graph.AddPass(GaussianSplatPass::CreateOverlayDescriptor(RenderPassNames::gaussian_splat_sort), []() {
  });

  ASSERT_TRUE(graph.Validate());
  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(graph.GetPasses().size(), 5);
  EXPECT_EQ(graph.GetPasses()[0].name, RenderPassNames::ray_tracing_camera);
  EXPECT_EQ(graph.GetPasses()[1].name, RenderPassNames::volumetric_clouds);
  EXPECT_EQ(graph.GetPasses()[2].name, RenderPassNames::gaussian_splat_cull);
  EXPECT_EQ(graph.GetPasses()[3].name, RenderPassNames::gaussian_splat_sort);
  EXPECT_EQ(graph.GetPasses()[4].name, RenderPassNames::gaussian_splat);
  ASSERT_EQ(graph.GetPasses()[2].dependencies.size(), 1);
  EXPECT_EQ(graph.GetPasses()[2].dependencies[0], RenderPassNames::volumetric_clouds);
  ASSERT_EQ(graph.GetPasses()[3].dependencies.size(), 1);
  EXPECT_EQ(graph.GetPasses()[3].dependencies[0], RenderPassNames::gaussian_splat_cull);
  ASSERT_EQ(graph.GetPasses()[4].dependencies.size(), 1);
  EXPECT_EQ(graph.GetPasses()[4].dependencies[0], RenderPassNames::gaussian_splat_sort);
  ASSERT_EQ(plan.passes[2].dependency_indices.size(), 1);
  EXPECT_EQ(plan.passes[2].dependency_indices[0], 1);
  ASSERT_EQ(plan.passes[3].dependency_indices.size(), 1);
  EXPECT_EQ(plan.passes[3].dependency_indices[0], 2);
  ASSERT_EQ(plan.passes[4].dependency_indices.size(), 1);
  EXPECT_EQ(plan.passes[4].dependency_indices[0], 3);

  const auto color_transition = std::find_if(
      plan.transitions.begin(), plan.transitions.end(), [&](const RenderResourceTransitionPlan& transition) {
        return graph.GetResources()[transition.resource_index].name == RenderResourceNames::camera_color &&
               transition.pass_index == 4;
      });
  ASSERT_NE(color_transition, plan.transitions.end());
  EXPECT_EQ(color_transition->previous_state, RenderResourceState::StorageReadWrite);
  EXPECT_EQ(color_transition->next_state, RenderResourceState::ColorAttachment);
}

TEST(RenderGraph, AdvancedResourcesDescribeHistoryAndVisibilityInputs) {
  RenderGraph graph;
  AddAdvancedFrameResources(graph);
  AddAdvancedCameraResources(graph);

  EXPECT_TRUE(graph.HasResource(RenderResourceNames::frame_visibility_buffer));
  EXPECT_TRUE(graph.HasResource(RenderResourceNames::camera_motion_vectors));
  EXPECT_TRUE(graph.HasResource(RenderResourceNames::camera_object_id));
  EXPECT_TRUE(graph.HasResource(RenderResourceNames::camera_material_id));
  EXPECT_TRUE(graph.HasResource(RenderResourceNames::camera_depth_pyramid));

  const auto& resources = graph.GetResources();
  const auto color_history =
      std::find_if(resources.begin(), resources.end(), [](const RenderResourceDescriptor& descriptor) {
        return descriptor.name == RenderResourceNames::camera_color_history;
      });

  ASSERT_NE(color_history, resources.end());
  EXPECT_EQ(color_history->lifetime, RenderResourceLifetime::History);
  EXPECT_EQ(color_history->dimensions.size_mode, RenderResourceSizeMode::CameraRelative);
  EXPECT_EQ(color_history->history_length, 2);
}
