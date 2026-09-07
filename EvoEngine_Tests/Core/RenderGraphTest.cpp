#include "EvoEngine_SDK_PCH.hpp"

#include "GpuProfiler.hpp"
#include "Platform.hpp"
#include "RenderGraph.hpp"
#include "RenderLayer.hpp"
#include "RenderPasses/DdgiAtlasPreparePass.hpp"
#include "RenderPasses/DdgiProbeClassificationPass.hpp"
#include "RenderPasses/DdgiProbeRayVisualizationPass.hpp"
#include "RenderPasses/DdgiProbeRelocationPass.hpp"
#include "RenderPasses/DdgiProbeScrollPass.hpp"
#include "RenderPasses/DdgiProbeTracePass.hpp"
#include "RenderPasses/DdgiProbeUpdatePass.hpp"
#include "RenderPasses/DdgiProbeVisualizationPass.hpp"
#include "RenderPasses/DeferredComputeLightingPass.hpp"
#include "RenderPasses/DeferredGeometryPass.hpp"
#include "RenderPasses/DepthPyramidPass.hpp"
#include "RenderPasses/DirectionalLightShadowPass.hpp"
#include "RenderPasses/EntitySelectionHighlightPass.hpp"
#include "RenderPasses/GaussianSplatPass.hpp"
#include "RenderPasses/MotionCoveragePass.hpp"
#include "RenderPasses/MotionVectorPass.hpp"
#include "RenderPasses/PostProcessingPass.hpp"
#include "RenderPasses/RayTracingCameraPass.hpp"
#include "RenderPasses/TransparentGeometryPass.hpp"
#include "RenderPasses/VolumetricCloudsPass.hpp"
#include "Rendering/RenderInstances/RenderInstanceStorage.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <utility>

using namespace evo_engine;

namespace {
constexpr const char* kTestDebugOutputResource = "Frame.TestDebugOutput";

std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::filesystem::path SdkPath(const std::filesystem::path& relative_path) {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / relative_path;
}

std::filesystem::path SourcePath(const std::filesystem::path& relative_path) {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / relative_path;
}
}  // namespace

TEST(RenderGraph, RenderPassDrawStatsExposeDirectIndirectBreakdown) {
  RenderPassDrawStats stats;
  stats.direct_draw_calls = 2;
  stats.indirect_draw_calls = 3;
  stats.indirect_draw_commands = 10;
  stats.prim_count = 42;

  EXPECT_EQ(stats.TotalDrawCalls(), 5);
  EXPECT_EQ(Platform::kRenderPassDrawBucketCount, static_cast<size_t>(RenderPassDrawBucket::Count));
  EXPECT_STREQ("Deferred geometry", Platform::GetRenderPassDrawBucketName(RenderPassDrawBucket::DeferredGeometry));
  EXPECT_STREQ("Directional shadow",
               Platform::GetRenderPassDrawBucketName(RenderPassDrawBucket::DirectionalLightShadow));
  EXPECT_STREQ("Editor gizmos", Platform::GetRenderPassDrawBucketName(RenderPassDrawBucket::EditorGizmos));
}

TEST(RenderGraph, DirectionalShadowCasterPathsUseProductionRenderers) {
  const auto render_layer_header = ReadTextFile(SdkPath("include/Layers/RenderLayer.hpp"));
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto render_instance_storage = ReadTextFile(SdkPath("src/RenderInstanceStorage.cpp"));
  const auto platform = ReadTextFile(SdkPath("src/Platform.cpp"));
  const auto directional_shadow = ReadTextFile(SdkPath("src/RenderPasses/DirectionalLightShadowPass.cpp"));

  EXPECT_NE(render_layer_header.find("glm::mat4 light_space_matrix"), std::string::npos);
  for (const auto renderer : {"ForEachMeshRenderInstance", "ForEachInstancedRenderInstance",
                              "ForEachSkinnedMeshRenderInstance", "ForEachStrandsRenderInstance"}) {
    EXPECT_NE(directional_shadow.find(renderer), std::string::npos) << renderer;
  }
  EXPECT_NE(directional_shadow.find("DrawMeshTasksIndirect"), std::string::npos);
  EXPECT_NE(directional_shadow.find("DrawIndexedIndirect"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("if (graphics_pipeline->mesh_shader)"), std::string::npos);
  EXPECT_EQ(render_instance_storage.find("if (Platform::MeshShaderEnabled())"), std::string::npos);
  EXPECT_NE(platform.find("VK_FORMAT_FEATURE_2_SAMPLED_IMAGE_DEPTH_COMPARISON_BIT"), std::string::npos);
  EXPECT_NE(platform.find("VK_KHR_FORMAT_FEATURE_FLAGS_2_EXTENSION_NAME"), std::string::npos);
  EXPECT_NE(platform.find("vkGetPhysicalDeviceImageFormatProperties"), std::string::npos);
  EXPECT_EQ(render_layer.find("PointLightShadowMapStrands"), std::string::npos);
  EXPECT_EQ(render_layer.find("SpotLightShadowMapStrands"), std::string::npos);
  EXPECT_NE(render_layer.find("PointLightStrandsShadowMap.slang"), std::string::npos);
  EXPECT_NE(render_layer.find("SpotLightStrandsShadowMap.slang"), std::string::npos);
  EXPECT_NE(render_layer_header.find("strands_directional_light_shadow_pipeline"), std::string::npos);
  EXPECT_NE(render_layer.find("DirectionalLightStrandsShadowMap"), std::string::npos);
  EXPECT_NE(directional_shadow.find("AccountDraws(parameters.count_draw_calls"), std::string::npos);
  EXPECT_NE(render_layer.find("Platform::CountRenderPassDraw(RenderPassDrawBucket::DirectionalLightShadow"),
            std::string::npos);
}

TEST(RenderGraph, RenderPassDrawCountersRouteRasterAccountingByPass) {
  const auto platform_header = ReadTextFile(SdkPath("include/Rendering/Platform/Platform.hpp"));
  const auto platform_source = ReadTextFile(SdkPath("src/Platform.cpp"));
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto editor_layer = ReadTextFile(SdkPath("src/EditorLayer.cpp"));
  const auto deferred_geometry = ReadTextFile(SdkPath("src/RenderPasses/DeferredGeometryPass.cpp"));
  const auto directional_shadow = ReadTextFile(SdkPath("src/RenderPasses/DirectionalLightShadowPass.cpp"));
  const auto transparent = ReadTextFile(SdkPath("src/RenderPasses/TransparentGeometryPass.cpp"));
  ASSERT_FALSE(platform_header.empty());
  ASSERT_FALSE(platform_source.empty());
  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(editor_layer.empty());
  ASSERT_FALSE(deferred_geometry.empty());
  ASSERT_FALSE(directional_shadow.empty());
  ASSERT_FALSE(transparent.empty());

  EXPECT_NE(platform_header.find("render_pass_draw_stats"), std::string::npos);
  EXPECT_NE(platform_source.find("graphics.render_pass_draw_stats.resize"), std::string::npos);
  EXPECT_NE(platform_source.find("stats.indirect_draw_commands += indirect_draw_commands"), std::string::npos);
  EXPECT_NE(render_layer.find("Platform::ResetRenderPassDrawStats(current_frame_index)"), std::string::npos);
  EXPECT_NE(editor_layer.find("DrawRenderCounterSummary"), std::string::npos);

  EXPECT_NE(deferred_geometry.find("RenderPassDrawBucket::DeferredGeometry"), std::string::npos);
  EXPECT_NE(deferred_geometry.find("RenderDrawCallKind::Indirect"), std::string::npos);
  EXPECT_NE(directional_shadow.find("RenderPassDrawBucket::DirectionalLightShadow"), std::string::npos);
  EXPECT_NE(transparent.find("RenderPassDrawBucket::TransparentGeometry"), std::string::npos);
  EXPECT_NE(render_layer.find("RenderPassDrawBucket::PointLightShadow"), std::string::npos);
  EXPECT_NE(render_layer.find("RenderPassDrawBucket::SpotLightShadow"), std::string::npos);
  EXPECT_NE(render_layer.find("RenderPassDrawBucket::ForwardExternal"), std::string::npos);
  EXPECT_EQ(render_layer.find("platform.draw_call"), std::string::npos);
  EXPECT_EQ(deferred_geometry.find("platform.draw_call"), std::string::npos);
  EXPECT_EQ(directional_shadow.find("platform.draw_call"), std::string::npos);
  EXPECT_EQ(transparent.find("platform.draw_call"), std::string::npos);
}

TEST(RenderGraph, ProfilerBreakdownUsesSharedCpuAndGpuHistoryModels) {
  const auto editor_layer = ReadTextFile(SdkPath("src/EditorLayer.cpp"));
  EXPECT_NE(editor_layer.find("BeginTabItem(\"Breakdown\")"), std::string::npos);
  EXPECT_NE(editor_layer.find("BuildProfilerHistoryStats(profiler_panel_frames_, selected_index)"), std::string::npos);
  EXPECT_NE(editor_layer.find("BuildGpuTimestampHistoryStats("), std::string::npos);
  EXPECT_NE(editor_layer.find("breakdown_next_refresh"), std::string::npos);
  EXPECT_NE(editor_layer.find("Loop self (uncategorized)"), std::string::npos);
  EXPECT_NE(editor_layer.find("Frame overhead (outside Application Loop)"), std::string::npos);
  EXPECT_NE(editor_layer.find("Worker CPU work (parallel; excluded from wall)"), std::string::npos);
  EXPECT_NE(editor_layer.find("ProfilerBreakdownCpuPlot"), std::string::npos);
  EXPECT_NE(editor_layer.find("ProfilerBreakdownGpuPlot"), std::string::npos);
  EXPECT_NE(editor_layer.find("pass.duration.duty_cycle"), std::string::npos);
}

TEST(RenderGraph, RegisteredGpuPassOccurrencesAggregateAndSummaryScopesDoNotDoubleCount) {
  GpuTimestampFrameSnapshot frame;
  frame.results_available = true;
  const GpuTimestampScopeMetadata stage{"EcoSysLab.DynamicStrands.Prediction",
                                        "Prediction",
                                        "DynamicStrands",
                                        GpuTimestampQueue::Compute,
                                        0,
                                        0,
                                        true,
                                        "EcoSysLab"};
  const GpuTimestampScopeMetadata summary{"EcoSysLab.DynamicStrands.Simulation",
                                          "DynamicStrands Simulation",
                                          "DynamicStrands",
                                          GpuTimestampQueue::Compute,
                                          0,
                                          0,
                                          false,
                                          "EcoSysLab"};
  frame.samples = {{summary, 0, 0, 1, 0.0, 3.0, 3.0}, {stage, 1, 2, 3, 0.2, 1.2, 1.0}, {stage, 2, 4, 5, 1.4, 2.9, 1.5}};

  const auto aggregates = BuildGpuTimestampPassAggregates(frame);
  ASSERT_EQ(aggregates.size(), 2);
  const auto stage_aggregate = std::find_if(aggregates.begin(), aggregates.end(), [](const auto& aggregate) {
    return aggregate.metadata.stable_pass_id == "EcoSysLab.DynamicStrands.Prediction";
  });
  ASSERT_NE(stage_aggregate, aggregates.end());
  EXPECT_EQ(stage_aggregate->call_count, 2);
  EXPECT_DOUBLE_EQ(stage_aggregate->total_milliseconds, 2.5);
  EXPECT_EQ(stage_aggregate->metadata.instance_id, 0);

  const auto history = BuildGpuTimestampHistoryStats({frame}, 1);
  EXPECT_DOUBLE_EQ(history.summed_work.selected_milliseconds, 2.5);
}

TEST(RenderGraph, RegisteredGpuProfilerItemsMapToTimestampMetadata) {
  RegisteredProfilerItem item;
  item.owner_name = "EcoSysLab";
  item.stable_id = "EcoSysLab.DynamicStrands.Physics";
  item.descriptor.display_name = "Physics";
  item.descriptor.gpu_group = "DynamicStrands";
  item.descriptor.gpu_queue = ProfilerGpuQueue::Compute;
  item.descriptor.gpu_contributes_to_frame_total = true;

  const auto metadata = MakeGpuTimestampScopeMetadata(item);
  EXPECT_EQ(metadata.stable_pass_id, item.stable_id);
  EXPECT_EQ(metadata.display_name, item.descriptor.display_name);
  EXPECT_EQ(metadata.group, item.descriptor.gpu_group);
  EXPECT_EQ(metadata.queue, GpuTimestampQueue::Compute);
  EXPECT_TRUE(metadata.contributes_to_frame_total);
  EXPECT_EQ(metadata.owner_name, item.owner_name);
}

TEST(RenderGraph, RenderPassDescriptorsExposeExplicitGpuProfilerTaxonomy) {
  const std::vector<RenderPassDescriptor> descriptors = {
      DdgiAtlasPreparePass::CreateDescriptor(),
      DdgiProbeClassificationPass::CreateDescriptor(),
      DdgiProbeRayVisualizationPass::CreateDescriptor(nullptr),
      DdgiProbeRelocationPass::CreateDescriptor(),
      DdgiProbeScrollPass::CreateDescriptor(),
      DdgiProbeUpdatePass::CreateDescriptor(),
      DdgiProbeUpdatePass::CreateHistoryInvalidationDescriptor(true, true),
      DdgiProbeVisualizationPass::CreateDescriptor(),
      DdgiProbeTracePass::CreateDescriptor(),
      DeferredGeometryPass::CreateDescriptor(),
      DeferredComputeLightingPass::CreateDescriptor(false, false),
      DepthPyramidPass::CreateDescriptor(),
      DirectionalLightShadowPass::CreateDescriptor(),
      EntitySelectionHighlightPass::CreateDescriptor(),
      GaussianSplatCullPass::CreateDescriptor(nullptr),
      GaussianSplatSortPass::CreateDescriptor(nullptr),
      GaussianSplatPass::CreateDescriptor(nullptr),
      MotionCoveragePass::CreateDescriptor(),
      MotionVectorPass::CreateDescriptor(),
      AmbientOcclusionPass::CreateDescriptor(),
      PostProcessingPass::CreateDescriptor(nullptr),
      RayTracingCameraPass::CreateDescriptor(),
      RayQueryCameraPass::CreateDescriptor(),
      TransparentGeometryPass::CreateDescriptor(),
      VolumetricCloudsPass::CreateRasterDescriptor(nullptr),
  };

  for (const auto& descriptor : descriptors) {
    SCOPED_TRACE(descriptor.name);
    EXPECT_FALSE(descriptor.name.empty());
    EXPECT_FALSE(descriptor.profiler_display_name.empty());
    EXPECT_NE(descriptor.profiler_group, RenderPassProfilerGroup::Other);
  }
  EXPECT_EQ(DirectionalLightShadowPass::CreateDescriptor().profiler_group, RenderPassProfilerGroup::Shadows);
  EXPECT_EQ(PostProcessingPass::CreateDescriptor(nullptr).profiler_group, RenderPassProfilerGroup::PostProcessing);

  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  EXPECT_NE(render_layer.find("{\"PointShadow\", \"Point Shadow\", \"Shadows\""), std::string::npos);
  EXPECT_NE(render_layer.find("{\"SpotShadow\", \"Spot Shadow\", \"Shadows\""), std::string::npos);
  EXPECT_NE(render_layer.find("ReflectionProbeGgxPrefilter"), std::string::npos);
  EXPECT_NE(render_layer.find("DynamicReflectionProbeGgxPrefilter"), std::string::npos);
  EXPECT_NE(render_layer.find("ReflectionProbeBakeTotal"), std::string::npos);
  EXPECT_NE(render_layer.find("DynamicReflectionProbeUpdateTotal"), std::string::npos);
  EXPECT_NE(render_layer.find("GpuTimestampQueue::Graphics, 0, 0, false"), std::string::npos);
}

TEST(RenderGraph, CameraFrustumBoundsAreConservative) {
  const glm::mat4 projection_view = glm::perspective(glm::radians(45.0f), 16.0f / 9.0f, 0.1f, 100.0f);
  const auto intersects = [&](const glm::vec3& min, const glm::vec3& max) {
    Bound bound;
    bound.min = min;
    bound.max = max;
    return RenderInstanceStorage::BoundIntersectsCameraClipSpace(bound, projection_view);
  };

  EXPECT_TRUE(intersects({-1.0f, -1.0f, -6.0f}, {1.0f, 1.0f, -4.0f}));
  EXPECT_TRUE(intersects({-0.1f, -0.1f, -0.2f}, {0.1f, 0.1f, -0.05f}));
  EXPECT_FALSE(intersects({100.0f, -1.0f, -6.0f}, {102.0f, 1.0f, -4.0f}));
  EXPECT_FALSE(intersects({-1.0f, -1.0f, 4.0f}, {1.0f, 1.0f, 6.0f}));
  EXPECT_FALSE(intersects({-1.0f, -1.0f, -202.0f}, {1.0f, 1.0f, -200.0f}));

  Bound invalid_bound;
  EXPECT_FALSE(RenderInstanceStorage::IsFiniteBound(invalid_bound));
  EXPECT_TRUE(RenderInstanceStorage::BoundIntersectsCameraClipSpace(invalid_bound, projection_view));
}

TEST(RenderGraph, MeshletSphereFrustumCullingHandlesTransformsConservatively) {
  const glm::mat4 projection_view = glm::perspective(glm::radians(45.0f), 16.0f / 9.0f, 0.1f, 100.0f);
  const glm::vec4 sphere{0.0f, 0.0f, 0.0f, 1.0f};

  glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -5.0f));
  model = glm::scale(model, glm::vec3(-2.0f, 0.5f, 1.0f));
  EXPECT_TRUE(RenderInstanceStorage::MeshletSphereIntersectsClipSpace(sphere, model, projection_view, true));

  model = glm::translate(glm::mat4(1.0f), glm::vec3(100.0f, 0.0f, -5.0f));
  model = glm::scale(model, glm::vec3(-2.0f, 0.5f, 1.0f));
  EXPECT_FALSE(RenderInstanceStorage::MeshletSphereIntersectsClipSpace(sphere, model, projection_view, true));

  model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -0.05f));
  EXPECT_TRUE(RenderInstanceStorage::MeshletSphereIntersectsClipSpace(sphere, model, projection_view, true));
  EXPECT_TRUE(
      RenderInstanceStorage::MeshletSphereIntersectsClipSpace({0.0f, 0.0f, 0.0f, -1.0f}, model, projection_view, true));
}

TEST(RenderGraph, MeshletConeCullingIsConservativelyGated) {
  const auto culling =
      ReadTextFile(SdkPath("Internals/DefaultResources/Shaders/Modules/EvoEngine/MeshletCulling.slang"));
  const auto task = ReadTextFile(SdkPath("Internals/DefaultResources/Shaders/Graphics/Task/Standard/Standard.slang"));

  EXPECT_NE(culling.find("determinant(linear) > 0.0f"), std::string::npos);
  EXPECT_NE(culling.find("maximum_length <= minimum_length * 1.0001f"), std::string::npos);
  EXPECT_NE(culling.find("EE_MESHLET_CULL_CONE_FRONT"), std::string::npos);
  EXPECT_NE(task.find("EE_MESHLET_CONE_VISIBLE"), std::string::npos);
}

TEST(RenderGraph, ShadowFrustumBoundsAreConservative) {
  Bound visible_bound;
  visible_bound.min = {-0.5f, -0.5f, -0.5f};
  visible_bound.max = {0.5f, 0.5f, 0.5f};
  Bound outside_bound;
  outside_bound.min = {2.0f, -0.5f, -0.5f};
  outside_bound.max = {3.0f, 0.5f, 0.5f};
  Bound invalid_bound;
  EXPECT_TRUE(RenderInstanceStorage::BoundIntersectsShadowClipSpace(visible_bound, glm::mat4(1.0f)));
  EXPECT_FALSE(RenderInstanceStorage::BoundIntersectsShadowClipSpace(outside_bound, glm::mat4(1.0f)));
  EXPECT_TRUE(RenderInstanceStorage::BoundIntersectsShadowClipSpace(invalid_bound, glm::mat4(1.0f)));
}

TEST(RenderGraph, PackedShadowIndirectUsesOneStableArenaAndViewOffsets) {
  const auto storage = ReadTextFile(SdkPath("src/RenderInstanceStorage.cpp"));
  const auto header = ReadTextFile(SdkPath("include/Rendering/RenderInstances/RenderInstanceStorage.hpp"));
  EXPECT_NE(header.find("VkDeviceSize indirect_buffer_offset = 0"), std::string::npos);
  EXPECT_NE(storage.find("FinalizeShadowIndirectBuffers(use_mesh_shader)"), std::string::npos);
  EXPECT_NE(storage.find("visibility.indirect_buffer_offset = VectorBytes(packed_shadow"), std::string::npos);
  EXPECT_NE(storage.find("add_vector_if_changed(packed_shadow_indirect_buffer"), std::string::npos);
  EXPECT_NE(storage.find("BufferUploadBatch upload_batch"), std::string::npos);
}

TEST(RenderGraph, PackedCameraIndirectUsesAlignedArenaRanges) {
  const auto storage = ReadTextFile(SdkPath("src/RenderInstanceStorage.cpp"));
  const auto geometry = ReadTextFile(SdkPath("src/RenderPasses/DeferredGeometryPass.cpp"));
  EXPECT_NE(storage.find("StorageBufferAlignment()"), std::string::npos);
  EXPECT_NE(storage.find("AppendAligned(packed_camera_indexed_commands"), std::string::npos);
  EXPECT_NE(geometry.find("indexed_indirect_buffer_offset +"), std::string::npos);
}

TEST(RenderGraph, PersistentInstanceStatePlansOnlyDirtyGpuRanges) {
  using Storage = RenderInstanceStorage;
  std::vector<Storage::InstanceInfoBlock> previous_instances(5);
  std::vector<Storage::InstanceInfoBlock> current_instances = previous_instances;
  current_instances[1].model.value[3].x = 1.0f;
  current_instances[2].world_bound_max.x = 2.0f;
  current_instances[4].material_index = 3;
  const auto instance_ranges = Storage::PlanInstanceInfoUploadRanges(previous_instances, current_instances);
  ASSERT_EQ(instance_ranges.size(), 2u);
  EXPECT_EQ(instance_ranges[0].first_instance, 1u);
  EXPECT_EQ(instance_ranges[0].instance_count, 2u);
  EXPECT_EQ(instance_ranges[1].first_instance, 4u);
  EXPECT_EQ(instance_ranges[1].instance_count, 1u);
  EXPECT_TRUE(Storage::PlanInstanceInfoUploadRanges(current_instances, current_instances).empty());

  auto selection_instances = previous_instances;
  selection_instances[3].info_index = 1;
  const auto selection_ranges = Storage::PlanInstanceInfoUploadRanges(previous_instances, selection_instances);
  ASSERT_EQ(selection_ranges.size(), 1u);
  EXPECT_EQ(selection_ranges[0].first_instance, 3u);
  EXPECT_EQ(selection_ranges[0].instance_count, 1u);

  std::vector<Storage::PreviousInstanceInfoBlock> previous_motion(4);
  std::vector<Storage::PreviousInstanceInfoBlock> current_motion = previous_motion;
  current_motion[2].previous_model[3].z = 5.0f;
  current_motion[3].flags.x = 1u;
  const auto motion_ranges = Storage::PlanPreviousInstanceInfoUploadRanges(previous_motion, current_motion);
  ASSERT_EQ(motion_ranges.size(), 1u);
  EXPECT_EQ(motion_ranges[0].first_instance, 2u);
  EXPECT_EQ(motion_ranges[0].instance_count, 2u);
}

TEST(RenderGraph, RasterSpatialIndexMatchesLinearQueriesAndTracksLifecycle) {
  using SpatialIndex = RenderInstanceStorage::RasterSpatialIndex;
  SpatialIndex index;
  std::vector<std::pair<Handle, Bound>> leaves;
  for (uint32_t i = 0; i < 64u; ++i) {
    const glm::vec3 center{static_cast<float>((i * 17u) % 23u) - 11.0f, static_cast<float>((i * 11u) % 19u) - 9.0f,
                           static_cast<float>((i * 7u) % 29u) - 14.0f};
    const glm::vec3 extent{0.2f + static_cast<float>(i % 5u) * 0.15f};
    leaves.emplace_back(Handle(i + 1u), Bound{center - extent, center + extent});
  }
  index.BeginUpdate();
  for (const auto& [handle, bound] : leaves) {
    index.Upsert(handle, bound);
  }
  index.EndUpdate();
  EXPECT_EQ(index.GetUpdateStats().leaf_count, leaves.size());
  EXPECT_EQ(index.GetUpdateStats().inserted_leaves, leaves.size());

  const auto overlaps = [](const Bound& left, const Bound& right) {
    return glm::all(glm::lessThanEqual(left.min, right.max)) && glm::all(glm::greaterThanEqual(left.max, right.min));
  };
  for (uint32_t i = 0; i < 32u; ++i) {
    const glm::vec3 center{static_cast<float>((i * 13u) % 31u) - 15.0f, static_cast<float>((i * 5u) % 17u) - 8.0f,
                           static_cast<float>((i * 19u) % 37u) - 18.0f};
    const Bound query{center - glm::vec3(3.0f), center + glm::vec3(3.0f)};
    std::vector<Handle> linear;
    for (const auto& [handle, bound] : leaves) {
      if (overlaps(bound, query)) {
        linear.emplace_back(handle);
      }
    }
    auto spatial = index.Query([&](const Bound& bound) {
      return overlaps(bound, query);
    });
    std::sort(linear.begin(), linear.end(), [](const auto left, const auto right) {
      return static_cast<uint64_t>(left) < static_cast<uint64_t>(right);
    });
    std::sort(spatial.begin(), spatial.end(), [](const auto left, const auto right) {
      return static_cast<uint64_t>(left) < static_cast<uint64_t>(right);
    });
    EXPECT_EQ(spatial, linear);
  }

  index.BeginUpdate();
  for (size_t i = 0; i + 1u < leaves.size(); ++i) {
    auto bound = leaves[i].second;
    if (i == 0u) {
      bound.min += glm::vec3(100.0f);
      bound.max += glm::vec3(100.0f);
    }
    index.Upsert(leaves[i].first, bound);
  }
  index.EndUpdate();
  EXPECT_EQ(index.GetUpdateStats().reinserted_leaves, 1u);
  EXPECT_EQ(index.GetUpdateStats().removed_leaves, 1u);
  EXPECT_EQ(index.GetUpdateStats().unchanged_leaves, leaves.size() - 2u);
  EXPECT_EQ(index.GetUpdateStats().leaf_count, leaves.size() - 1u);
}

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
      [&](const RenderGraphExecutionContext&) {
        EXPECT_EQ(sequence++, 0);
      });
  graph.AddPass(
      {"Lighting",
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
        {RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
      [&](const RenderGraphExecutionContext&) {
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

TEST(RenderGraph, ValidateRejectsInvalidGraphDescriptors) {
  {
    SCOPED_TRACE("unknown resource");
    RenderGraph graph;
    graph.AddPass(
        {"Lighting",
         RenderPassQueue::Graphics,
         RenderPassScope::Camera,
         {{RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
        [](const RenderGraphExecutionContext&) {
        });

    EXPECT_FALSE(graph.Validate());
  }

  {
    SCOPED_TRACE("unknown or forward dependency");
    RenderGraph graph;
    graph.AddResource({RenderResourceNames::camera_color, RenderResourceType::Image, RenderResourceLifetime::Camera});
    graph.AddPass(
        {"Lighting",
         RenderPassQueue::Graphics,
         RenderPassScope::Camera,
         {{RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}},
         {"DepthPrepass"}},
        [](const RenderGraphExecutionContext&) {
        });

    EXPECT_FALSE(graph.Validate());

    graph.AddPass({"DepthPrepass",
                   RenderPassQueue::Graphics,
                   RenderPassScope::Camera,
                   {{RenderResourceNames::camera_color, RenderResourceUsage::Read, RenderResourceState::ShaderRead}}},
                  [](const RenderGraphExecutionContext&) {
                  });

    EXPECT_FALSE(graph.Validate());
  }

  {
    SCOPED_TRACE("duplicate pass names");
    RenderGraph graph;
    graph.AddResource({RenderResourceNames::camera_color, RenderResourceType::Image, RenderResourceLifetime::Camera});
    graph.AddPass({"Copy",
                   RenderPassQueue::Graphics,
                   RenderPassScope::Camera,
                   {{RenderResourceNames::camera_color, RenderResourceUsage::Read, RenderResourceState::ShaderRead}}},
                  [](const RenderGraphExecutionContext&) {
                  });
    graph.AddPass(
        {"Copy",
         RenderPassQueue::Graphics,
         RenderPassScope::Camera,
         {{RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
        [](const RenderGraphExecutionContext&) {
        });

    EXPECT_FALSE(graph.Validate());
  }
}

TEST(RenderGraph, DdgiIncrementalScrollDoesNotRequireAtlasClearPass) {
  EXPECT_TRUE(DdgiProbeScrollPass::CreateDescriptor().dependencies.empty());
}

TEST(RenderGraph, ClearRemovesResourcesAndPasses) {
  RenderGraph graph;
  graph.AddResource(
      {RenderResourceNames::frame_render_instances, RenderResourceType::Buffer, RenderResourceLifetime::Frame});
  graph.AddPass({"Prepare", RenderPassQueue::Compute, RenderPassScope::Frame, {}},
                [](const RenderGraphExecutionContext&) {
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
      [](const RenderGraphExecutionContext&) {
      });
  graph.AddPass(
      {"Denoise",
       RenderPassQueue::Compute,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::StorageReadWrite}},
       {"Raster"}},
      [](const RenderGraphExecutionContext&) {
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
      [](const RenderGraphExecutionContext&) {
      });
  graph.AddPass({"DebugOverlay",
                 RenderPassQueue::Compute,
                 RenderPassScope::Frame,
                 {{kTestDebugOutputResource, RenderResourceUsage::Write, RenderResourceState::General}}},
                [](const RenderGraphExecutionContext&) {
                });
  graph.AddPass({"BuildDepthPyramid",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
                  {RenderResourceNames::camera_depth_pyramid, RenderResourceUsage::Write,
                   RenderResourceState::StorageReadWrite}}},
                [](const RenderGraphExecutionContext&) {
                });
  graph.AddPass(
      {"Lighting",
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
        {RenderResourceNames::camera_depth_pyramid, RenderResourceUsage::Read, RenderResourceState::ShaderRead}}},
      [](const RenderGraphExecutionContext&) {
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
      [&](const RenderGraphExecutionContext&) {
        execution_order.emplace_back(0);
      });
  graph.AddPass({"DepthPyramid",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead}}},
                [&](const RenderGraphExecutionContext&) {
                  execution_order.emplace_back(1);
                });
  graph.AddPass({"Debug",
                 RenderPassQueue::Compute,
                 RenderPassScope::Frame,
                 {{kTestDebugOutputResource, RenderResourceUsage::Write, RenderResourceState::General}}},
                [&](const RenderGraphExecutionContext&) {
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
                [](const RenderGraphExecutionContext&) {
                });
  graph.AddPass(
      {"TemporalResolve",
       RenderPassQueue::Compute,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
        {temporary_radiance, RenderResourceUsage::ReadWrite, RenderResourceState::StorageReadWrite},
        {RenderResourceNames::camera_color_history, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}},
       {"Raster"}},
      [](const RenderGraphExecutionContext&) {
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
                [](const RenderGraphExecutionContext&) {
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
                [](const RenderGraphExecutionContext&) {
                });
  graph.AddPass({"BloomUpsample",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{bloom_b, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}},
                 {"BloomDownsample"}},
                [](const RenderGraphExecutionContext&) {
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
                [](const RenderGraphExecutionContext&) {
                });
  graph.AddPass({"WritePongReadPing",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{ping, RenderResourceUsage::Read, RenderResourceState::StorageReadWrite},
                  {pong, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}},
                 {"WritePing"}},
                [](const RenderGraphExecutionContext&) {
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
      [](const RenderGraphExecutionContext&) {
      });
  graph.AddPass(
      {"Denoise",
       RenderPassQueue::Compute,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::StorageReadWrite}},
       {"Raster"}},
      [](const RenderGraphExecutionContext&) {
      });
  graph.AddPass({"Sample",
                 RenderPassQueue::Graphics,
                 RenderPassScope::Camera,
                 {{RenderResourceNames::camera_color, RenderResourceUsage::Read, RenderResourceState::ShaderRead}},
                 {"Denoise"}},
                [](const RenderGraphExecutionContext&) {
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

TEST(RenderGraph, RenderPassUtilitiesApplyQueueFamilyOwnershipTransfersForQueueChangingBarriers) {
  const auto utilities_path = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" /
                              "RenderPasses" / "RenderPassUtilities.cpp";
  std::ifstream utilities_file(utilities_path);
  ASSERT_TRUE(utilities_file.good()) << utilities_path.string();
  const std::string utilities_source((std::istreambuf_iterator<char>(utilities_file)),
                                     std::istreambuf_iterator<char>());

  EXPECT_NE(utilities_source.find("uint32_t GetRenderPassQueueFamilyIndex(const RenderPassQueue queue)"),
            std::string::npos);
  EXPECT_NE(utilities_source.find("TryGetQueueFamilyOwnershipTransfer"), std::string::npos);
  EXPECT_NE(utilities_source.find("const auto expected_queue = release_barrier ? barrier.previous_queue : "
                                  "barrier.next_queue"),
            std::string::npos);
  EXPECT_NE(utilities_source.find("src_queue_family_index = GetRenderPassQueueFamilyIndex(barrier.previous_queue)"),
            std::string::npos);
  EXPECT_NE(utilities_source.find("dst_queue_family_index = GetRenderPassQueueFamilyIndex(barrier.next_queue)"),
            std::string::npos);
  EXPECT_NE(utilities_source.find("return src_queue_family_index != dst_queue_family_index"), std::string::npos);
  EXPECT_NE(utilities_source.find("ApplyGraphImageQueueOwnershipBarrier"), std::string::npos);
  EXPECT_NE(utilities_source.find("image->TransitImageLayout(vk_command_buffer, previous_layout, next_layout, "
                                  "src_queue_family_index"),
            std::string::npos);
  EXPECT_NE(utilities_source.find("ApplyGraphBufferQueueOwnershipBarrier"), std::string::npos);
  EXPECT_NE(utilities_source.find("Platform::BufferMemoryBarrier(vk_command_buffer, *binding->buffer, "
                                  "src_queue_family_index, dst_queue_family_index"),
            std::string::npos);
  EXPECT_NE(utilities_source.find("ApplyGraphResourceReleaseBarriers"), std::string::npos);
  EXPECT_NE(utilities_source.find("context.GetCurrentPassReleaseBarriers()"), std::string::npos);
}

TEST(RenderGraph, CompilePlansDdgiAtlasPrepareResources) {
  DdgiSettings settings;
  uint32_t probe_count = 64u;
  settings.storage.atlas_probe_columns = 8;
  settings.storage.irradiance_tile_resolution = 8;
  settings.storage.visibility_tile_resolution = 16;
  settings.runtime.ray_count = 32;
  const auto layout = DdgiRuntime::CalculateFrameResourceLayout(settings, probe_count);

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
  graph.AddPass(DdgiAtlasPreparePass::CreateDescriptor(), [](const RenderGraphExecutionContext&) {
  });

  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.uses_graphics_queue);
  EXPECT_TRUE(std::all_of(plan.resources.begin(), plan.resources.end(), [](const auto& resource) {
    return resource.imported;
  }));
  EXPECT_TRUE(plan.allocations.empty());
  ASSERT_EQ(plan.barriers.size(), 2);
  EXPECT_NE(std::find_if(plan.barriers.begin(), plan.barriers.end(),
                         [&](const RenderResourceBarrierPlan& barrier) {
                           return graph.GetResources()[barrier.resource_index].name ==
                                      RenderResourceNames::frame_ddgi_irradiance_atlas &&
                                  barrier.barrier_type == RenderGraphBarrierType::ImageLayout &&
                                  barrier.next_state == RenderResourceState::TransferDestinationGeneral;
                         }),
            plan.barriers.end());
  EXPECT_NE(std::find_if(plan.barriers.begin(), plan.barriers.end(),
                         [&](const RenderResourceBarrierPlan& barrier) {
                           return graph.GetResources()[barrier.resource_index].name ==
                                      RenderResourceNames::frame_ddgi_visibility_atlas &&
                                  barrier.barrier_type == RenderGraphBarrierType::ImageLayout &&
                                  barrier.next_state == RenderResourceState::TransferDestinationGeneral;
                         }),
            plan.barriers.end());
}

TEST(RenderGraph, CompilePlansDdgiProbeTraceWithoutRayClear) {
  DdgiSettings settings;
  uint32_t probe_count = 8u;
  settings.runtime.ray_count = 16;
  const auto layout = DdgiRuntime::CalculateFrameResourceLayout(settings, probe_count);

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
  graph.AddResource({RenderResourceNames::frame_ddgi_selected_ray_diagnostics,
                     RenderResourceType::Buffer,
                     RenderResourceLifetime::Persistent,
                     {},
                     {},
                     1,
                     1,
                     false,
                     layout.selected_ray_diagnostics_byte_size});
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
  graph.AddPass(DdgiProbeTracePass::CreateDescriptor(), [](const RenderGraphExecutionContext&) {
  });

  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.uses_ray_tracing_queue);
  ASSERT_EQ(plan.passes.size(), 1);
  EXPECT_TRUE(plan.passes[0].resource_dependency_indices.empty());
  ASSERT_EQ(plan.allocations.size(), 1);
  EXPECT_EQ(plan.allocations[0].byte_size, layout.ray_output_byte_size);
  const auto diagnostics_resource = std::find_if(
      graph.GetResources().begin(), graph.GetResources().end(), [](const RenderResourceDescriptor& resource) {
        return resource.name == RenderResourceNames::frame_ddgi_selected_ray_diagnostics;
      });
  ASSERT_NE(diagnostics_resource, graph.GetResources().end());
  EXPECT_TRUE(
      plan.resources[static_cast<size_t>(std::distance(graph.GetResources().begin(), diagnostics_resource))].imported);
  const auto ray_output_resource = std::find_if(graph.GetResources().begin(), graph.GetResources().end(),
                                                [](const RenderResourceDescriptor& resource) {
                                                  return resource.name == RenderResourceNames::frame_ddgi_ray_output;
                                                });
  ASSERT_NE(ray_output_resource, graph.GetResources().end());
  const auto ray_output_index = static_cast<size_t>(std::distance(graph.GetResources().begin(), ray_output_resource));
  EXPECT_EQ(plan.resources[ray_output_index].writer_pass_indices, std::vector<size_t>({0}));
  EXPECT_TRUE(plan.resources[ray_output_index].reader_pass_indices.empty());
  EXPECT_NE(std::find_if(plan.barriers.begin(), plan.barriers.end(),
                         [&](const RenderResourceBarrierPlan& barrier) {
                           return graph.GetResources()[barrier.resource_index].name ==
                                      RenderResourceNames::frame_ddgi_irradiance_atlas &&
                                  barrier.barrier_type == RenderGraphBarrierType::ImageLayout &&
                                  barrier.next_state == RenderResourceState::General;
                         }),
            plan.barriers.end());
  EXPECT_NE(std::find_if(plan.barriers.begin(), plan.barriers.end(),
                         [&](const RenderResourceBarrierPlan& barrier) {
                           return graph.GetResources()[barrier.resource_index].name ==
                                      RenderResourceNames::frame_ddgi_visibility_atlas &&
                                  barrier.barrier_type == RenderGraphBarrierType::ImageLayout &&
                                  barrier.next_state == RenderResourceState::General;
                         }),
            plan.barriers.end());
}

TEST(RenderGraph, CompilePlansDdgiProbeUpdateAfterProbeTrace) {
  DdgiSettings settings;
  uint32_t probe_count = 8u;
  settings.runtime.ray_count = 16;
  const auto layout = DdgiRuntime::CalculateFrameResourceLayout(settings, probe_count);

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
  graph.AddResource({RenderResourceNames::frame_ddgi_selected_ray_diagnostics,
                     RenderResourceType::Buffer,
                     RenderResourceLifetime::Persistent,
                     {},
                     {},
                     1,
                     1,
                     false,
                     layout.selected_ray_diagnostics_byte_size});
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
  graph.AddPass(DdgiProbeTracePass::CreateDescriptor(), [](const RenderGraphExecutionContext&) {
  });
  for (size_t i = 0; i < layout.history.buffer_bytes.size(); ++i) {
    RenderResourceDescriptor resource{RenderResourceNames::frame_ddgi_history[i], RenderResourceType::Buffer,
                                      RenderResourceLifetime::Imported};
    resource.byte_size = layout.history.buffer_bytes[i];
    graph.AddResource(resource);
  }
  graph.AddPass(DdgiProbeUpdatePass::CreateDescriptor(), [](const RenderGraphExecutionContext&) {
  });
  graph.AddPass(DdgiProbeRelocationPass::CreateDescriptor(), [](const RenderGraphExecutionContext&) {
  });
  graph.AddPass(DdgiProbeClassificationPass::CreateDescriptor(), [](const RenderGraphExecutionContext&) {
  });
  graph.AddPass(DdgiProbeUpdatePass::CreateHistoryInvalidationDescriptor(true, true),
                [](const RenderGraphExecutionContext&) {
                });

  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.uses_graphics_queue);
  EXPECT_TRUE(plan.uses_ray_tracing_queue);
  ASSERT_EQ(plan.passes.size(), 5);
  EXPECT_NE(std::find(plan.passes[1].dependency_indices.begin(), plan.passes[1].dependency_indices.end(), 0),
            plan.passes[1].dependency_indices.end());
  EXPECT_NE(std::find(plan.passes[1].resource_dependency_indices.begin(),
                      plan.passes[1].resource_dependency_indices.end(), 0),
            plan.passes[1].resource_dependency_indices.end());
  EXPECT_NE(std::find(plan.passes[2].dependency_indices.begin(), plan.passes[2].dependency_indices.end(), 1),
            plan.passes[2].dependency_indices.end());
  EXPECT_NE(std::find(plan.passes[2].resource_dependency_indices.begin(),
                      plan.passes[2].resource_dependency_indices.end(), 1),
            plan.passes[2].resource_dependency_indices.end());
  EXPECT_NE(std::find(plan.passes[3].dependency_indices.begin(), plan.passes[3].dependency_indices.end(), 1),
            plan.passes[3].dependency_indices.end());
  EXPECT_NE(std::find(plan.passes[3].resource_dependency_indices.begin(),
                      plan.passes[3].resource_dependency_indices.end(), 2),
            plan.passes[3].resource_dependency_indices.end());
  EXPECT_NE(std::find(plan.passes[4].dependency_indices.begin(), plan.passes[4].dependency_indices.end(), 1),
            plan.passes[4].dependency_indices.end());
  EXPECT_NE(std::find(plan.passes[4].resource_dependency_indices.begin(),
                      plan.passes[4].resource_dependency_indices.end(), 3),
            plan.passes[4].resource_dependency_indices.end());
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
  auto validate_layout = [](const DdgiFrameResourceLayout& layout) {
    RenderGraph graph;
    auto add_buffer = [&](const char* name, const RenderResourceLifetime lifetime, const bool managed_by_graph,
                          const uint64_t byte_size) {
      graph.AddResource({name, RenderResourceType::Buffer, lifetime, {}, {}, 1, 1, managed_by_graph, byte_size});
    };
    auto add_atlas = [&](const char* name, const DdgiAtlasLayout& atlas_layout, const char* format_name) {
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
    add_atlas(RenderResourceNames::frame_ddgi_irradiance_atlas, layout.irradiance_atlas, "RGBA16F");
    add_atlas(RenderResourceNames::frame_ddgi_visibility_atlas, layout.visibility_atlas, "RG16F");
    graph.AddPass(DdgiAtlasPreparePass::CreateDescriptor(), [](const RenderGraphExecutionContext&) {
    });
    graph.AddPass(
        {"DDGIAtlasPreviewRead",
         RenderPassQueue::Compute,
         RenderPassScope::Frame,
         {{RenderResourceNames::frame_ddgi_probe_metadata, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
          {RenderResourceNames::frame_ddgi_irradiance_atlas, RenderResourceUsage::Read,
           RenderResourceState::ShaderRead},
          {RenderResourceNames::frame_ddgi_visibility_atlas, RenderResourceUsage::Read,
           RenderResourceState::ShaderRead}},
         {RenderPassNames::ddgi_atlas_prepare}},
        [](const RenderGraphExecutionContext&) {
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
    const auto irradiance_index = find_resource_index(RenderResourceNames::frame_ddgi_irradiance_atlas);
    const auto visibility_index = find_resource_index(RenderResourceNames::frame_ddgi_visibility_atlas);
    ASSERT_NE(metadata_index, RenderGraphConstants::invalid_resource_index);
    ASSERT_NE(state_index, RenderGraphConstants::invalid_resource_index);
    ASSERT_NE(irradiance_index, RenderGraphConstants::invalid_resource_index);
    ASSERT_NE(visibility_index, RenderGraphConstants::invalid_resource_index);

    EXPECT_TRUE(plan.resources[metadata_index].imported);
    EXPECT_TRUE(plan.resources[state_index].imported);
    EXPECT_TRUE(plan.resources[irradiance_index].imported);
    EXPECT_TRUE(plan.resources[visibility_index].imported);
    EXPECT_EQ(graph.GetResources()[metadata_index].byte_size, layout.probe_metadata_byte_size);
    EXPECT_EQ(graph.GetResources()[state_index].byte_size, layout.probe_state_byte_size);
    EXPECT_TRUE(plan.allocations.empty());
    EXPECT_EQ(plan.resources[irradiance_index].resolved_dimensions.width, layout.irradiance_atlas.resolution.x);
    EXPECT_EQ(plan.resources[irradiance_index].resolved_dimensions.height, layout.irradiance_atlas.resolution.y);
    EXPECT_EQ(plan.resources[visibility_index].resolved_dimensions.width, layout.visibility_atlas.resolution.x);
    EXPECT_EQ(plan.resources[visibility_index].resolved_dimensions.height, layout.visibility_atlas.resolution.y);

    auto has_barrier = [&](const size_t resource_index, const RenderGraphBarrierType type,
                           const RenderResourceState next_state) {
      return std::find_if(plan.barriers.begin(), plan.barriers.end(), [&](const auto& barrier) {
               return barrier.resource_index == resource_index && barrier.barrier_type == type &&
                      barrier.next_state == next_state;
             }) != plan.barriers.end();
    };
    EXPECT_TRUE(has_barrier(metadata_index, RenderGraphBarrierType::BufferMemory, RenderResourceState::ShaderRead));
    EXPECT_TRUE(has_barrier(irradiance_index, RenderGraphBarrierType::ImageLayout,
                            RenderResourceState::TransferDestinationGeneral));
    EXPECT_TRUE(has_barrier(irradiance_index, RenderGraphBarrierType::ImageLayout, RenderResourceState::ShaderRead));
    EXPECT_TRUE(has_barrier(visibility_index, RenderGraphBarrierType::ImageLayout,
                            RenderResourceState::TransferDestinationGeneral));
    EXPECT_TRUE(has_barrier(visibility_index, RenderGraphBarrierType::ImageLayout, RenderResourceState::ShaderRead));
    EXPECT_NE(std::find_if(plan.barriers.begin(), plan.barriers.end(),
                           [&](const auto& barrier) {
                             return barrier.resource_index == irradiance_index && barrier.memory_dependency &&
                                    barrier.previous_state == RenderResourceState::TransferDestinationGeneral &&
                                    barrier.next_state == RenderResourceState::ShaderRead;
                           }),
              plan.barriers.end());
  };

  DdgiSettings settings;
  uint32_t probe_count = 8u;
  settings.storage.atlas_probe_columns = 4;
  settings.storage.irradiance_tile_resolution = 6;
  settings.storage.visibility_tile_resolution = 10;
  settings.runtime.ray_count = 12;
  const auto initial_layout = DdgiRuntime::CalculateFrameResourceLayout(settings, probe_count);
  validate_layout(initial_layout);

  probe_count = 30u;
  settings.storage.atlas_probe_columns = 5;
  settings.storage.irradiance_tile_resolution = 10;
  settings.storage.visibility_tile_resolution = 18;
  settings.runtime.ray_count = 48;
  const auto resized_layout = DdgiRuntime::CalculateFrameResourceLayout(settings, probe_count);
  ASSERT_NE(resized_layout.irradiance_atlas.resolution, initial_layout.irradiance_atlas.resolution);
  ASSERT_NE(resized_layout.visibility_atlas.resolution, initial_layout.visibility_atlas.resolution);
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
                [](const RenderGraphExecutionContext&) {
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
                [](const RenderGraphExecutionContext&) {
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
      [](const RenderGraphExecutionContext&) {
      });
  graph.AddPass(
      {RenderPassNames::post_processing,
       RenderPassQueue::Compute,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::StorageReadWrite}},
       {RenderPassNames::deferred_camera}},
      [](const RenderGraphExecutionContext&) {
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
      [](const RenderGraphExecutionContext&) {
      });
  graph.AddPass(
      {RenderPassNames::depth_pyramid,
       RenderPassQueue::Compute,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
        {RenderResourceNames::camera_depth_pyramid, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}},
       {RenderPassNames::deferred_geometry}},
      [](const RenderGraphExecutionContext&) {
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
      [](const RenderGraphExecutionContext&) {
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

TEST(RenderGraph, AmbientOcclusionPrecedesFusedDeferredCompute) {
  RenderGraph graph;
  AddDefaultRasterCameraResources(graph);
  AddAdvancedCameraResources(graph);
  graph.AddPass(
      {RenderPassNames::deferred_geometry,
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_depth, RenderResourceUsage::Write, RenderResourceState::DepthAttachment},
        {RenderResourceNames::camera_g_buffer, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
      [](const RenderGraphExecutionContext&) {
      });
  graph.AddPass(
      {RenderPassNames::depth_pyramid,
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
        {RenderResourceNames::camera_depth_pyramid, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}},
       {RenderPassNames::deferred_geometry}},
      [](const RenderGraphExecutionContext&) {
      });
  graph.AddPass(AmbientOcclusionPass::CreateDescriptor(), [](const RenderGraphExecutionContext&) {
  });
  graph.AddPass(DeferredComputeLightingPass::CreateDescriptor(true, true), [](const RenderGraphExecutionContext&) {
  });

  RenderGraphCompileContext context;
  context.camera_width = 640;
  context.camera_height = 360;
  const auto plan = graph.Compile(context);
  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(plan.passes.size(), 4);
  EXPECT_TRUE(plan.passes[2].dependency_indices.empty());
  EXPECT_EQ(plan.passes[2].resource_dependency_indices, std::vector<size_t>({0}));
  EXPECT_EQ(plan.passes[3].dependency_indices, std::vector<size_t>({2}));
  EXPECT_EQ(graph.GetPasses()[3].name, RenderPassNames::deferred_camera);

  const auto transition = std::find_if(plan.transitions.begin(), plan.transitions.end(), [](const auto& candidate) {
    return candidate.pass_index == 3 && candidate.previous_pass_index == 2 &&
           candidate.previous_state == RenderResourceState::ShaderRead &&
           candidate.next_state == RenderResourceState::StorageReadWrite;
  });
  ASSERT_NE(transition, plan.transitions.end());
  EXPECT_TRUE(transition->memory_dependency);
}

TEST(RenderGraph, AmbientOcclusionDoesNotRequireDepthPyramid) {
  RenderGraph graph;
  AddDefaultRasterCameraResources(graph);
  AddAdvancedCameraResources(graph);
  graph.AddPass(
      {RenderPassNames::deferred_geometry,
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_depth, RenderResourceUsage::Write, RenderResourceState::DepthAttachment},
        {RenderResourceNames::camera_g_buffer, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
      [](const RenderGraphExecutionContext&) {
      });
  graph.AddPass(AmbientOcclusionPass::CreateDescriptor(), [](const RenderGraphExecutionContext&) {
  });
  graph.AddPass(DeferredComputeLightingPass::CreateDescriptor(true, false), [](const RenderGraphExecutionContext&) {
  });

  const auto plan = graph.Compile();
  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(plan.passes.size(), 3);
  EXPECT_TRUE(plan.passes[1].dependency_indices.empty());
  EXPECT_EQ(plan.passes[1].resource_dependency_indices, std::vector<size_t>({0}));
  EXPECT_EQ(plan.passes[2].dependency_indices, std::vector<size_t>({1}));
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
                [](const RenderGraphExecutionContext&) {
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
                [](const RenderGraphExecutionContext&) {
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
                [](const RenderGraphExecutionContext&) {
                });
  graph.AddPass({"ComputePass",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{compute_target, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}},
                 {"ColorPass"}},
                [](const RenderGraphExecutionContext&) {
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
                [](const RenderGraphExecutionContext&) {
                });
  graph.AddPass({"WriteBloomB",
                 RenderPassQueue::Compute,
                 RenderPassScope::Camera,
                 {{bloom_b, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}},
                 {"WriteBloomA"}},
                [](const RenderGraphExecutionContext&) {
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
                [](const RenderGraphExecutionContext&) {
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
                [](const RenderGraphExecutionContext&) {
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
  AddAdvancedCameraResources(graph);
  AddVolumetricCloudCameraResources(graph);
  AddGaussianSplatCameraResources(graph);

  graph.AddPass(
      {RenderPassNames::deferred_camera,
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
      [](const RenderGraphExecutionContext&) {
      });
  graph.AddPass(VolumetricCloudsPass::CreateRasterDescriptor(RenderPassNames::deferred_camera),
                [](const RenderGraphExecutionContext&) {
                });
  graph.AddPass(GaussianSplatCullPass::CreateDescriptor(RenderPassNames::volumetric_clouds),
                [](const RenderGraphExecutionContext&) {
                });
  graph.AddPass(GaussianSplatSortPass::CreateDescriptor(RenderPassNames::gaussian_splat_cull),
                [](const RenderGraphExecutionContext&) {
                });
  graph.AddPass(GaussianSplatPass::CreateDescriptor(RenderPassNames::gaussian_splat_sort),
                [](const RenderGraphExecutionContext&) {
                });
  graph.AddPass(PostProcessingPass::CreateDescriptor(RenderPassNames::gaussian_splat),
                [](const RenderGraphExecutionContext&) {
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
  AddAdvancedCameraResources(graph);
  AddVolumetricCloudCameraResources(graph);

  graph.AddPass(
      {RenderPassNames::deferred_camera,
       RenderPassQueue::Graphics,
       RenderPassScope::Camera,
       {{RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}}},
      [](const RenderGraphExecutionContext&) {
      });
  graph.AddPass(VolumetricCloudsPass::CreateRasterDescriptor(RenderPassNames::deferred_camera),
                [](const RenderGraphExecutionContext&) {
                });
  graph.AddPass(PostProcessingPass::CreateDescriptor(RenderPassNames::volumetric_clouds),
                [](const RenderGraphExecutionContext&) {
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

  graph.AddPass(RayTracingCameraPass::CreateDescriptor(), [](const RenderGraphExecutionContext&) {
  });
  graph.AddPass(VolumetricCloudsPass::CreateRayTracingDescriptor(RenderPassNames::ray_tracing_camera),
                [](const RenderGraphExecutionContext&) {
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

TEST(RenderGraph, RayCameraOptionalOutputsAreOptInResources) {
  RenderGraph default_graph;
  AddDefaultRayTracingCameraResources(default_graph);
  EXPECT_FALSE(default_graph.HasResource(RenderResourceNames::camera_ray_albedo));
  EXPECT_FALSE(default_graph.HasResource(RenderResourceNames::camera_ray_normal));
  EXPECT_FALSE(default_graph.HasResource(RenderResourceNames::camera_ray_count));

  CameraSettings::RayOutputSettings outputs;
  outputs.albedo = true;
  outputs.normal = true;
  outputs.ray_count = true;
  outputs.path_length = true;
  outputs.time = true;
  outputs.debug = true;

  RenderGraph graph;
  AddDefaultRayTracingCameraResources(graph);
  AddRayCameraOptionalOutputResources(graph, outputs);

  const auto& resources = graph.GetResources();
  const std::array expected_outputs{
      std::pair{RenderResourceNames::camera_ray_albedo, "RGBA8"},
      std::pair{RenderResourceNames::camera_ray_normal, "RGBA16F"},
      std::pair{RenderResourceNames::camera_ray_count, "R32U"},
      std::pair{RenderResourceNames::camera_ray_path_length, "R32U"},
      std::pair{RenderResourceNames::camera_ray_time, "R32U"},
      std::pair{RenderResourceNames::camera_ray_debug, "RGBA32F"},
  };
  for (const auto& [name, format] : expected_outputs) {
    const auto output =
        std::find_if(resources.begin(), resources.end(), [name](const RenderResourceDescriptor& resource) {
          return resource.name == name;
        });
    ASSERT_NE(output, resources.end()) << name;
    EXPECT_EQ(output->format_name, format) << name;
  }
}

TEST(RenderGraph, GaussianSplatOverlayRunsAfterRayTracingClouds) {
  RenderGraph graph;
  AddDefaultRayTracingCameraResources(graph);
  AddVolumetricCloudCameraResources(graph);
  AddGaussianSplatCameraResources(graph);

  graph.AddPass(RayTracingCameraPass::CreateDescriptor(), [](const RenderGraphExecutionContext&) {
  });
  graph.AddPass(VolumetricCloudsPass::CreateRayTracingDescriptor(RenderPassNames::ray_tracing_camera),
                [](const RenderGraphExecutionContext&) {
                });
  graph.AddPass(GaussianSplatCullPass::CreateDescriptor(RenderPassNames::volumetric_clouds),
                [](const RenderGraphExecutionContext&) {
                });
  graph.AddPass(GaussianSplatSortPass::CreateDescriptor(RenderPassNames::gaussian_splat_cull),
                [](const RenderGraphExecutionContext&) {
                });
  graph.AddPass(GaussianSplatPass::CreateOverlayDescriptor(RenderPassNames::gaussian_splat_sort),
                [](const RenderGraphExecutionContext&) {
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

TEST(RenderGraph, PlanCacheTracksHitsMissesAndEvictsByTopology) {
  {
    SCOPED_TRACE("reuses topology without caching callbacks");
    RenderGraphPlanCache cache(4);
    RenderGraphResourceRegistry resources;
    int first_calls = 0;
    int second_calls = 0;
    const auto make_graph = [](int& calls) {
      RenderGraph graph;
      graph.AddPass({"CachedPass", RenderPassQueue::Graphics, RenderPassScope::Camera},
                    [&](const RenderGraphExecutionContext&) {
                      ++calls;
                    });
      return graph;
    };

    auto first = make_graph(first_calls);
    const auto& first_plan = cache.GetOrCompile(first, {1280, 720, 640, 360});
    first.Execute(first_plan, resources);
    auto second = make_graph(second_calls);
    const auto& second_plan = cache.GetOrCompile(second, {1280, 720, 640, 360});
    second.Execute(second_plan, resources);

    EXPECT_EQ(first_calls, 1);
    EXPECT_EQ(second_calls, 1);
    const auto stats = cache.GetStats();
    EXPECT_EQ(stats.entry_count, 1u);
    EXPECT_EQ(stats.hit_count, 1u);
    EXPECT_EQ(stats.miss_count, 1u);
    EXPECT_EQ(stats.compilation_count, 1u);
  }

  {
    SCOPED_TRACE("compile context and exact topology changes miss");
    RenderGraphPlanCache cache(4);
    RenderGraph graph;
    graph.AddResource({"External", RenderResourceType::External, RenderResourceLifetime::Imported});
    graph.AddPass({"Read",
                   RenderPassQueue::Graphics,
                   RenderPassScope::Camera,
                   {{"External", RenderResourceUsage::Read, RenderResourceState::General}}},
                  [](const RenderGraphExecutionContext&) {
                  });

    (void)cache.GetOrCompile(graph, {1280, 720, 640, 360});
    (void)cache.GetOrCompile(graph, {1280, 720, 800, 450});
    RenderGraph changed;
    changed.AddResource(
        {"External", RenderResourceType::External, RenderResourceLifetime::Imported, {}, {}, 1, 1, false, 16});
    changed.AddPass({"Read",
                     RenderPassQueue::Graphics,
                     RenderPassScope::Camera,
                     {{"External", RenderResourceUsage::Read, RenderResourceState::General}}},
                    [](const RenderGraphExecutionContext&) {
                    });
    (void)cache.GetOrCompile(changed, {1280, 720, 800, 450});

    const auto stats = cache.GetStats();
    EXPECT_EQ(stats.entry_count, 3u);
    EXPECT_EQ(stats.hit_count, 0u);
    EXPECT_EQ(stats.miss_count, 3u);
  }

  {
    SCOPED_TRACE("capacity eviction");
    RenderGraphPlanCache cache(2);
    RenderGraph graph;
    graph.AddPass({"Pass", RenderPassQueue::Graphics, RenderPassScope::Camera}, [](const RenderGraphExecutionContext&) {
    });
    (void)cache.GetOrCompile(graph, {1, 1, 1, 1});
    (void)cache.GetOrCompile(graph, {1, 1, 2, 1});
    (void)cache.GetOrCompile(graph, {1, 1, 1, 1});
    (void)cache.GetOrCompile(graph, {1, 1, 3, 1});

    const auto stats = cache.GetStats();
    EXPECT_EQ(stats.entry_count, 2u);
    EXPECT_EQ(stats.hit_count, 1u);
    EXPECT_EQ(stats.miss_count, 3u);
    EXPECT_EQ(stats.eviction_count, 1u);
  }
}

TEST(PlatformFrameScheduling, WaitsOnSlotReuseAndFlushesCaptureTail) {
  const auto platform_header = ReadTextFile(SdkPath("include/Rendering/Platform/Platform.hpp"));
  const auto platform_source = ReadTextFile(SdkPath("src/Platform.cpp"));
  const auto render_layer_header = ReadTextFile(SdkPath("include/Layers/RenderLayer.hpp"));
  const auto render_layer_source = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto editor_source = ReadTextFile(SourcePath("EvoEngine_App/src/EvoEngineEditor.cpp"));

  const auto pre_update = platform_source.find("void Platform::PreUpdate()");
  const auto late_update = platform_source.find("void Platform::LateUpdate()");
  ASSERT_NE(pre_update, std::string::npos);
  ASSERT_NE(late_update, std::string::npos);
  const auto recycle_wait = platform_source.find("WaitForFrameSlotSubmission(current_frame_index", pre_update);
  const auto fence_reset = platform_source.find("vkResetFences", late_update);
  const auto submit = platform_source.find("graphics.main_queue_->Submit", late_update);
  ASSERT_NE(recycle_wait, std::string::npos);
  ASSERT_NE(fence_reset, std::string::npos);
  ASSERT_NE(submit, std::string::npos);
  EXPECT_LT(recycle_wait, late_update);
  EXPECT_LT(fence_reset, submit);
  EXPECT_EQ(platform_source.find("Submitted Frame Fence Wait"), std::string::npos);
  EXPECT_NE(platform_source.find("frame_slot_submitted_[submitted_frame_index] = true", late_update),
            std::string::npos);
  EXPECT_NE(platform_source.find("AccumulateCpuTiming(\"Redundant Fence Wait\""), std::string::npos);
  const auto timestamp_scope =
      platform_source.substr(platform_source.find("GpuTimestampScopeToken Platform::BeginGpuTimestampScope"),
                             platform_source.find("GpuService& Platform::GetGpuService") -
                                 platform_source.find("GpuTimestampScopeToken Platform::BeginGpuTimestampScope"));
  EXPECT_EQ(timestamp_scope.find("VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT"), std::string::npos);
  EXPECT_NE(timestamp_scope.find("vkCmdWriteTimestamp2"), std::string::npos);
  EXPECT_NE(timestamp_scope.find("VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT"), std::string::npos);
  const auto capture_warmup_flush =
      editor_source.find("WaitForFrameSubmissions(\"DDGI Validation Warmup Fence Wait\")");
  const auto capture_completion_flush =
      editor_source.find("WaitForFrameSubmissions(\"Capture Completion Fence Wait\")");
  ASSERT_NE(capture_warmup_flush, std::string::npos);
  ASSERT_NE(capture_completion_flush, std::string::npos);
  EXPECT_LT(capture_warmup_flush, capture_completion_flush);
  const auto camera_source = ReadTextFile(SdkPath("src/Camera.cpp"));
  EXPECT_NE(camera_source.find("WaitForFrameSubmissions(\"Required Camera Resize Fence Wait\")"), std::string::npos);
  EXPECT_NE(render_layer_header.find("std::vector<std::vector<RenderGraphTransientResourceStore>>"), std::string::npos);
  EXPECT_NE(render_layer_source.find("current_frame_transient_resources.clear()"), std::string::npos);
  const auto render_layer_destroy = render_layer_source.find("void RenderLayer::OnDestroy()");
  ASSERT_NE(render_layer_destroy, std::string::npos);
  EXPECT_NE(render_layer_source.find("ray_camera_shader_variant_cache_.reset()", render_layer_destroy),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("ray_tracing_camera_pipeline.reset()", render_layer_destroy), std::string::npos);
  EXPECT_NE(render_layer_source.find("ray_query_camera_pipeline_.reset()", render_layer_destroy), std::string::npos);
  EXPECT_NE(platform_header.find("GetPendingFrameSubmissionCount"), std::string::npos);
  EXPECT_NE(platform_source.find("VkPhysicalDeviceVulkan11Features"), std::string::npos);
  EXPECT_NE(platform_source.find("vk_physical_device_vulkan11_features.shaderDrawParameters = VK_TRUE"),
            std::string::npos);
  EXPECT_EQ(platform_source.find("VkPhysicalDeviceShaderDrawParametersFeatures"), std::string::npos);
  EXPECT_EQ(platform_source.find("VkPhysicalDeviceMultiviewFeatures"), std::string::npos);
  EXPECT_NE(platform_source.find("dynamic_rendering_features.pNext = &vk_physical_device_vulkan12_features"),
            std::string::npos);
  EXPECT_NE(platform_source.find("if (graphics.vk_surface_ != VK_NULL_HANDLE)"), std::string::npos);
}

TEST(PlatformFrameScheduling, SynchronizesAcquiredSwapchainTransitionAtSemaphoreWaitStage) {
  const auto platform_source = ReadTextFile(SdkPath("src/Platform.cpp"));

  const auto undefined_layout = platform_source.find("case VK_IMAGE_LAYOUT_UNDEFINED:");
  const auto generic_undefined_stage =
      platform_source.find("stage_flags = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT", undefined_layout);
  const auto acquired_image = platform_source.find("target_image == swapchain->GetVkImage()");
  const auto acquired_image_stage =
      platform_source.find("source_stage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT", acquired_image);
  const auto acquire_wait = platform_source.find("graphics.image_available_semaphores_[graphics.current_frame_index_]");
  const auto acquire_wait_stage = platform_source.find("VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT", acquire_wait);

  ASSERT_NE(undefined_layout, std::string::npos);
  ASSERT_NE(generic_undefined_stage, std::string::npos);
  ASSERT_NE(acquired_image, std::string::npos);
  ASSERT_NE(acquired_image_stage, std::string::npos);
  ASSERT_NE(acquire_wait, std::string::npos);
  EXPECT_NE(acquire_wait_stage, std::string::npos);
  EXPECT_LT(undefined_layout, generic_undefined_stage);
  EXPECT_LT(acquired_image, acquired_image_stage);
}

TEST(PlatformFrameScheduling, ProtectsMutableResourcesAcrossFrameSlots) {
  const auto render_layer_header = ReadTextFile(SdkPath("include/Layers/RenderLayer.hpp"));
  const auto render_layer_source = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto render_graph_header = ReadTextFile(SdkPath("include/Rendering/RenderGraph.hpp"));
  const auto lighting_header = ReadTextFile(SdkPath("include/Rendering/PBR/Lights.hpp"));
  const auto post_processing_header = ReadTextFile(SdkPath("include/Rendering/PostProcessing/PostProcessingStack.hpp"));
  const auto geometry_source = ReadTextFile(SdkPath("src/GeometryStorage.cpp"));
  const auto texture_source = ReadTextFile(SdkPath("src/Texture2D.cpp"));
  const auto render_texture_source = ReadTextFile(SdkPath("src/RenderTexture.cpp"));
  const auto camera_source = ReadTextFile(SdkPath("src/Camera.cpp"));
  const auto window_source = ReadTextFile(SdkPath("src/WindowLayer.cpp"));
  const auto editor_source = ReadTextFile(SdkPath("src/EditorLayer.cpp"));
  const auto post_processing_pass = ReadTextFile(SdkPath("src/RenderPasses/PostProcessingPass.cpp"));
  const auto ddgi_probe_update_pass = ReadTextFile(SdkPath("src/RenderPasses/DdgiProbeUpdatePass.cpp"));
  const auto ddgi_probe_trace_pass = ReadTextFile(SdkPath("src/RenderPasses/DdgiProbeTracePass.cpp"));

  EXPECT_NE(lighting_header.find("lighting_descriptor_sets_"), std::string::npos);
  EXPECT_NE(post_processing_header.find("class EVOENGINE_API PerFrameDescriptorSet"), std::string::npos);
  EXPECT_NE(post_processing_header.find("duplicate_descriptor_sets"), std::string::npos);
  EXPECT_NE(post_processing_header.find("duplicate_descriptor_set_lists"), std::string::npos);
  EXPECT_NE(post_processing_header.find("PerFrameDescriptorSetList downsampling_descriptor_sets"), std::string::npos);
  EXPECT_NE(post_processing_header.find("struct EVOENGINE_API PostProcessingCameraResources"), std::string::npos);
  EXPECT_NE(render_layer_header.find("struct DdgiReadbackTicket"), std::string::npos);
  EXPECT_NE(render_layer_header.find("history_submission"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("Required DDGI"), std::string::npos);
  EXPECT_NE(render_layer_source.find("Platform::WaitForFrameSubmission(ticket.frame_index"), std::string::npos);
  EXPECT_NE(render_layer_source.find("FrameSubmissionState::Status::Pending"), std::string::npos);
  EXPECT_NE(render_layer_source.find("FrameSubmissionState::Status::Submitted"), std::string::npos);
  EXPECT_NE(render_layer_source.find("FrameSubmissionState::Status::Discarded"), std::string::npos);
  EXPECT_NE(render_layer_source.find("runtime_state.metadata_readback_ticket = {};"), std::string::npos);
  EXPECT_NE(render_layer_source.find("runtime_state.ray_readback_ticket = {};"), std::string::npos);
  EXPECT_NE(render_layer_source.find("runtime_state.probe_debug_ray_samples.clear();"), std::string::npos);
  EXPECT_NE(render_graph_header.find("void RetainAsset(std::shared_ptr<IAsset> asset)"), std::string::npos);
  EXPECT_NE(render_graph_header.find("void RetainBuffer(std::shared_ptr<Buffer> buffer)"), std::string::npos);
  EXPECT_NE(
      render_graph_header.find("void RetainRenderTextureResources(std::shared_ptr<RenderTexture> render_texture)"),
      std::string::npos);
  EXPECT_NE(post_processing_pass.find("RetainAsset(post_processing_stack)"), std::string::npos);
  EXPECT_NE(post_processing_pass.find("RetainPostProcessingResources"), std::string::npos);
  EXPECT_NE(ddgi_probe_update_pass.find("RetainBuffer(parameters.metadata_readback_buffer)"), std::string::npos);
  EXPECT_NE(ddgi_probe_trace_pass.find("RetainBuffer(parameters.selected_ray_readback_buffer)"), std::string::npos);
  const auto ray_process = post_processing_pass.find("post_processing_stack->ProcessRayCamera");
  const auto ray_retention = post_processing_pass.find("RetainPostProcessingResources", ray_process);
  const auto raster_process =
      post_processing_pass.find("post_processing_stack->Process(parameters.camera", ray_retention);
  const auto raster_retention = post_processing_pass.find("RetainPostProcessingResources", raster_process);
  ASSERT_NE(ray_process, std::string::npos);
  EXPECT_NE(ray_retention, std::string::npos);
  ASSERT_NE(raster_process, std::string::npos);
  EXPECT_NE(raster_retention, std::string::npos);
  EXPECT_NE(geometry_source.find("particle_data_pending"), std::string::npos);
  EXPECT_NE(texture_source.find("Texture Readback Fence Wait"), std::string::npos);
  EXPECT_NE(render_texture_source.find("Render Texture Readback Fence Wait"), std::string::npos);
  EXPECT_NE(camera_source.find("Camera Destroy Fence Wait"), std::string::npos);
  EXPECT_NE(window_source.find("Screenshot Readback Fence Wait"), std::string::npos);
  EXPECT_NE(editor_source.find("Entity Picking Readback Fence Wait"), std::string::npos);
}

TEST(RenderGraph, ImageMemoryBarriersCoverRayTracingAndComputeShaderAccess) {
  const auto utilities = ReadTextFile(SdkPath("src/RenderPasses/RenderPassUtilities.cpp"));
  const auto platform = ReadTextFile(SdkPath("src/Platform.cpp"));
  const auto ray_camera = ReadTextFile(SdkPath("src/RenderPasses/RayTracingCameraPass.cpp"));
  EXPECT_NE(utilities.find("VkImageMemoryBarrier2"), std::string::npos);
  EXPECT_NE(utilities.find("VK_PIPELINE_STAGE_2_RAY_TRACING_SHADER_BIT_KHR"), std::string::npos);
  EXPECT_NE(utilities.find("VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT"), std::string::npos);
  EXPECT_NE(utilities.find("VK_ACCESS_2_SHADER_WRITE_BIT"), std::string::npos);
  EXPECT_NE(utilities.find("case RenderResourceState::TransferDestinationGeneral:"), std::string::npos);
  EXPECT_NE(utilities.find("VK_ACCESS_2_TRANSFER_WRITE_BIT"), std::string::npos);
  EXPECT_NE(platform.find("case VK_IMAGE_LAYOUT_GENERAL:"), std::string::npos);
  EXPECT_NE(platform.find("VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT"), std::string::npos);
  EXPECT_NE(platform.find("VK_PIPELINE_STAGE_ALL_COMMANDS_BIT"), std::string::npos);
  EXPECT_NE(platform.find("(image_aspect & VK_IMAGE_ASPECT_DEPTH_BIT) != 0"), std::string::npos);
  EXPECT_NE(platform.find("VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | "
                          "VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT"),
            std::string::npos);
  EXPECT_NE(platform.find("SelectStageFlagsAccessMask(old_layout, barrier.subresourceRange.aspectMask"),
            std::string::npos);
  EXPECT_NE(platform.find("SelectStageFlagsAccessMask(new_layout, barrier.subresourceRange.aspectMask"),
            std::string::npos);
  const auto shader_read_layout = platform.find("case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:");
  const auto depth_attachment_layout = platform.find("case VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL:");
  ASSERT_NE(shader_read_layout, std::string::npos);
  ASSERT_NE(depth_attachment_layout, std::string::npos);
  ASSERT_LT(shader_read_layout, depth_attachment_layout);
  EXPECT_NE(platform.substr(shader_read_layout, depth_attachment_layout - shader_read_layout)
                .find("VK_PIPELINE_STAGE_ALL_COMMANDS_BIT"),
            std::string::npos);
  EXPECT_NE(ray_camera.find("ApplyRayCameraStorageDependencies"), std::string::npos);
  EXPECT_NE(ray_camera.find("VK_ACCESS_2_MEMORY_WRITE_BIT"), std::string::npos);
  EXPECT_EQ(ray_camera.find("Platform::EverythingBarrier"), std::string::npos);
}
