// Reference-loop fixture adapted from Godot gi.cpp::SDFGI::update,
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>
#include "Application.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "EnvironmentalLighting.hpp"
#include "Lights.hpp"
#include "LodGroup.hpp"
#include "Material.hpp"
#include "MeshRenderer.hpp"
#include "ResolvedEnvironmentalLighting.hpp"
#include "Scene.hpp"
#include "SdfgiDebug.hpp"
#include "SdfgiGather.hpp"
#include "SdfgiLight.hpp"
#include "SdfgiPreprocess.hpp"
#include "SdfgiResources.hpp"
#include "SdfgiRuntime.hpp"

using namespace evo_engine;

TEST(SdfgiDebug, FrozenStepAndCommandsRunOncePerSceneBoundary) {
  SdfgiDebugState debug;
  EXPECT_TRUE(debug.BeginFrame(0));
  EXPECT_FALSE(debug.BeginFrame(0));
  debug.frozen = true;
  EXPECT_FALSE(debug.BeginFrame(1));
  debug.single_step = true;
  EXPECT_FALSE(debug.BeginFrame(1));
  EXPECT_TRUE(debug.single_step);
  EXPECT_TRUE(debug.BeginFrame(2));
  EXPECT_FALSE(debug.single_step);
  EXPECT_FALSE(debug.BeginFrame(3));
  debug.reset_history = true;
  EXPECT_TRUE(debug.BeginFrame(4));
  EXPECT_FALSE(debug.BeginFrame(4));
  debug.reset_history = false;
  debug.full_redraw = true;
  EXPECT_TRUE(debug.BeginFrame(5));
  debug.full_redraw = false;
  EXPECT_FALSE(debug.BeginFrame(6));
  debug.frozen = false;
  EXPECT_TRUE(debug.BeginFrame(7));
}

TEST(SdfgiDebug, SelectedCameraDoesNotAdoptUtilityViewsOrChangeTheAnchor) {
  SdfgiRuntime runtime({}, {});
  runtime.Maintain(0, {17, {1, 2, 3}});
  auto& debug = *runtime.debug;
  EXPECT_FALSE(debug.MatchesCamera(1, true, true));
  debug.enabled = true;
  EXPECT_TRUE(debug.MatchesCamera(1, true, true));
  EXPECT_FALSE(debug.MatchesCamera(2, false, true));
  EXPECT_FALSE(debug.MatchesCamera(1, true, false));
  debug.camera_id = 2;
  EXPECT_FALSE(debug.MatchesCamera(1, true, true));
  EXPECT_TRUE(debug.MatchesCamera(2, false, true));
  EXPECT_FALSE(debug.MatchesCamera(2, false, false));
  EXPECT_EQ(runtime.anchor.camera_id, 17u);
  EXPECT_EQ(runtime.maintenance_count, 1u);
  EXPECT_THROW(CaptureSdfgiDebugImage(runtime, "unrendered.png"), std::runtime_error);
}

TEST(SdfgiGather, ReferenceMetadataAndOrdinaryCameraEligibility) {
  SdfgiSettings settings;
  settings.voxel_count_x = 128;
  std::vector<SdfgiCascade> cascades;
  const glm::vec3 anchor(-13, 2, 9);
  ASSERT_TRUE(UpdateSdfgiCascades(settings, anchor, cascades).empty());
  const auto data = BuildSdfgiGatherData(settings, cascades, anchor, 17);
  EXPECT_EQ(data.generation, 17u);
  EXPECT_EQ(data.max_cascades, 4u);
  EXPECT_EQ(data.probe_axis_size, 17);
  EXPECT_FLOAT_EQ(data.anchor_origin[1], 3);
  EXPECT_FLOAT_EQ(data.normal_bias, 1.1f / 8);
  EXPECT_FLOAT_EQ(data.occlusion_clamp[0], 0.9375f);
  EXPECT_FLOAT_EQ(data.occlusion_renormalize[2], 0.25f);
  EXPECT_FLOAT_EQ(data.lightprobe_tex_pixel_size[0], 1.0f / 2312);
  EXPECT_FLOAT_EQ(data.lightprobe_uv_offset[2], 1.0f / 17);
  for (uint32_t c = 0; c < 4; ++c) {
    const auto& metadata = data.cascades[c];
    for (uint32_t axis = 0; axis < 3; ++axis) {
      EXPECT_FLOAT_EQ(metadata.position[axis] + data.anchor_origin[axis],
                      (cascades[c].position[axis] - 64) * cascades[c].cell_size);
      EXPECT_EQ(metadata.probe_world_offset[axis], cascades[c].position[axis] / 8);
    }
    EXPECT_FLOAT_EQ(metadata.to_probe, 1 / (8 * cascades[c].cell_size));
    EXPECT_FLOAT_EQ(metadata.exposure_normalization, 1);
  }
  Application app;
  ApplicationInitializationSettings initialization;
  initialization.allow_empty_project = true;
  initialization.load_default_resources = false;
  initialization.load_project_assets = false;
  initialization.load_project_start_scene = false;
  initialization.enable_runtime_packages = false;
  app.Initialize(initialization);
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto entity = scene->CreateEntity("SDFGI receiver camera");
  const auto camera = scene->GetOrSetPrivateComponent<Camera>(entity).lock();
  auto utility = std::make_shared<Camera>();
  EXPECT_TRUE(IsSdfgiCameraEligible(scene, camera, utility, false, false, false));
  EXPECT_TRUE(IsSdfgiCameraEligible(scene, utility, utility, false, false, false));
  EXPECT_FALSE(IsSdfgiCameraEligible(scene, utility, {}, false, false, false));
  EXPECT_FALSE(IsSdfgiCameraEligible(scene, camera, {}, true, false, false));
  EXPECT_FALSE(IsSdfgiCameraEligible(scene, camera, {}, false, true, false));
  EXPECT_FALSE(IsSdfgiCameraEligible(scene, camera, {}, false, false, true));
  EXPECT_FALSE(IsSdfgiCameraEligible(AssetManager::CreateTemporaryAsset<Scene>(), camera, {}, false, false, false));
  camera->SetEnabled(false);
  EXPECT_FALSE(IsSdfgiCameraEligible(scene, camera, {}, false, false, false));
  app.Terminate();
}

TEST(SdfgiGather, ReflectionCaptureSelectsOnlyPublishedProviderWithoutChangingAnchor) {
  Application app;
  ApplicationContextScope scope{app};
  auto runtime = std::make_shared<SdfgiRuntime>(SdfgiSettings{}, SdfgiCapabilityReport{});
  runtime->anchor = {42, glm::vec3(3, 4, 5)};
  EXPECT_FALSE(SelectSdfgiCaptureResources({}, IndirectGiProvider::AutomaticSdfgi));
  EXPECT_FALSE(SelectSdfgiCaptureResources(runtime, IndirectGiProvider::AutomaticSdfgi));
  runtime->resources = std::make_shared<SdfgiResources>();
  runtime->resources->publication = std::make_shared<SdfgiGatherFrame>();
  EXPECT_FALSE(SelectSdfgiCaptureResources(runtime, IndirectGiProvider::AutomaticSdfgi));
  runtime->published = true;
  EXPECT_EQ(SelectSdfgiCaptureResources(runtime, IndirectGiProvider::AutomaticSdfgi), runtime->resources);
  EXPECT_FALSE(SelectSdfgiCaptureResources(runtime, IndirectGiProvider::AuthoredDdgi));
  EXPECT_FALSE(SelectSdfgiCaptureResources(runtime, IndirectGiProvider::Environment));
  runtime->resources->preprocess_status.failure_flags = kSdfgiFailureSolidOverflow;
  EXPECT_FALSE(SelectSdfgiCaptureResources(runtime, IndirectGiProvider::AutomaticSdfgi));
  runtime->resources->preprocess_status.failure_flags = 0;
  for (auto* failure : {&runtime->resources->voxel_failure, &runtime->resources->preprocess_failure,
                        &runtime->resources->light_failure, &runtime->resources->transport_failure}) {
    *failure = "unavailable";
    EXPECT_FALSE(SelectSdfgiCaptureResources(runtime, IndirectGiProvider::AutomaticSdfgi));
    failure->clear();
  }
  runtime->resources->publication.reset();
  EXPECT_FALSE(SelectSdfgiCaptureResources(runtime, IndirectGiProvider::AutomaticSdfgi));
  EXPECT_EQ(runtime->anchor.camera_id, 42u);
  EXPECT_EQ(runtime->anchor.world_position, glm::vec3(3, 4, 5));
}

TEST(SdfgiScene, LightCapacitySelectionIsBoundedAndIndependentOfInputOrder) {
  for (const bool dynamic : {false, true}) {
    const uint32_t capacity = dynamic ? 128 : 1024;
    std::vector<SdfgiLightInput> lights(capacity + 2);
    for (uint32_t i = 0; i < lights.size(); ++i) {
      lights[i].id = lights.size() - i;
      lights[i].type = SdfgiLightInput::Type::Point;
    }
    if (dynamic)
      lights[0].type = SdfgiLightInput::Type::Directional;
    auto reversed = lights;
    std::reverse(reversed.begin(), reversed.end());
    EXPECT_EQ(BoundSdfgiLightList(lights, dynamic), 2u);
    EXPECT_EQ(BoundSdfgiLightList(reversed, dynamic), 2u);
    ASSERT_EQ(lights.size(), capacity);
    ASSERT_EQ(reversed.size(), capacity);
    for (size_t i = 0; i < capacity; ++i)
      EXPECT_EQ(lights[i].id, reversed[i].id);
    EXPECT_EQ(lights.front().id, dynamic ? capacity + 2 : 1);
    EXPECT_EQ(BoundSdfgiLightList(lights, dynamic), 0u);
  }
}

TEST(SdfgiLighting, CascadeClassificationAndHostPhotometry) {
  std::vector<SdfgiLightInput> inputs(4);
  for (uint32_t i = 0; i < inputs.size(); ++i) {
    inputs[i].id = i + 1;
    inputs[i].type = SdfgiLightInput::Type::Point;
    inputs[i].world_bounds = {glm::vec3(-1), glm::vec3(1)};
    inputs[i].position = {1, 2, 3};
    inputs[i].color = {2, 3, 4};
    inputs[i].attenuation = {1, 0.2f, 0.03f};
    inputs[i].range = 20;
  }
  inputs[0].type = SdfgiLightInput::Type::Directional;
  inputs[0].dynamic = false;
  inputs[0].direction = glm::normalize(glm::vec3(1, 1, 0));
  inputs[1].dynamic = false;
  inputs[2].type = SdfgiLightInput::Type::Spot;
  inputs[2].cos_inner = 0.9f;
  inputs[2].cos_outer = 0.6f;
  inputs[3].world_bounds = {glm::vec3(1000), glm::vec3(1001)};
  SdfgiCascade cascade;
  cascade.cell_size = 1;
  const auto first_cascade = BuildSdfgiCascadeLights(inputs, cascade, 0, 1.5f, 3);
  ASSERT_EQ(first_cascade.data[0].size(), 1u);
  ASSERT_EQ(first_cascade.data[1].size(), 2u);
  EXPECT_EQ(first_cascade.data[1][0].type, 0u);
  EXPECT_FLOAT_EQ(first_cascade.data[1][0].direction[1], glm::normalize(glm::vec3(1, 1.5f, 0)).y);
  const auto& spot = first_cascade.data[1][1];
  EXPECT_EQ(spot.type, 2u);
  EXPECT_FLOAT_EQ(spot.energy, 1);
  EXPECT_FLOAT_EQ(spot.position[1], 3);
  EXPECT_FLOAT_EQ(spot.color[2], 4);
  EXPECT_FLOAT_EQ(spot.radius, 20);
  EXPECT_FLOAT_EQ(spot.host_photometry[1], 0.2f);
  EXPECT_FLOAT_EQ(spot.host_photometry[2], 0.03f);
  EXPECT_FLOAT_EQ(spot.host_photometry[3], 0.9f);
  EXPECT_FLOAT_EQ(spot.cos_spot_angle, 0.6f);
  const auto fourth_cascade = BuildSdfgiCascadeLights(inputs, cascade, 3, 1.5f, 3);
  EXPECT_TRUE(fourth_cascade.data[0].empty());
  ASSERT_EQ(fourth_cascade.data[1].size(), 1u);
  EXPECT_EQ(fourth_cascade.data[1][0].type, 0u);
  EXPECT_EQ(SdfgiSettings{}.positional_light_cascade_count, 8u);
  for (uint32_t limit = 1; limit <= 8; ++limit)
    for (uint32_t index = 0; index < 8; ++index) {
      SCOPED_TRACE(std::to_string(limit) + "/" + std::to_string(index));
      const auto lights = BuildSdfgiCascadeLights(inputs, cascade, index, 1.5f, limit);
      EXPECT_EQ(lights.data[0].size(), index < limit ? 1u : 0u);
      ASSERT_EQ(lights.data[1].size(), index < limit ? 2u : 1u);
      EXPECT_EQ(lights.data[1][0].type, 0u);
      if (index < limit)
        EXPECT_EQ(lights.data[1][1].type, 2u);
    }
}

TEST(SdfgiResources, DescriptorLimitsIncludeTheWholeHostPipeline) {
  Application app;
  auto layout = std::make_shared<DescriptorSetLayout>();
  layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0, 32);
  VkPhysicalDeviceLimits limits{};
  limits.maxBoundDescriptorSets = 6;
  limits.maxDescriptorSetSamplers = limits.maxDescriptorSetSampledImages = 48;
  limits.maxPerStageDescriptorSamplers = limits.maxPerStageDescriptorSampledImages = limits.maxPerStageResources = 48;
  EXPECT_TRUE(SdfgiResources::ValidateDescriptorLimits({layout}, limits).empty());
  EXPECT_FALSE(SdfgiResources::ValidateDescriptorLimits({layout, layout}, limits).empty());
  EXPECT_FALSE(SdfgiResources::ValidateDescriptorLimits({nullptr}, limits).empty());
}

TEST(SdfgiRuntime, WideHorizontalFieldPreservesSpacingAndReferenceSelection) {
  SdfgiSettings reference, wide;
  reference.voxel_count_x = 128;
  EXPECT_EQ(reference.GridSize(), glm::ivec3(128));
  EXPECT_EQ(reference.ProbeSize(), glm::ivec3(17));
  EXPECT_EQ(wide.GridSize(), glm::ivec3(256, 128, 256));
  EXPECT_EQ(wide.ProbeSize(), glm::ivec3(33, 17, 33));
  EXPECT_EQ(wide.SolidCellCapacity(), reference.SolidCellCapacity() * 4);
  EXPECT_FALSE(wide.HasSameLayout(reference));
  EXPECT_FALSE(wide == reference);
  YAML::Emitter out;
  SerializeSdfgiSettings(out, wide);
  SdfgiSettings loaded;
  DeserializeSdfgiSettings(YAML::Load(out.c_str()), loaded);
  EXPECT_EQ(loaded, wide);
  DeserializeSdfgiSettings(YAML::Load("{}"), loaded);
  EXPECT_EQ(loaded.GridSize(), glm::ivec3(256, 128, 256));
  std::vector<SdfgiCascade> a, b;
  ASSERT_TRUE(UpdateSdfgiCascades(reference, glm::vec3(0), a).empty());
  ASSERT_TRUE(UpdateSdfgiCascades(wide, glm::vec3(0), b).empty());
  for (uint32_t c = 0; c < a.size(); ++c) {
    EXPECT_EQ(b[c].size, wide.GridSize());
    EXPECT_EQ(a[c].cell_size, b[c].cell_size);
    const auto before = a[c].WorldBounds(1), after = b[c].WorldBounds(1);
    EXPECT_EQ(after.max - after.min, (before.max - before.min) * glm::vec3(2, 1, 2));
  }
  const auto gather = BuildSdfgiGatherData(wide, b, glm::vec3(0), 1);
  EXPECT_EQ(gather.probe_axis_size, 33);
  EXPECT_EQ(gather.grid_size[1], 128);
  EXPECT_EQ(gather.cascade_probe_size[0], 32);
  EXPECT_EQ(gather.cascade_probe_size[1], 16);
  EXPECT_FLOAT_EQ(gather.cascades[0].to_probe, 1 / (8 * wide.min_cell_size));
  EXPECT_FLOAT_EQ(gather.lightprobe_tex_pixel_size[0], 1.0f / 8712);
  EXPECT_FLOAT_EQ(gather.lightprobe_uv_offset[2], 264.0f / 8712);
  const auto requirements = GetSdfgiImageRequirements(8, 30, 256, 128);
  ASSERT_EQ(requirements.size(), 14u);
  EXPECT_EQ(requirements[0].extent.width, 256u);
  EXPECT_EQ(requirements[0].extent.height, 128u);
  EXPECT_EQ(requirements[9].extent.width, 512u);
  EXPECT_EQ(requirements[9].extent.depth, 2048u);
  EXPECT_EQ(requirements[12].extent.width, 8712u);
  EXPECT_EQ(requirements[12].extent.height, 136u);
  SdfgiSliceLayout slices{wide.GridSize()};
  EXPECT_EQ(slices.Total(), 131072u);
  for (uint32_t axis = 0; axis < 3; ++axis)
    for (uint32_t edge : {0u, 255u}) {
      EXPECT_LT(slices.Pixel(axis, edge, edge, 256), slices.Count(axis));
      EXPECT_LE(slices.Offset(axis) + slices.Count(axis), slices.Total());
    }
  for (const auto anchor : {glm::vec3(2, -2, 2), glm::vec3(-2, 2, -2), glm::vec3(200, 0, -200)}) {
    ASSERT_TRUE(UpdateSdfgiCascades(wide, anchor, b).empty());
    const auto regions = GetSdfgiPendingRegions(b, 1);
    for (uint32_t c = 0; c < b.size(); ++c) {
      uint32_t total = 0;
      for (const auto& region : regions) {
        if (region.cascade != c)
          continue;
        EXPECT_TRUE(glm::all(glm::greaterThan(region.size, glm::ivec3(0))));
        EXPECT_TRUE(glm::all(glm::greaterThanEqual(region.offset, glm::ivec3(0))));
        EXPECT_TRUE(glm::all(glm::lessThanEqual(region.offset + region.size, wide.GridSize())));
        total += region.size.x * region.size.y * region.size.z;
      }
      const auto retained = b[c].size - glm::abs(b[c].dirty_regions);
      EXPECT_EQ(total, b[c].full_redraw ? 256u * 128 * 256 : 256u * 128 * 256 - retained.x * retained.y * retained.z);
    }
  }
}

TEST(SdfgiConfigurable, DefaultsLegacyMigrationAndInvalidCounts) {
  SdfgiSettings loaded;
  for (const char* yaml : {"{}", "wide_horizontal_field: false", "wide_horizontal_field: true"}) {
    DeserializeSdfgiSettings(YAML::Load(yaml), loaded);
    EXPECT_EQ(loaded.GridSize(), glm::ivec3(256, 128, 256));
  }
  DeserializeSdfgiSettings(YAML::Load("voxel_count_x: 80\nwide_horizontal_field: true"), loaded);
  EXPECT_EQ(loaded.GridSize(), glm::ivec3(80, 128, 80));
  DeserializeSdfgiSettings(YAML::Load("voxel_count_y: 144"), loaded);
  EXPECT_EQ(loaded.GridSize(), glm::ivec3(256, 144, 256));
  for (uint32_t invalid : {0u, 63u, 65u, 255u, 257u, UINT32_MAX}) {
    for (bool vertical : {false, true}) {
      SdfgiSettings settings;
      (vertical ? settings.voxel_count_y : settings.voxel_count_x) = invalid;
      EXPECT_FALSE(settings.Validate().empty());
      EXPECT_TRUE(GetSdfgiImageRequirements(4, 30, settings.voxel_count_x, settings.voxel_count_y).empty());
      auto changed = SdfgiSettings{};
      (vertical ? changed.voxel_count_y : changed.voxel_count_x) = 80;
      EXPECT_FALSE(changed.HasSameLayout(SdfgiSettings{}));
    }
  }
  YAML::Emitter out;
  SerializeSdfgiSettings(out, loaded);
  EXPECT_EQ(std::string(out.c_str()).find("wide_horizontal_field"), std::string::npos);
}

TEST(SdfgiConfigurable, AllGridPairsSizesDispatchCoverageAndDiagnosticBounds) {
  for (uint32_t x = 64; x <= 256; x += 16)
    for (uint32_t y = 64; y <= 256; y += 16) {
      SCOPED_TRACE(std::to_string(x) + "/" + std::to_string(y));
      SdfgiSettings settings;
      settings.voxel_count_x = x;
      settings.voxel_count_y = y;
      ASSERT_TRUE(settings.Validate().empty());
      const auto grid = settings.GridSize(), probes = settings.ProbeSize();
      EXPECT_EQ(grid, glm::ivec3(x, y, x));
      EXPECT_EQ(probes, glm::ivec3(x / 8 + 1, y / 8 + 1, x / 8 + 1));
      EXPECT_EQ(settings.SolidCellCapacity(), x * y * x / 4);
      const auto requirements = GetSdfgiImageRequirements(8, 30, x, y);
      ASSERT_EQ(requirements.size(), 14u);
      EXPECT_EQ(requirements[0].extent.height, y);
      EXPECT_EQ(requirements[4].extent.height, y / 2);
      EXPECT_EQ(requirements[9].extent.depth, x * 8);
      EXPECT_EQ(requirements[10].extent.width, probes.x * probes.z);
      EXPECT_EQ(requirements[10].extent.height, probes.y * 16);
      EXPECT_EQ(requirements[12].extent.width, probes.x * probes.z * 8);
      EXPECT_EQ(requirements[12].extent.height, probes.y * 8);
      EXPECT_EQ(requirements[13].extent.height, probes.y);
      const auto steps = SdfgiJumpFloodSteps(grid);
      ASSERT_FALSE(steps.empty());
      EXPECT_EQ(steps.back(), 1u);
      EXPECT_GE(steps.front() * 2, std::max(x, y) / 2);
      EXPECT_LT(steps.front(), std::max(x, y) / 2);
      for (size_t i = 1; i < steps.size(); ++i)
        EXPECT_EQ(steps[i - 1], steps[i] * 2);
      EXPECT_EQ(steps.size(), std::max(x, y) <= 64 ? 5u : std::max(x, y) <= 128 ? 6u : 7u);
      for (const uint32_t step : steps) {
        if (step > 8)
          continue;
        const auto groups = SdfgiJumpFloodGroups(glm::uvec3(grid / 2), step);
        for (uint32_t axis = 0; axis < 3; ++axis) {
          std::vector<uint32_t> hits(grid[axis] / 2);
          for (uint32_t group = 0; group < groups[axis]; ++group)
            for (uint32_t thread = 0; thread < 8; ++thread) {
              const uint32_t cell = group % step + (group / step) * 8 * step + thread * step;
              if (cell < hits.size())
                ++hits[cell];
            }
          EXPECT_TRUE(std::all_of(hits.begin(), hits.end(), [](uint32_t count) {
            return count == 1;
          }));
        }
      }
      std::vector<SdfgiCascade> cascades;
      ASSERT_TRUE(UpdateSdfgiCascades(settings, glm::vec3(0), cascades).empty());
      EXPECT_EQ(cascades[0].size, grid);
      const auto gather = BuildSdfgiGatherData(settings, cascades, glm::vec3(0), 1);
      for (uint32_t axis = 0; axis < 3; ++axis)
        EXPECT_EQ(gather.cascade_probe_size[axis], probes[axis] - 1);
      EXPECT_FLOAT_EQ(gather.lightprobe_tex_pixel_size[1], 1.0f / (probes.y * 8));
      EXPECT_FLOAT_EQ(gather.cascades[0].to_probe, 1 / (8 * settings.min_cell_size));
      SdfgiSliceLayout slices{grid};
      EXPECT_EQ(slices.SliceCount(), std::min(x, y));
      for (uint32_t axis = 0; axis < 3; ++axis) {
        EXPECT_LE(slices.Offset(axis) + slices.Count(axis), slices.Total());
        for (uint32_t u : {0u, 255u})
          for (uint32_t v : {0u, 255u})
            EXPECT_LT(slices.Pixel(axis, u, v, 256), slices.Count(axis));
      }
      YAML::Emitter out;
      SerializeSdfgiSettings(out, settings);
      SdfgiSettings loaded;
      DeserializeSdfgiSettings(YAML::Load(out.c_str()), loaded);
      EXPECT_EQ(loaded, settings);
    }
}

TEST(SdfgiRuntime, DefaultsAndSettingsRoundTrip) {
  SdfgiSettings settings;
  EXPECT_EQ(settings.cascade_count, 4u);
  EXPECT_EQ(settings.positional_light_cascade_count, 8u);
  EXPECT_EQ(settings.voxel_count_x, 256u);
  EXPECT_EQ(settings.voxel_count_y, 128u);
  EXPECT_FLOAT_EQ(settings.min_cell_size, 0.2f);
  EXPECT_EQ(settings.vertical_scale, SdfgiSettings::VerticalScale::Percent75);
  EXPECT_TRUE(settings.use_occlusion);
  EXPECT_EQ(settings.ray_count, 16u);
  EXPECT_EQ(settings.history_size, 30u);
  EXPECT_EQ(settings.light_update_frames, 4u);
  EXPECT_FLOAT_EQ(settings.bounce_feedback, 0.5f);
  EXPECT_TRUE(settings.read_sky_light);
  EXPECT_FLOAT_EQ(settings.energy, 1);
  EXPECT_FLOAT_EQ(settings.normal_bias, 1.1f);
  EXPECT_FLOAT_EQ(settings.probe_bias, 1.1f);
  EXPECT_EQ(settings.anchor_camera_entity, 0u);
  EXPECT_TRUE(settings.Validate().empty());
  settings.voxel_count_x = 80;
  settings.voxel_count_y = 144;
  settings.cascade_count = 8;
  settings.positional_light_cascade_count = 3;
  settings.min_cell_size = 0.4f;
  settings.vertical_scale = SdfgiSettings::VerticalScale::Percent50;
  settings.use_occlusion = false;
  settings.ray_count = 96;
  settings.history_size = 5;
  settings.light_update_frames = 16;
  settings.bounce_feedback = 0;
  settings.read_sky_light = false;
  settings.energy = 2;
  settings.normal_bias = 0.5f;
  settings.probe_bias = 0.7f;
  settings.anchor_camera_entity = 123456789;
  YAML::Emitter out;
  SerializeSdfgiSettings(out, settings);
  SdfgiSettings loaded;
  DeserializeSdfgiSettings(YAML::Load(out.c_str()), loaded);
  EXPECT_TRUE(loaded == settings);
  EXPECT_FALSE(loaded.use_occlusion);
  EXPECT_TRUE(loaded.Validate().empty());
  SdfgiSettings missing;
  DeserializeSdfgiSettings(YAML::Load("{}"), missing);
  EXPECT_TRUE(missing.use_occlusion);
  EXPECT_EQ(missing.positional_light_cascade_count, 8u);
  DeserializeSdfgiSettings(YAML::Load("cascade_count: 4"), missing);
  EXPECT_EQ(missing.positional_light_cascade_count, 8u);
}

TEST(SdfgiRuntime, ReferenceLayoutChangesAndInvalidSettings) {
  const SdfgiSettings defaults;
  auto changed = defaults;
  changed.positional_light_cascade_count = 3;
  EXPECT_TRUE(defaults.HasSameLayout(changed));
  EXPECT_FALSE(defaults == changed);
  EXPECT_TRUE(changed.Validate().empty());
  changed.positional_light_cascade_count = 0;
  EXPECT_FALSE(changed.Validate().empty());
  changed.positional_light_cascade_count = 9;
  EXPECT_FALSE(changed.Validate().empty());
  changed = defaults;
  changed.cascade_count = 2;
  EXPECT_FALSE(defaults.HasSameLayout(changed));
  changed = defaults;
  changed.min_cell_size = 1;
  EXPECT_FALSE(defaults.HasSameLayout(changed));
  changed = defaults;
  changed.vertical_scale = SdfgiSettings::VerticalScale::Percent100;
  EXPECT_FALSE(defaults.HasSameLayout(changed));
  changed = defaults;
  changed.use_occlusion = false;
  EXPECT_FALSE(defaults.HasSameLayout(changed));
  changed = defaults;
  changed.history_size = 10;
  EXPECT_FALSE(defaults.HasSameLayout(changed));
  changed = defaults;
  changed.ray_count = 32;
  changed.light_update_frames = 1;
  changed.energy = 2;
  changed.bounce_feedback = 0;
  EXPECT_TRUE(defaults.HasSameLayout(changed));
  changed.ray_count = 7;
  EXPECT_FALSE(changed.Validate().empty());
  changed = defaults;
  changed.min_cell_size = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(changed.Validate().empty());
  changed = defaults;
  changed.light_update_frames = 3;
  EXPECT_FALSE(changed.Validate().empty());
}

TEST(SdfgiRuntime, AnchorPrecedenceDoesNotDependOnCameraOrderOrFocus) {
  const SdfgiAnchor explicit_camera{1, {1, 2, 3}};
  const SdfgiAnchor main_camera{2, {4, 5, 6}};
  const SdfgiAnchor editor_camera{3, {7, 8, 9}};
  EXPECT_EQ(SelectSdfgiAnchor(explicit_camera, main_camera, editor_camera, true, true, true).source,
            SdfgiAnchorSource::Explicit);
  const auto playing = SelectSdfgiAnchor({}, main_camera, editor_camera, true, true, true);
  EXPECT_EQ(playing.camera_id, main_camera.camera_id);
  EXPECT_TRUE(playing.override_fell_back);
  EXPECT_EQ(SelectSdfgiAnchor({}, main_camera, editor_camera, false, false, true).source,
            SdfgiAnchorSource::EditorScene);
  EXPECT_EQ(SelectSdfgiAnchor({}, main_camera, {}, false, false, false).source, SdfgiAnchorSource::MainCamera);
  EXPECT_EQ(SelectSdfgiAnchor({}, {}, editor_camera, false, true, true).source, SdfgiAnchorSource::EditorScene);
  EXPECT_EQ(SelectSdfgiAnchor({}, {}, {}, true, true, false).camera_id, 0u);
}

TEST(SdfgiRuntime, MaintainsOncePerSceneFrameAndRetainsMissingAnchorPosition) {
  SdfgiCapabilityReport supported;
  supported.checks.push_back({"test capability", true});
  SdfgiRuntime runtime({}, supported);
  const SdfgiAnchor anchor{7, {2, 4, 6}, SdfgiAnchorSource::MainCamera};
  EXPECT_TRUE(runtime.Maintain(1, anchor));
  EXPECT_FALSE(runtime.Maintain(1, {99, {99, 99, 99}}));
  EXPECT_EQ(runtime.maintenance_count, 1u);
  EXPECT_EQ(runtime.anchor.camera_id, 7u);
  EXPECT_FALSE(runtime.published);
  runtime.published = true;
  EXPECT_FALSE(runtime.Maintain(1, {99, {99, 99, 99}}));
  EXPECT_TRUE(runtime.published);
  EXPECT_TRUE(runtime.Maintain(2, {}));
  EXPECT_TRUE(runtime.published);
  EXPECT_TRUE(runtime.missing_anchor);
  EXPECT_EQ(runtime.anchor.world_position, anchor.world_position);
  EXPECT_NE(runtime.fallback_reason.find("stationary"), std::string::npos);
  EXPECT_TRUE(runtime.Maintain(3, anchor));
  EXPECT_FALSE(runtime.published);
  SdfgiRuntime unsupported({}, {});
  EXPECT_TRUE(unsupported.Maintain(1, anchor));
  EXPECT_NE(unsupported.fallback_reason.find("supported=0"), std::string::npos);
  EXPECT_FALSE(unsupported.published);
}

TEST(SdfgiRuntime, ProviderSerializationPreservesLegacyDefaultsWithoutAddingAutomaticData) {
  Application app;
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  app.Initialize(settings);
  EnvironmentalLighting lighting;
  lighting.ddgi_settings.runtime.enabled = true;
  EXPECT_EQ(lighting.indirect_gi_provider, IndirectGiProvider::AuthoredDdgi);
  YAML::Emitter legacy;
  legacy << YAML::BeginMap;
  SerializeEnvironmentalLighting(legacy, lighting);
  legacy << YAML::EndMap;
  auto legacy_node = YAML::Load(legacy.c_str());
  EXPECT_FALSE(legacy_node["sdfgi_settings"]);
  legacy_node.remove("indirect_gi_provider");
  EnvironmentalLighting loaded;
  DeserializeEnvironmentalLighting(legacy_node, loaded);
  EXPECT_EQ(loaded.indirect_gi_provider, IndirectGiProvider::AuthoredDdgi);
  EXPECT_TRUE(loaded.ddgi_settings.runtime.enabled);
  lighting.indirect_gi_provider = IndirectGiProvider::AutomaticSdfgi;
  lighting.sdfgi_settings.anchor_camera_entity = 12;
  YAML::Emitter automatic;
  automatic << YAML::BeginMap;
  SerializeEnvironmentalLighting(automatic, lighting);
  automatic << YAML::EndMap;
  DeserializeEnvironmentalLighting(YAML::Load(automatic.c_str()), loaded);
  EXPECT_EQ(loaded.indirect_gi_provider, IndirectGiProvider::AutomaticSdfgi);
  EXPECT_TRUE(loaded.sdfgi_settings == lighting.sdfgi_settings);
  EXPECT_TRUE(loaded.ddgi_settings.runtime.enabled);
  legacy_node["indirect_gi_provider"] = 999;
  DeserializeEnvironmentalLighting(legacy_node, loaded);
  EXPECT_EQ(loaded.indirect_gi_provider, IndirectGiProvider::Environment);
}

TEST(SdfgiScene, PlacementMatchesReferenceRoundingThresholdsAndDisjointSlabs) {
  SdfgiSettings settings;
  settings.voxel_count_x = 128;
  settings.min_cell_size = 1;
  settings.vertical_scale = SdfgiSettings::VerticalScale::Percent100;
  std::vector<SdfgiCascade> cascades;
  ASSERT_TRUE(UpdateSdfgiCascades(settings, {-4, -4.01f, 4}, cascades).empty());
  ASSERT_EQ(cascades.size(), 4u);
  EXPECT_EQ(cascades[0].position, glm::ivec3(0, -8, 8));
  EXPECT_FLOAT_EQ(cascades[3].cell_size, 8);
  EXPECT_EQ(GetSdfgiPendingRegions(cascades, 1).size(), 4u);
  cascades.clear();
  ASSERT_TRUE(UpdateSdfgiCascades(settings, glm::vec3(0), cascades).empty());
  ASSERT_TRUE(UpdateSdfgiCascades(settings, {-4.99f, 4.99f, 0}, cascades).empty());
  EXPECT_TRUE(GetSdfgiPendingRegions(cascades, 1).empty());
  ASSERT_TRUE(UpdateSdfgiCascades(settings, {-5, 5, 5}, cascades).empty());
  EXPECT_EQ(cascades[0].position, glm::ivec3(-8, 8, 8));
  EXPECT_EQ(cascades[0].dirty_regions, glm::ivec3(8, -8, -8));
  const auto regions = GetSdfgiPendingRegions(cascades, 1);
  ASSERT_EQ(regions.size(), 3u);
  EXPECT_EQ(regions[0].offset, glm::ivec3(0));
  EXPECT_EQ(regions[0].size, glm::ivec3(8, 128, 128));
  EXPECT_EQ(regions[1].offset, glm::ivec3(8, 120, 0));
  EXPECT_EQ(regions[1].size, glm::ivec3(120, 8, 128));
  EXPECT_EQ(regions[2].offset, glm::ivec3(8, 0, 120));
  EXPECT_EQ(regions[2].size, glm::ivec3(120, 120, 8));
  uint32_t dirty_volume = 0;
  for (const auto& region : regions)
    dirty_volume += region.size.x * region.size.y * region.size.z;
  EXPECT_EQ(dirty_volume, 128 * 128 * 128 - 120 * 120 * 120);
  EXPECT_EQ(regions[0].world_bounds.min, glm::vec3(-72, -56, -56));
  const auto block = BuildSdfgiCascadeBlock(cascades);
  EXPECT_EQ(block.data[0].probe_world_offset[0], -1);
  EXPECT_FLOAT_EQ(block.data[0].offset[1], -56);
  EXPECT_FLOAT_EQ(block.data[3].to_cell, 0.125f);
  EXPECT_FLOAT_EQ(block.data[7].to_cell, 0);
  settings.vertical_scale = SdfgiSettings::VerticalScale::Percent75;
  cascades.clear();
  ASSERT_TRUE(UpdateSdfgiCascades(settings, {0, 4, 0}, cascades).empty());
  EXPECT_EQ(cascades[0].position.y, 8);
  EXPECT_FLOAT_EQ(cascades[0].WorldBounds(1.5f).min.y, -56 / 1.5f);
  for (const int shift : {40, 48, 128}) {
    cascades.clear();
    ASSERT_TRUE(UpdateSdfgiCascades(settings, glm::vec3(0), cascades).empty());
    ASSERT_TRUE(UpdateSdfgiCascades(settings, {shift, 0, 0}, cascades).empty());
    EXPECT_EQ(cascades[0].full_redraw, shift >= 48);
  }
}

TEST(SdfgiScene, MovementMatchesPinnedReferenceLoopsAcrossScriptedTeleports) {
  SdfgiSettings settings;
  settings.voxel_count_x = 128;
  settings.min_cell_size = 1;
  settings.vertical_scale = SdfgiSettings::VerticalScale::Percent100;
  std::vector<SdfgiCascade> actual;
  ASSERT_TRUE(UpdateSdfgiCascades(settings, glm::vec3(0), actual).empty());
  auto expected = actual;
  uint32_t random = 17;
  for (int frame = 0; frame < 200; ++frame) {
    glm::vec3 anchor;
    for (int axis = 0; axis < 3; ++axis) {
      random = random * 1664525u + 1013904223u;
      anchor[axis] = static_cast<float>(static_cast<int>(random % 4096) - 2048) / 4;
    }
    for (auto& cascade : expected) {
      cascade.full_redraw = false;
      cascade.dirty_regions = glm::ivec3(0);
      const glm::ivec3 pos(anchor / cascade.cell_size);
      for (int axis = 0; axis < 3; ++axis) {
        while (pos[axis] < cascade.position[axis] - 4) {
          cascade.position[axis] -= 8;
          cascade.dirty_regions[axis] += 8;
        }
        while (pos[axis] > cascade.position[axis] + 4) {
          cascade.position[axis] += 8;
          cascade.dirty_regions[axis] -= 8;
        }
        if (std::abs(cascade.dirty_regions[axis]) >= 128) {
          cascade.full_redraw = true;
          break;
        }
      }
      if (!cascade.full_redraw) {
        uint32_t safe_volume = 1;
        for (int axis = 0; axis < 3; ++axis)
          safe_volume *= 128 - std::abs(cascade.dirty_regions[axis]);
        cascade.full_redraw = 128 * 128 * 128 - safe_volume > safe_volume / 2;
      }
      if (cascade.full_redraw)
        cascade.dirty_regions = glm::ivec3(0);
    }
    ASSERT_TRUE(UpdateSdfgiCascades(settings, anchor, actual).empty());
    for (size_t i = 0; i < actual.size(); ++i) {
      EXPECT_EQ(actual[i].position, expected[i].position) << frame;
      EXPECT_EQ(actual[i].dirty_regions, expected[i].dirty_regions) << frame;
      EXPECT_EQ(actual[i].full_redraw, expected[i].full_redraw) << frame;
    }
  }
  const auto previous = actual[0].position;
  EXPECT_FALSE(UpdateSdfgiCascades(settings, glm::vec3(std::numeric_limits<float>::infinity()), actual).empty());
  EXPECT_EQ(actual[0].position, previous);
}

TEST(SdfgiScene, RegistrySeparatesCoveragePayloadAndReceiverOnlyEdits) {
  SdfgiContributorRegistry registry;
  SdfgiContributor input;
  input.id = {5, 7};
  input.mesh_id = 11;
  input.world_bounds = {glm::vec3(-1), glm::vec3(1)};
  auto dynamic = input;
  dynamic.id.first = 9;
  dynamic.exclusion = SdfgiExclusion::Dynamic;
  registry.Update({dynamic, input});
  ASSERT_EQ(registry.entries.size(), 1u);
  ASSERT_EQ(registry.changes.size(), 1u);
  EXPECT_EQ(registry.changes[0].flags, SdfgiAdded);
  dynamic.transform[3].x = 100;
  registry.Update({input, dynamic});
  EXPECT_TRUE(registry.changes.empty());
  input.transform[3].x = 2;
  input.world_bounds.min.x += 2;
  input.world_bounds.max.x += 2;
  registry.Update({input});
  ASSERT_EQ(registry.changes.size(), 1u);
  EXPECT_EQ(registry.changes[0].flags, SdfgiTransformChanged);
  EXPECT_FLOAT_EQ(registry.changes[0].before->world_bounds.min.x, -1);
  EXPECT_FLOAT_EQ(registry.changes[0].after->world_bounds.min.x, 1);
  const std::vector<SdfgiCascade> cascades{
      {0.01f, glm::ivec3(0)}, {0.01f, glm::ivec3(200, 0, 0)}, {0.01f, glm::ivec3(1000, 0, 0)}};
  EXPECT_EQ(registry.AffectedCascades(cascades, 1),
            (std::vector<uint32_t>{SdfgiTransformChanged, SdfgiTransformChanged, 0}));
  ++input.geometry_version;
  registry.Update({input});
  EXPECT_EQ(registry.changes[0].flags, SdfgiGeometryChanged);
  input.material.base_color.r = 0.5f;
  input.material.emission = glm::vec3(2);
  registry.Update({input});
  EXPECT_EQ(registry.changes[0].flags, SdfgiPayloadChanged);
  input.material.base_color.a = 0.2f;
  registry.Update({input});
  EXPECT_TRUE(registry.changes.empty());
  input.material.masked = true;
  registry.Update({input});
  EXPECT_EQ(registry.changes[0].flags, SdfgiCoverageChanged);
  ++input.material.base_texture.version;
  registry.Update({input});
  EXPECT_EQ(registry.changes[0].flags, SdfgiCoverageChanged | SdfgiPayloadChanged);
  input.material.double_sided = true;
  registry.Update({input});
  EXPECT_EQ(registry.changes[0].flags, SdfgiCoverageChanged);
  input.exclusion = SdfgiExclusion::Dynamic;
  registry.Update({input});
  ASSERT_EQ(registry.changes.size(), 1u);
  EXPECT_EQ(registry.changes[0].flags, SdfgiRemoved);
  EXPECT_FALSE(registry.changes[0].after);
  registry.Update({});
  EXPECT_TRUE(registry.changes.empty());
}

TEST(SdfgiRuntime, EditRoutingRetainsPendingWorkAndBoundsFailureIsNotRepeated) {
  SdfgiSettings settings;
  settings.voxel_count_x = 128;
  settings.cascade_count = 2;
  settings.min_cell_size = 1;
  settings.vertical_scale = SdfgiSettings::VerticalScale::Percent100;
  SdfgiRuntime runtime(settings, {});
  uint32_t frame = 0;
  SdfgiSceneSnapshot snapshot;
  SdfgiContributor input;
  input.id = {1, 1};
  input.world_bounds = {{90, 0, 0}, {94, 4, 4}};
  snapshot.contributors = {input};
  const auto update = [&](const bool anchor = true, const glm::vec3 position = glm::vec3(0)) {
    EXPECT_TRUE(runtime.Maintain(++frame, anchor ? SdfgiAnchor{1, position} : SdfgiAnchor{}));
    runtime.UpdateSceneSnapshot(snapshot);
    if (anchor)
      runtime.PrepareUpdates(true, false);
  };
  update();
  runtime.AcknowledgeChanges(3);
  update();
  EXPECT_TRUE(runtime.pending_regions.empty());
  snapshot.contributors[0].material.emission = glm::vec3(2);
  update();
  EXPECT_EQ(runtime.payload_cascades, 2u);
  EXPECT_FALSE(runtime.cascades[0].full_redraw);
  EXPECT_FALSE(runtime.cascades[1].full_redraw);
  ASSERT_EQ(runtime.pending_regions.size(), 1u);
  EXPECT_EQ(runtime.pending_regions[0].size, glm::ivec3(128));
  update();  // No acknowledgment models a failed preparation; the edit must not be lost.
  EXPECT_EQ(runtime.payload_cascades, 2u);
  runtime.published = true;
  update(false);
  EXPECT_FALSE(runtime.published);
  EXPECT_EQ(runtime.pending_changes[1], SdfgiPayloadChanged);
  update();
  EXPECT_EQ(runtime.payload_cascades, 2u);
  runtime.AcknowledgeChanges(2);
  snapshot.contributors[0].world_bounds = {glm::vec3(-1), glm::vec3(1)};
  snapshot.contributors[0].transform[3].x = -90;
  update();
  EXPECT_TRUE(runtime.cascades[0].full_redraw);
  EXPECT_TRUE(runtime.cascades[1].full_redraw);
  EXPECT_EQ(runtime.payload_cascades, 0u);
  runtime.AcknowledgeChanges(3);
  for (uint32_t kind = 0; kind < 3; ++kind) {
    if (kind == 0)
      ++snapshot.contributors[0].geometry_version;
    if (kind == 1)
      snapshot.contributors[0].material.masked = true;
    if (kind == 2)
      snapshot.contributors[0].material.double_sided = true;
    update();
    EXPECT_TRUE(runtime.cascades[0].full_redraw);
    EXPECT_EQ(runtime.payload_cascades, 0u);
    runtime.AcknowledgeChanges(3);
  }
  snapshot.contributors[0].material.base_color.r = 0.5f;
  update(true, {17, 0, 0});  // Both cascades scroll; an edit plus scrolling needs a full rebuild.
  EXPECT_EQ(runtime.payload_cascades, 0u);
  EXPECT_TRUE(runtime.cascades[0].full_redraw);
  runtime.AcknowledgeChanges(3);
  snapshot.contributors[0].exclusion = SdfgiExclusion::InvalidBounds;
  update();
  EXPECT_TRUE(runtime.cascades[0].full_redraw);
  EXPECT_TRUE(runtime.cascades[1].full_redraw);
  EXPECT_NE(runtime.invalidation_reason.find("Unknown"), std::string::npos);
  runtime.AcknowledgeChanges(3);
  update();
  EXPECT_TRUE(runtime.pending_regions.empty());
  snapshot.contributors.clear();
  update();
  EXPECT_TRUE(runtime.pending_regions.empty());
  input.exclusion = SdfgiExclusion::Dynamic;
  snapshot.contributors = {input};
  update();
  EXPECT_TRUE(runtime.pending_regions.empty());
  snapshot.contributors[0].transform[3].x = 100;
  update();
  EXPECT_TRUE(runtime.pending_regions.empty());
  runtime.allocation_attempted = true;
  runtime.resource_failure = "forced pipeline failure";
  update();
  EXPECT_TRUE(runtime.allocation_attempted);
  EXPECT_FALSE(runtime.resource_failure.empty());
  update(false);
  update();
  EXPECT_FALSE(runtime.allocation_attempted);
  EXPECT_TRUE(runtime.resource_failure.empty());
}

TEST(SdfgiScene, MaterialSnapshotIgnoresUnrelatedBrdfAndBindlessIndices) {
  Application app;
  const auto material = std::make_shared<Material>();
  const auto before = SnapshotSdfgiMaterial(material);
  material->material_data.shade_material.pbr_metallic_factor = 0.9f;
  material->material_data.shade_material.pbr_roughness_factor = 0.7f;
  material->material_data.shade_material.normal_texture_scale = 0.4f;
  material->SetUnsaved();
  auto after = SnapshotSdfgiMaterial(material);
  EXPECT_TRUE(before.SameCoverage(after));
  EXPECT_TRUE(before.SamePayload(after));
  material->material_data.texture_infos.resize(2);
  material->material_data.shade_material.pbr_base_color_texture = 1;
  after = SnapshotSdfgiMaterial(material);
  material->material_data.texture_infos[1].index = 991;
  EXPECT_TRUE(after.SamePayload(SnapshotSdfgiMaterial(material)));
  material->material_data.texture_infos[1].tex_coord = 1;
  EXPECT_FALSE(after.SamePayload(SnapshotSdfgiMaterial(material)));
}

TEST(SdfgiScene, LiveSceneUsesStaticBaseLodAndSceneLevelLights) {
  Application app;
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  app.Initialize(settings);
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto material = AssetManager::CreateTemporaryAsset<Material>();
  const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
  std::vector<Vertex> vertices(3);
  vertices[0].position = {-1, -1, 0};
  vertices[1].position = {1, -1, 0};
  vertices[2].position = {0, 1, 0};
  mesh->SetVertices(VertexAttributes{}, vertices, {glm::uvec3(0, 1, 2)});
  const auto create_mesh = [&](bool is_static) {
    const auto entity = scene->CreateEntity("SDFGI contributor");
    scene->SetEntityStatic(entity, is_static);
    const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
    renderer->mesh = mesh;
    renderer->material = material;
    return renderer;
  };
  const auto base = create_mesh(true);
  const auto alternate = create_mesh(true);
  const auto dynamic = create_mesh(false);
  const auto group_entity = scene->CreateEntity("LOD group");
  scene->SetEntityStatic(group_entity, true);
  GlobalTransform transform;
  transform.SetPosition({10, 0, 0});
  scene->SetDataComponent(group_entity, transform);
  const auto group = scene->GetOrSetPrivateComponent<LodGroup>(group_entity).lock();
  group->lods.resize(2);
  group->lods[0].renderers.emplace_back(base);
  group->lods[1].renderers.emplace_back(alternate);
  const auto directional_entity = scene->CreateEntity("Static directional");
  scene->SetEntityStatic(directional_entity, true);
  const auto directional = scene->GetOrSetPrivateComponent<DirectionalLight>(directional_entity).lock();
  directional->diffuse = {0.2f, 0.3f, 0.4f};
  directional->diffuse_brightness = 2;
  const auto point_entity = scene->CreateEntity("Static point");
  scene->SetEntityStatic(point_entity, true);
  const auto point = scene->GetOrSetPrivateComponent<PointLight>(point_entity).lock();
  point->range = 7;
  point->constant = 2;
  point->linear = 3;
  point->quadratic = 4;
  const auto spot_entity = scene->CreateEntity("Dynamic spot");
  scene->SetEntityStatic(spot_entity, false);
  const auto spot = scene->GetOrSetPrivateComponent<SpotLight>(spot_entity).lock();
  spot->inner_degrees = 20;
  spot->outer_degrees = 40;
  ResolvedEnvironmentalLighting lighting;
  lighting.indirect_environment_source.kind = ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::Color;
  lighting.indirect_environment_source.color = {0.1f, 0.2f, 0.3f};
  lighting.environment_lighting_intensity = 2;
  auto snapshot = SnapshotSdfgiScene(scene, lighting);
  SdfgiContributorRegistry registry;
  registry.Update(snapshot.contributors);
  ASSERT_EQ(registry.entries.size(), 1u);
  EXPECT_EQ(registry.entries.begin()->second.id.first, base->GetHandle().GetValue());
  EXPECT_EQ(registry.entries.begin()->second.world_bounds.min, glm::vec3(9, -1, 0));
  EXPECT_EQ(snapshot.excluded[SdfgiExclusion::Dynamic], 1u);
  ASSERT_EQ(snapshot.lights.size(), 3u);
  EXPECT_TRUE(snapshot.lights[0].dynamic);
  EXPECT_EQ(snapshot.lights[0].color, glm::vec3(0.4f, 0.6f, 0.8f));
  EXPECT_EQ(snapshot.lights[0].direction, glm::vec3(0, 0, -1));
  EXPECT_FALSE(snapshot.lights[1].dynamic);
  EXPECT_EQ(snapshot.lights[1].attenuation, glm::vec3(2, 3, 4));
  EXPECT_FLOAT_EQ(snapshot.lights[1].range, 7);
  EXPECT_TRUE(snapshot.lights[2].dynamic);
  EXPECT_FLOAT_EQ(snapshot.lights[2].cos_inner, glm::cos(glm::radians(20.0f)));
  EXPECT_FLOAT_EQ(snapshot.lights[2].cos_outer, glm::cos(glm::radians(40.0f)));
  EXPECT_TRUE(snapshot.sky.constant_color);
  EXPECT_EQ(snapshot.sky.color, lighting.indirect_environment_source.color);
  EXPECT_FLOAT_EQ(snapshot.sky.energy, 2);
  group->lod_factor = 999;
  transform.SetPosition({99, 0, 0});
  scene->SetDataComponent(dynamic->GetOwner(), transform);
  snapshot = SnapshotSdfgiScene(scene, lighting);
  registry.Update(snapshot.contributors);
  EXPECT_TRUE(registry.changes.empty());
  point->SetEnabled(false);
  EXPECT_EQ(SnapshotSdfgiScene(scene, lighting).lights.size(), 2u);
  material->material_data.shade_material.alpha_mode = static_cast<int32_t>(GltfAlphaMode::Blend);
  snapshot = SnapshotSdfgiScene(scene, lighting);
  registry.Update(snapshot.contributors);
  EXPECT_TRUE(registry.entries.empty());
  EXPECT_EQ(snapshot.excluded[SdfgiExclusion::Forward], 1u);
  ASSERT_EQ(registry.changes.size(), 1u);
  EXPECT_EQ(registry.changes[0].flags, SdfgiRemoved);
}

TEST(SdfgiScene, AnchorReplacementPreservesOnlyReferenceOverlapAndNoAnchorFreezesCoverage) {
  SdfgiSettings settings;
  settings.voxel_count_x = 128;
  settings.min_cell_size = 1;
  SdfgiRuntime runtime(settings, {});
  ASSERT_TRUE(runtime.Maintain(0, {1, glm::vec3(0)}));
  ASSERT_EQ(runtime.pending_regions.size(), 4u);
  ASSERT_TRUE(runtime.Maintain(1, {2, {5, 0, 0}}));
  EXPECT_TRUE(runtime.anchor_replaced);
  EXPECT_EQ(runtime.cascades[0].position.x, 8);
  EXPECT_FALSE(runtime.cascades[0].full_redraw);
  const auto position = runtime.cascades[0].position;
  ASSERT_TRUE(runtime.Maintain(2, {}));
  EXPECT_EQ(runtime.cascades[0].position, position);
  EXPECT_TRUE(runtime.pending_regions.empty());
  EXPECT_TRUE(runtime.missing_anchor);
  ASSERT_TRUE(runtime.Maintain(3, {2, {1024, 0, 0}}));
  EXPECT_FALSE(runtime.anchor_replaced);
  for (const auto& cascade : runtime.cascades)
    EXPECT_TRUE(cascade.full_redraw);
}
