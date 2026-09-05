#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>
#include "Application.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "EnvironmentalLighting.hpp"
#include "SdfgiRuntime.hpp"

using namespace evo_engine;

TEST(SdfgiRuntime, ReferenceDefaultsAndSettingsRoundTrip) {
  SdfgiSettings settings;
  EXPECT_EQ(settings.cascade_count, 4u);
  EXPECT_EQ(settings.kCascadeSize, 128u);
  EXPECT_FLOAT_EQ(settings.min_cell_size, 0.2f);
  EXPECT_EQ(settings.vertical_scale, SdfgiSettings::VerticalScale::Percent75);
  EXPECT_FALSE(settings.use_occlusion);
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
  settings.cascade_count = 8;
  settings.min_cell_size = 0.4f;
  settings.vertical_scale = SdfgiSettings::VerticalScale::Percent50;
  settings.use_occlusion = true;
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
  EXPECT_TRUE(loaded.Validate().empty());
}

TEST(SdfgiRuntime, ReferenceLayoutChangesAndInvalidSettings) {
  const SdfgiSettings defaults;
  auto changed = defaults;
  changed.cascade_count = 2;
  EXPECT_FALSE(defaults.HasSameLayout(changed));
  changed = defaults;
  changed.min_cell_size = 1;
  EXPECT_FALSE(defaults.HasSameLayout(changed));
  changed = defaults;
  changed.vertical_scale = SdfgiSettings::VerticalScale::Percent100;
  EXPECT_FALSE(defaults.HasSameLayout(changed));
  changed = defaults;
  changed.use_occlusion = true;
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
  EXPECT_TRUE(runtime.Maintain(2, {}));
  EXPECT_TRUE(runtime.missing_anchor);
  EXPECT_EQ(runtime.anchor.world_position, anchor.world_position);
  EXPECT_NE(runtime.fallback_reason.find("stationary"), std::string::npos);
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
