#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "AssetManager.hpp"
#include "EvoEngine_SDK_PCH.hpp"
#include "Material.hpp"
#include "Serialization.hpp"
#include "SubsurfaceProfile.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace evo_engine;

TEST(SubsurfaceProfile, RadialDistributionIsNormalizedAndScalesWithWorldDistance) {
  for (const float distance : {0.001f, 0.1f, 1.0f}) {
    double integral = 0;
    constexpr int samples = 10000;
    const float step = 60.0f * distance / samples;
    for (int i = 0; i < samples; ++i)
      integral += SubsurfaceRadialPdf((i + 0.5f) * step, distance) * step;
    EXPECT_NEAR(integral, 1.0, 1e-5);
    double mean = 0;
    for (int i = 0; i < samples; ++i)
      mean += SampleSubsurfaceRadius((i + 0.5f) / samples, distance) / samples;
    EXPECT_NEAR(mean / distance, 2.5, 0.002);
    for (const float u : {0.0f, 0.25f, 0.5f, 0.9f, 0.999f}) {
      const float radius = SampleSubsurfaceRadius(u, distance);
      const float cdf = 1.0f - 0.25f * std::exp(-radius / distance) - 0.75f * std::exp(-radius / (3.0f * distance));
      EXPECT_NEAR(cdf, u, 1e-6f);
    }
    EXPECT_FLOAT_EQ(SubsurfaceRadialPdf(-1.0f, distance), 0.0f);
  }
}

TEST(SubsurfaceProfile, ThinSheetReflectionAndTransmissionPartitionEnergy) {
  for (const float thickness : {0.0f, 0.0001f, 0.01f, 1.0f}) {
    const auto transmission = SubsurfaceThinTransmission(thickness, {0.001f, 0.002f, 0.003f});
    for (int channel = 0; channel < 3; ++channel) {
      EXPECT_GE(transmission[channel], 0.0f);
      EXPECT_LE(transmission[channel], 1.0f);
      EXPECT_FLOAT_EQ(transmission[channel] + (1.0f - transmission[channel]), 1.0f);
    }
  }
  EXPECT_EQ(SubsurfaceThinTransmission(0.0f, glm::vec3(0.001f)), glm::vec3(1.0f));
}

TEST(SubsurfaceProfile, SharedProfileEditsInvalidateMaterialAndMissingProfileDisablesSss) {
  Application app;
  ApplicationContextScope scope(app);
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.load_default_resources = settings.load_project_assets = settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  app.Initialize(settings);
  const auto profile = AssetManager::CreateTemporaryAsset<SubsurfaceProfile>();
  const auto material = AssetManager::CreateTemporaryAsset<Material>();
  material->subsurface_profile = profile;
  material->subsurface_distance_scale = 2.0f;
  material->material_data.shade_material.subsurface_weight = 0.8f;
  material->material_data.shade_material.ior = 1.8f;
  profile->ior = 1.3f;
  const auto first = material->BuildGltfMaterialData();
  EXPECT_EQ(first.shade_material.subsurface_distance, profile->scattering_distance * 2.0f);
  EXPECT_FLOAT_EQ(first.shade_material.subsurface_weight, 0.8f);
  EXPECT_FLOAT_EQ(first.shade_material.ior, 1.8f);
  EXPECT_FLOAT_EQ(first.shade_material.subsurface_ior, 1.3f);
  EXPECT_FLOAT_EQ(material->material_data.shade_material.ior, 1.8f);
  const auto version = material->GetVersion();
  material->RefreshSubsurfaceProfile();
  EXPECT_EQ(material->GetVersion(), version);
  profile->scattering_distance *= 3.0f;
  profile->SetUnsaved();
  const auto second = material->BuildGltfMaterialData();
  EXPECT_GT(material->GetVersion(), version);
  EXPECT_EQ(second.shade_material.subsurface_distance, profile->scattering_distance * 2.0f);
  std::vector<AssetRef> refs;
  material->CollectAssetRef(refs);
  EXPECT_EQ(refs.front().GetAssetHandle(), profile->GetHandle());
  material->subsurface_profile.Clear();
  EXPECT_FLOAT_EQ(material->BuildGltfMaterialData().shade_material.subsurface_weight, 0.0f);
  EXPECT_FLOAT_EQ(material->material_data.shade_material.subsurface_weight, 0.8f);
  material->subsurface_profile = profile;
  profile->scattering_distance.x = std::numeric_limits<float>::quiet_NaN();
  profile->SetUnsaved();
  material->subsurface_distance_scale = std::numeric_limits<float>::quiet_NaN();
  const auto sanitized = material->BuildGltfMaterialData().shade_material;
  EXPECT_TRUE(std::isfinite(sanitized.subsurface_distance.x));
  EXPECT_GT(sanitized.subsurface_distance.x, 0.0f);
  app.Terminate();
}

TEST(SubsurfaceProfile, ProfileAndMaterialRoundTripSerialization) {
  Application app;
  ApplicationContextScope scope(app);
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.load_default_resources = settings.load_project_assets = settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  app.Initialize(settings);
  const auto profile = AssetManager::CreateTemporaryAsset<SubsurfaceProfile>();
  profile->albedo = {0.7f, 0.8f, 0.9f};
  YAML::Emitter profile_yaml;
  profile_yaml << YAML::BeginMap;
  profile->Save("profile", profile_yaml);
  profile_yaml << YAML::EndMap;
  const auto restored = AssetManager::CreateTemporaryAsset<SubsurfaceProfile>();
  restored->Load("profile", YAML::Load(profile_yaml.c_str()));
  EXPECT_EQ(restored->albedo, profile->albedo);
  EXPECT_EQ(restored->scattering_distance, profile->scattering_distance);
  Material material;
  material.subsurface_profile = profile;
  material.subsurface_distance_scale = 3.0f;
  material.material_data.shade_material.subsurface_mode = 1;
  material.material_data.shade_material.subsurface_weight = 0.7f;
  material.material_data.shade_material.subsurface_thickness = 0.002f;
  YAML::Emitter yaml;
  yaml << YAML::BeginMap;
  material.Save("material", yaml);
  yaml << YAML::EndMap;
  Material copy;
  copy.Load("material", YAML::Load(yaml.c_str()));
  EXPECT_EQ(copy.subsurface_profile.GetAssetHandle(), profile->GetHandle());
  EXPECT_FLOAT_EQ(copy.subsurface_distance_scale, 3.0f);
  EXPECT_EQ(copy.material_data.shade_material.subsurface_mode, 1u);
  EXPECT_FLOAT_EQ(copy.material_data.shade_material.subsurface_weight, 0.7f);
  EXPECT_FLOAT_EQ(copy.material_data.shade_material.subsurface_thickness, 0.002f);
  app.Terminate();
}
