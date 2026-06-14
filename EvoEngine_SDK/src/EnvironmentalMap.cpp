#include "EnvironmentalMap.hpp"
#include "AssetManager.hpp"
using namespace evo_engine;

void EnvironmentalMap::BuildSkyIllumination(const SkyIllumination& sky_illumination, uint32_t resolution) {
  const auto cubemap = AssetManager::CreateTemporaryAsset<Cubemap>();
  cubemap->BuildSkyIllumination(sky_illumination, resolution);

  light_probe = AssetManager::CreateTemporaryAsset<LightProbe>();
  light_probe.Get<LightProbe>()->ConstructFromCubemap(cubemap);
  reflection_probe = AssetManager::CreateTemporaryAsset<ReflectionProbe>();
  reflection_probe.Get<ReflectionProbe>()->ConstructFromCubemap(cubemap);
}

void EnvironmentalMap::ConstructFromCubemap(const std::shared_ptr<Cubemap>& target_cubemap) {
  light_probe = AssetManager::CreateTemporaryAsset<LightProbe>();
  light_probe.Get<LightProbe>()->ConstructFromCubemap(target_cubemap);
  reflection_probe = AssetManager::CreateTemporaryAsset<ReflectionProbe>();
  reflection_probe.Get<ReflectionProbe>()->ConstructFromCubemap(target_cubemap);
}

void EnvironmentalMap::ConstructFromTexture2D(const std::shared_ptr<Texture2D>& target_texture_2d) {
  const auto cubemap = AssetManager::CreateTemporaryAsset<Cubemap>();
  cubemap->ConvertFromEquirectangularTexture(target_texture_2d);
  light_probe = AssetManager::CreateTemporaryAsset<LightProbe>();
  light_probe.Get<LightProbe>()->ConstructFromCubemap(cubemap);
  reflection_probe = AssetManager::CreateTemporaryAsset<ReflectionProbe>();
  reflection_probe.Get<ReflectionProbe>()->ConstructFromCubemap(cubemap);
}

void EnvironmentalMap::ConstructFromRenderTexture(const std::shared_ptr<RenderTexture>& target_render_texture) {
}
