#include "EnvironmentalMap.hpp"
#include "AssetManager.hpp"
#include "EditorLayer.hpp"
#include "ProjectManager.hpp"
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

bool EnvironmentalMap::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (editor_layer->DragAndDropButton<Cubemap>(inspection_target_texture_, "Convert from Skybox")) {
    if (const auto tex = inspection_target_texture_.Get<Cubemap>()) {
      ConstructFromCubemap(tex);
      changed = true;
    }
    inspection_target_texture_.Clear();
  }

  if (ImGui::TreeNode("Sky illumination")) {
    static bool auto_rebuild = true;
    ImGui::Checkbox("Auto refresh", &auto_rebuild);
    static SkyIllumination sky_illumination{};
    const bool rebuild = sky_illumination.OnInspect(editor_layer);
    if (ImGui::Button("Build") || (auto_rebuild && rebuild)) {
      BuildSkyIllumination(sky_illumination);
      changed = true;
    }
    ImGui::TreePop();
  }

  if (editor_layer->DragAndDropButton<LightProbe>(light_probe, "LightProbe"))
    changed = true;
  if (editor_layer->DragAndDropButton<LightProbe>(reflection_probe, "ReflectionProbe"))
    changed = true;

  return changed;
}
