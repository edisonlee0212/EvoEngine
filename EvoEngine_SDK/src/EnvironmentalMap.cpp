#include "EnvironmentalMap.hpp"
#include "EditorLayer.hpp"
#include "ProjectManager.hpp"

using namespace evo_engine;

void EnvironmentalMap::ConstructFromCubemap(const std::shared_ptr<Cubemap>& target_cubemap) {
  light_probe = ProjectManager::CreateTemporaryAsset<LightProbe>();
  light_probe.Get<LightProbe>()->ConstructFromCubemap(target_cubemap);
  reflection_probe = ProjectManager::CreateTemporaryAsset<ReflectionProbe>();
  reflection_probe.Get<ReflectionProbe>()->ConstructFromCubemap(target_cubemap);
}

void EnvironmentalMap::ConstructFromTexture2D(const std::shared_ptr<Texture2D>& target_texture_2d) {
  const auto cubemap = ProjectManager::CreateTemporaryAsset<Cubemap>();
  cubemap->ConvertFromEquirectangularTexture(target_texture_2d);
  light_probe = ProjectManager::CreateTemporaryAsset<LightProbe>();
  light_probe.Get<LightProbe>()->ConstructFromCubemap(cubemap);
  reflection_probe = ProjectManager::CreateTemporaryAsset<ReflectionProbe>();
  reflection_probe.Get<ReflectionProbe>()->ConstructFromCubemap(cubemap);
}

void EnvironmentalMap::ConstructFromRenderTexture(const std::shared_ptr<RenderTexture>& target_render_texture) {
}

bool EnvironmentalMap::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  static AssetRef target_texture;

  if (editor_layer->DragAndDropButton<Cubemap>(target_texture, "Convert from cubemap")) {
    if (const auto tex = target_texture.Get<Cubemap>()) {
      ConstructFromCubemap(tex);
      changed = true;
    }
    target_texture.Clear();
  }

  if (editor_layer->DragAndDropButton<LightProbe>(light_probe, "LightProbe"))
    changed = true;
  if (editor_layer->DragAndDropButton<LightProbe>(reflection_probe, "ReflectionProbe"))
    changed = true;

  return changed;
}
