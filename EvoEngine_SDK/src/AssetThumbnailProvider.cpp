#include "AssetThumbnailProvider.hpp"

#include "IAsset.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "OffscreenPreviewRenderer.hpp"
#include "Prefab.hpp"
#include "Scene.hpp"
#include "Serialization.hpp"
#include "Texture2D.hpp"

using namespace evo_engine;

namespace {
void EnsureDefaultPreviewHandlers() {
  Serialization::RegisterAssetPreviewHandler<Texture2D>(
      [](const std::shared_ptr<Texture2D>& texture, const OffscreenPreviewSettings&) {
        return texture ? texture->GenerateThumbnailTexture() : nullptr;
      },
      {}, "Texture2D");
  Serialization::RegisterAssetPreviewHandler<Material>(
      [](const std::shared_ptr<Material>& material, const OffscreenPreviewSettings& settings) {
        return OffscreenPreviewRenderer::RenderMaterial(material, settings);
      },
      {}, "Material");
  Serialization::RegisterAssetPreviewHandler<Mesh>(
      [](const std::shared_ptr<Mesh>& mesh, const OffscreenPreviewSettings& settings) {
        return OffscreenPreviewRenderer::RenderMesh(mesh, {}, settings);
      },
      {}, "Mesh");
  Serialization::RegisterAssetPreviewHandler<Prefab>(
      [](const std::shared_ptr<Prefab>& prefab, const OffscreenPreviewSettings&) {
        return prefab ? prefab->GenerateThumbnailTexture() : nullptr;
      },
      {}, "Prefab");
  Serialization::RegisterAssetPreviewHandler<Scene>(
      [](const std::shared_ptr<Scene>& scene, const OffscreenPreviewSettings&) {
        return scene ? scene->GenerateThumbnailTexture() : nullptr;
      },
      {}, "Scene");
}
}  // namespace

bool AssetThumbnailProvider::SupportsGeneratedThumbnail(const std::string& asset_type_name) {
  EnsureDefaultPreviewHandlers();
  return Serialization::HasAssetPreviewHandler(asset_type_name) ||
         (Serialization::HasAssetType(asset_type_name) && Serialization::HasAssetPreviewHandler<IAsset>());
}

std::shared_ptr<Texture2D> AssetThumbnailProvider::GenerateThumbnail(const std::shared_ptr<IAsset>& asset,
                                                                     const OffscreenPreviewSettings& settings) {
  EnsureDefaultPreviewHandlers();
  return asset && SupportsGeneratedThumbnail(asset->GetTypeName())
             ? Serialization::GenerateAssetThumbnail(asset, settings)
             : nullptr;
}
