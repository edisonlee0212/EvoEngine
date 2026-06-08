#include "AssetThumbnailProvider.hpp"

#include "IAsset.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "OffscreenPreviewRenderer.hpp"
#include "Texture2D.hpp"

using namespace evo_engine;

bool AssetThumbnailProvider::SupportsGeneratedThumbnail(const std::string& asset_type_name) {
  return asset_type_name == "Texture2D" || asset_type_name == "Material" || asset_type_name == "Mesh";
}

std::shared_ptr<Texture2D> AssetThumbnailProvider::GenerateThumbnail(const std::shared_ptr<IAsset>& asset,
                                                                     const OffscreenPreviewSettings& settings) {
  if (!asset) {
    return {};
  }
  if (const auto texture = std::dynamic_pointer_cast<Texture2D>(asset)) {
    return texture->GenerateThumbnailTexture();
  }
  if (const auto material = std::dynamic_pointer_cast<Material>(asset)) {
    return OffscreenPreviewRenderer::RenderMaterial(material, settings);
  }
  if (const auto mesh = std::dynamic_pointer_cast<Mesh>(asset)) {
    return OffscreenPreviewRenderer::RenderMesh(mesh, {}, settings);
  }
  return {};
}
