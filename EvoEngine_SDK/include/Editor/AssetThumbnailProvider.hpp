#pragma once

#include "OffscreenPreviewRenderer.hpp"

#include <memory>
#include <string>

namespace evo_engine {
class IAsset;
class Texture2D;

class AssetThumbnailProvider {
 public:
  [[nodiscard]] static bool SupportsGeneratedThumbnail(const std::string& asset_type_name);
  [[nodiscard]] static std::shared_ptr<Texture2D> GenerateThumbnail(const std::shared_ptr<IAsset>& asset,
                                                                    const OffscreenPreviewSettings& settings = {});
};
}  // namespace evo_engine
