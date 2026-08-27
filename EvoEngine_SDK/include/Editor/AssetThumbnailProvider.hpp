#pragma once

#include "OffscreenPreviewRenderer.hpp"

#include <memory>
#include <string>

namespace evo_engine {
class EVOENGINE_API IAsset;
class EVOENGINE_API Texture2D;

class EVOENGINE_API AssetThumbnailProvider {
 public:
  [[nodiscard]] static bool SupportsGeneratedThumbnail(const std::string& asset_type_name);
  [[nodiscard]] static std::shared_ptr<Texture2D> GenerateThumbnail(const std::shared_ptr<IAsset>& asset,
                                                                    const OffscreenPreviewSettings& settings = {});
};
}  // namespace evo_engine
