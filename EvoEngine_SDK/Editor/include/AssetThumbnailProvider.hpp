#pragma once

#include "EvoEngineEditorAPI.hpp"

#include "OffscreenPreviewRenderer.hpp"

#include <memory>
#include <string>

namespace evo_engine {
class EVOENGINE_API IAsset;
class File;
class EVOENGINE_API Texture2D;

class EVOENGINE_EDITOR_API AssetThumbnailProvider {
 public:
  static std::shared_ptr<Texture2D> GetFileThumbnail(const std::shared_ptr<File>& file, bool allow_asset_load = true);
  static void ClearFileThumbnails();
  [[nodiscard]] static bool SupportsGeneratedThumbnail(const std::string& asset_type_name);
  [[nodiscard]] static std::shared_ptr<Texture2D> GenerateThumbnail(const std::shared_ptr<IAsset>& asset,
                                                                    const OffscreenPreviewSettings& settings = {});
};
}  // namespace evo_engine
