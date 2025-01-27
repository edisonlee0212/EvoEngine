#include "FruitDescriptor.hpp"

using namespace eco_sys_lab_plugin;

std::shared_ptr<Texture2D> FruitDescriptor::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(
        std::filesystem::absolute(std::filesystem::path("./EcoSysLabResources") / "Icons/FruitDescriptor.png"));
  }
  return thumbnail;
}
